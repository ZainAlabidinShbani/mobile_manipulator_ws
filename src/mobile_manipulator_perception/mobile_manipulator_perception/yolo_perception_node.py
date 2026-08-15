#!/usr/bin/env python3
"""
Live YOLOv8 detection on the wrist D435i, and a TF for the object to pick.

yolo_perception_node.py
─────────────────────────────────────────────────────────────────────────────
Phase 7.

Pipeline
  /camera/color/image_raw ┐
                          ├─ message_filters.ApproximateTimeSynchronizer
  /camera/depth/image_raw ┘        │
                                   ├─ Ultralytics YOLOv8 (yolov8n.pt) inference
                                   ├─ annotate boxes + class + confidence,
                                   │  cv2.imshow("Live YOLOv8 Target Detection")
                                   └─ best target-class detection ->
                                      back-project its box centre with the
                                      depth value + /camera/color/camera_info
                                      intrinsics -> 3D point
  10 Hz timer  ────────────────────>  TransformBroadcaster
                                      camera_color_optical_frame
                                          -> object_target_frame

Design notes that are easy to get wrong here
  * Depth is registered to colour BY CONSTRUCTION.  gazebo.xacro hangs both
    the colour and the depth sensor off camera_color_frame with the same FOV
    and 640x480 resolution, so depth pixel (u,v) is colour pixel (u,v) and the
    /camera/color/camera_info intrinsics back-project the depth image exactly.
    The recovered point is therefore already in camera_color_optical_frame,
    the frame this node broadcasts from — no extra TF hop, no colour/depth
    baseline error.  Do not "fix" this by pulling depth intrinsics instead.

  * No cv_bridge.  The Humble cv_bridge boost extension is compiled against
    numpy 1.x; this workspace runs numpy 2.x for Ultralytics/torch, and
    importing it prints "_ARRAY_API not found" and yields a broken module.
    sensor_msgs/Image is a flat buffer plus an encoding string, so the four
    encodings that matter are decoded here directly with numpy.

  * The TF is broadcast on its own 10 Hz timer, not from the inference
    callback.  Inference is slower and jittery; the orchestrator wants a
    steady frame.  With no fresh target the broadcast simply stops (per
    spec) rather than freezing a stale pose — see target_timeout.

  * The bench holds three interchangeable targets, so "highest confidence"
    alone flips between them frame to frame.  The node locks onto whichever it
    picked first and keeps it while it stays in view (track_radius).

  * Depth measures the FRONT SURFACE of the object, not its centroid.
    target_radius_m is added along the view ray to recover the centre, which
    is what a grasp planner wants.  Default 0.0375 m = the 75 mm warehouse
    target balls.  Set it to 0.0 for a raw surface point.

Typical use (Gazebo warehouse from Phase 4 already running):
    ros2 run mobile_manipulator_perception yolo_perception_node \
        --ros-args -p use_sim_time:=true
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

import cv2

WINDOW_NAME = 'Live YOLOv8 Target Detection'


# ── sensor_msgs/Image -> numpy, without cv_bridge ────────────────────────────
def color_to_bgr(msg):
    """Decode a colour sensor_msgs/Image into an OpenCV BGR array."""
    if msg.encoding in ('rgb8', 'bgr8'):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else arr.copy()
    if msg.encoding == 'mono8':
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'unsupported colour encoding {msg.encoding!r}')


def depth_to_metres(msg):
    """Decode a depth sensor_msgs/Image into a float32 array of metres."""
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    if msg.encoding == '16UC1':
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return raw.astype(np.float32) * 0.001          # RealSense ships millimetres
    raise ValueError(f'unsupported depth encoding {msg.encoding!r}')


class YoloPerceptionNode(Node):

    def __init__(self):
        super().__init__('yolo_perception_node')

        p = self.declare_parameter
        p('color_topic', '/camera/color/image_raw')
        p('depth_topic', '/camera/depth/image_raw')
        p('camera_info_topic', '/camera/color/camera_info')
        p('model_path', 'yolov8n.pt')
        p('device', '')                       # '' = let Ultralytics choose (cuda if present)
        p('confidence', 0.25)
        p('iou', 0.45)
        p('target_classes', ['sports ball'])
        p('camera_frame', 'camera_color_optical_frame')
        p('target_frame', 'object_target_frame')
        p('broadcast_rate', 10.0)
        p('target_timeout', 1.0)              # s without a detection before TF stops
        p('target_radius_m', 0.0375)          # front surface -> centroid along the ray
        p('track_radius', 0.15)               # lock-on radius [m]; 0 = pure highest-confidence
        p('depth_patch', 5)                   # odd px window median-sampled at the box centre
        p('min_depth', 0.10)
        p('max_depth', 10.0)
        p('sync_queue_size', 10)
        # 0.1 s, not the usual 0.05: the two Gazebo sensors are rendered in
        # separate passes, so measured colour/depth stamp offsets are 0 most of
        # the time but reach one frame period (0.14 s here).  0.05 pairs 83 % of
        # frames, 0.1 pairs 97 %.
        p('sync_slop', 0.1)
        p('show_window', True)
        p('publish_annotated', True)

        g = lambda n: self.get_parameter(n).value                       # noqa: E731
        self.target_classes = {c.strip().lower() for c in g('target_classes')}
        self.camera_frame = g('camera_frame')
        self.target_frame = g('target_frame')
        self.target_timeout = float(g('target_timeout'))
        self.target_radius = float(g('target_radius_m'))
        self.track_radius = float(g('track_radius'))
        self.depth_patch = max(1, int(g('depth_patch')) | 1)            # force odd
        self.min_depth, self.max_depth = float(g('min_depth')), float(g('max_depth'))
        self.confidence, self.iou = float(g('confidence')), float(g('iou'))
        self.show_window = bool(g('show_window'))

        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warn(
                'use_sim_time is false. Against a running Gazebo this stamps TF with '
                'wall time while everything else uses /clock, and tf2_echo will report '
                'extrapolation errors. Relaunch with -p use_sim_time:=true.')

        # ── model ────────────────────────────────────────────────────────────
        from ultralytics import YOLO                                    # slow import
        model_path = self._resolve_model(g('model_path'))
        self.get_logger().info(f'loading YOLOv8 weights: {model_path}')
        self.model = YOLO(model_path)
        self.device = g('device') or None
        self.class_names = self.model.names
        unknown = self.target_classes - {n.lower() for n in self.class_names.values()}
        if unknown:
            self.get_logger().warn(
                f'target_classes not in this model: {sorted(unknown)} — they can never match. '
                f'Model has {len(self.class_names)} classes.')

        # ── state shared between the inference callback and the TF timer ─────
        self.last_point = None          # (x, y, z) in camera_frame
        self.last_label = None
        self.last_stamp = None          # builtin_interfaces/Time of the source frame
        self.last_seen = None           # rclpy Time, for staleness
        self.frames = 0
        self.window_ok = self.show_window
        self.k = None                   # fx, fy, cx, cy

        # ── I/O ──────────────────────────────────────────────────────────────
        self.create_subscription(CameraInfo, g('camera_info_topic'),
                                 self._on_camera_info, qos_profile_sensor_data)

        color_sub = message_filters.Subscriber(self, Image, g('color_topic'),
                                               qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, g('depth_topic'),
                                               qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], int(g('sync_queue_size')), float(g('sync_slop')))
        self.sync.registerCallback(self._on_frame_pair)

        self.annotated_pub = (self.create_publisher(Image, '~/annotated_image', 1)
                              if g('publish_annotated') else None)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / float(g('broadcast_rate')), self._broadcast_target)

        self.get_logger().info(
            f"subscribed to {g('color_topic')} + {g('depth_topic')}; targets="
            f'{sorted(self.target_classes)}; broadcasting '
            f'{self.camera_frame} -> {self.target_frame} at {g("broadcast_rate")} Hz')

    # ── setup helpers ────────────────────────────────────────────────────────
    def _resolve_model(self, model_path):
        """Prefer a checked-in copy of the weights over an implicit download."""
        if os.path.isabs(model_path) and os.path.exists(model_path):
            return model_path
        candidates = [model_path]
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('mobile_manipulator_perception')
            candidates.insert(0, os.path.join(share, 'models', os.path.basename(model_path)))
        except Exception:                                               # noqa: BLE001
            pass
        for c in candidates:
            if os.path.exists(c):
                return c
        self.get_logger().warn(
            f'{model_path} not found locally; Ultralytics will try to download it.')
        return model_path

    def _on_camera_info(self, msg):
        if self.k is None:
            fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            if fx <= 0.0 or fy <= 0.0:
                self.get_logger().error(f'degenerate intrinsics in CameraInfo: k={list(msg.k)}')
                return
            self.k = (fx, fy, cx, cy)
            self.get_logger().info(
                f'intrinsics fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f} '
                f'({msg.width}x{msg.height}, frame {msg.header.frame_id})')

    # ── per-frame pipeline ───────────────────────────────────────────────────
    def _on_frame_pair(self, color_msg, depth_msg):
        try:
            frame = color_to_bgr(color_msg)
            depth = depth_to_metres(depth_msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        if depth.shape[:2] != frame.shape[:2]:
            self.get_logger().error(
                f'depth {depth.shape[:2]} does not match colour {frame.shape[:2]}; the two '
                'Gazebo sensors must share a resolution for the back-projection to hold',
                throttle_duration_sec=5.0)
            return

        detections = self._infer(frame)
        best = self._pick_target(detections, depth)
        self._render(frame, detections, best, color_msg.header)
        self.frames += 1

        if best is None:
            return
        self.last_point = best['point']
        self.last_label = f"{best['label']} {best['conf']:.2f}"
        self.last_stamp = color_msg.header.stamp
        self.last_seen = self.get_clock().now()

    def _infer(self, frame):
        """Run YOLOv8 and return a list of plain dicts."""
        kwargs = dict(conf=self.confidence, iou=self.iou, verbose=False)
        if self.device:
            kwargs['device'] = self.device
        result = self.model.predict(frame, **kwargs)[0]
        out = []
        for box in result.boxes:
            cls = int(box.cls[0])
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            out.append({
                'label': self.class_names.get(cls, str(cls)),
                'conf': float(box.conf[0]),
                'xyxy': (x1, y1, x2, y2),
                'uv': (0.5 * (x1 + x2), 0.5 * (y1 + y2)),
            })
        out.sort(key=lambda d: d['conf'], reverse=True)
        return out

    def _pick_target(self, detections, depth):
        """
        Pick one target-class detection and back-project it.

        Highest confidence wins, EXCEPT that an existing lock is kept while the
        locked object is still in view.  Without that, a bench holding three
        equally plausible targets makes the winner flip frame to frame and
        object_target_frame teleports between objects several times a second —
        useless to a grasp planner, and it shows up as ~0.5 m of apparent
        jitter.  track_radius:=0.0 restores pure per-frame highest-confidence.
        """
        if self.k is None:
            self.get_logger().warn('no CameraInfo yet — cannot back-project',
                                   throttle_duration_sec=5.0)
            return None

        candidates = []                             # confidence-sorted by _infer
        for det in detections:
            if det['label'].lower() not in self.target_classes:
                continue
            u, v = det['uv']
            z = self._sample_depth(depth, u, v)
            if z is None:
                self.get_logger().warn(
                    f"no valid depth at the centre of the {det['label']} box "
                    f'({u:.0f},{v:.0f})', throttle_duration_sec=5.0)
                continue
            fx, fy, cx, cy = self.k
            # Pinhole back-projection, then push along the ray from the measured
            # front surface to the object centroid.
            ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            z_centre = z + self.target_radius / float(np.linalg.norm(ray))
            det['point'] = tuple(ray * z_centre)
            det['depth'] = z
            candidates.append(det)

        if not candidates:
            return None

        if self.track_radius > 0.0 and self._lock_is_fresh():
            previous = np.asarray(self.last_point)
            near = [d for d in candidates
                    if np.linalg.norm(np.asarray(d['point']) - previous) <= self.track_radius]
            if near:
                return near[0]                      # best confidence among the locked object
            self.get_logger().info(
                'locked target left the frame — re-locking onto '
                f"{candidates[0]['label']} ({candidates[0]['conf']:.2f})")
        return candidates[0]

    def _lock_is_fresh(self):
        if self.last_point is None or self.last_seen is None:
            return False
        age = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        return age <= self.target_timeout

    def _sample_depth(self, depth, u, v):
        """Median of the finite, in-range depths in a small patch at (u, v)."""
        h, w = depth.shape[:2]
        cu, cv = int(round(u)), int(round(v))
        if not (0 <= cu < w and 0 <= cv < h):
            return None
        r = self.depth_patch // 2
        patch = depth[max(0, cv - r):min(h, cv + r + 1), max(0, cu - r):min(w, cu + r + 1)]
        good = patch[np.isfinite(patch) & (patch > self.min_depth) & (patch < self.max_depth)]
        return float(np.median(good)) if good.size else None

    # ── output ───────────────────────────────────────────────────────────────
    def _render(self, frame, detections, best, header):
        for det in detections:
            x1, y1, x2, y2 = (int(round(c)) for c in det['xyxy'])
            chosen = best is not None and det is best
            colour = (0, 0, 255) if chosen else (0, 200, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 3 if chosen else 2)
            text = f"{det['label']} {det['conf']:.2f}"
            if chosen:
                x, y, z = det['point']
                text += f'  [{x:+.3f} {y:+.3f} {z:+.3f}] m'
                cv2.drawMarker(frame, (int(round(det['uv'][0])), int(round(det['uv'][1]))),
                               colour, cv2.MARKER_CROSS, 14, 2)
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty = max(th + 4, y1)
            cv2.rectangle(frame, (x1, ty - th - 4), (x1 + tw + 4, ty), colour, -1)
            cv2.putText(frame, text, (x1 + 2, ty - 3), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)

        banner = (f'{len(detections)} detections'
                  if detections else 'no detections — TF not broadcast')
        if best is None and detections:
            banner += ' | no target class in view'
        cv2.putText(frame, banner, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, banner, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1, cv2.LINE_AA)

        if self.window_ok:
            try:
                cv2.imshow(WINDOW_NAME, frame)
                cv2.waitKey(1)
            except cv2.error as exc:
                self.window_ok = False
                self.get_logger().warn(
                    f'cv2.imshow unavailable ({exc.__class__.__name__}); continuing '
                    'headless. The annotated stream is still on ~/annotated_image.')

        if self.annotated_pub is not None:
            msg = Image()
            msg.header = header
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = 3 * msg.width
            msg.data = frame.tobytes()
            self.annotated_pub.publish(msg)

    def _broadcast_target(self):
        """10 Hz TF. Silent while there is no fresh target — that is the spec."""
        if self.last_point is None or self.last_seen is None:
            return
        age = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        if age > self.target_timeout:
            if self.last_point is not None:
                self.get_logger().info(
                    f'target lost ({age:.1f}s since last detection) — TF stopped')
                self.last_point = None
            return
        x, y, z = self.last_point
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.target_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.w = 1.0            # position only; orientation is Phase 8's job
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

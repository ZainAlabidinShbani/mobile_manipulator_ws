#!/usr/bin/env python3
"""
Evidence capture for the post-migration phase re-verification.

WHY THIS EXISTS RATHER THAN ffmpeg/wf-recorder: neither is installed, nor is
grim, gnome-screenshot or ImageMagick.  The only capture primitive on the box
is xwd (single X11 window dumps), and the session is GNOME *Wayland*, so only
XWayland clients (Gazebo GUI, RViz, cv2's imshow window) can be grabbed at all
— native Wayland clients such as gnome-terminal cannot.  OpenCV is already
present for Ultralytics and its VideoWriter carries its own encoder, so frames
are collected here and encoded with cv2 instead of shelling out.

Subcommands:
  window  <pattern> <out.png>              one frame of an X window
  video   <pattern> <out.mp4> <secs> [fps] screen-record an X window
  topic   <topic>   <out.mp4> <secs> [fps] record a ROS image topic directly
  shot    <topic>   <out.png>              one frame of a ROS image topic

`topic` matters for Gazebo: the demo has to run with gui:=false (the GUI
starves the wrist cameras, ~6 Hz -> <0.3 Hz), so the in-simulation view is
recorded from a world camera over the ros_gz bridge rather than from a window.
"""
import signal
import struct
import subprocess
import sys
import time

import numpy as np

import cv2


# Set by SIGTERM/SIGINT so a recording loop can stop and FINALISE its file.
# cv2.VideoWriter only writes a playable container when release() is called;
# a killed recorder leaves an unfinalised mp4 that ffmpeg reports as "header
# damaged", which is how the first Phase 9 recording was lost.
_STOP = False


def _on_signal(signum, frame):        # noqa: ARG001
    global _STOP
    _STOP = True


signal.signal(signal.SIGTERM, _on_signal)
signal.signal(signal.SIGINT, _on_signal)


def xwd_frame(win_id):
    """One RGB frame from an X window, decoded from xwd's own format."""
    raw = subprocess.run(['xwd', '-id', win_id, '-silent'],
                         capture_output=True, env={'DISPLAY': ':0', 'PATH': '/usr/bin'})
    d = raw.stdout
    if len(d) < 100:
        return None
    h = struct.unpack('>25I', d[:100])
    hdr, width, height, bpl, ncolors = h[0], h[4], h[5], h[12], h[19]
    if width == 0 or height == 0:
        return None
    bypp = bpl // width
    off = hdr + ncolors * 12
    buf = d[off:off + bpl * height]
    if len(buf) < bpl * height:
        return None
    a = np.frombuffer(buf, dtype=np.uint8).reshape(height, bpl)
    a = a[:, :width * bypp].reshape(height, width, bypp)
    return a[:, :, :3]                      # BGR as stored by X


def find_window(pattern):
    out = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True,
                         text=True, env={'DISPLAY': ':0', 'PATH': '/usr/bin'}).stdout
    best = None
    for line in out.splitlines():
        if pattern.lower() not in line.lower():
            continue
        parts = line.split()
        if not parts or not parts[0].startswith('0x'):
            continue
        geom = [p for p in parts if 'x' in p and '+' in p]
        area = 0
        if geom:
            try:
                wh = geom[0].split('+')[0].split('x')
                area = int(wh[0]) * int(wh[1])
            except ValueError:
                area = 0
        if best is None or area > best[1]:
            best = (parts[0], area)
    return best[0] if best else None


def cmd_window(pattern, out):
    wid = find_window(pattern)
    if wid is None:
        print(f'no X window matching {pattern!r}')
        return 1
    f = xwd_frame(wid)
    if f is None:
        print('capture failed')
        return 1
    cv2.imwrite(out, f)
    print(f'{out}  ({f.shape[1]}x{f.shape[0]}) from window {wid}')
    return 0


def cmd_video(pattern, out, secs, fps=8.0):
    wid = find_window(pattern)
    if wid is None:
        print(f'no X window matching {pattern!r}')
        return 1
    first = xwd_frame(wid)
    if first is None:
        print('capture failed')
        return 1
    h, w = first.shape[:2]
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    end = time.monotonic() + secs
    n = 0
    period = 1.0 / fps
    while time.monotonic() < end and not _STOP:
        t0 = time.monotonic()
        f = xwd_frame(wid)
        if f is not None and f.shape[:2] == (h, w):
            vw.write(f)
            n += 1
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    vw.release()
    print(f'{out}  {n} frames  {w}x{h}  ~{n / fps:.0f}s')
    return 0


def _ros_frames(topic, secs, fps, out, single):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    class Sub(Node):
        def __init__(self):
            super().__init__('media_capture')
            self.msg = None
            self.create_subscription(
                Image, topic, self._cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        def _cb(self, m):
            self.msg = m

    def to_bgr(m):
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
        a = a[:, :m.width * 3].reshape(m.height, m.width, 3)
        return a if m.encoding == 'bgr8' else a[:, :, ::-1]

    rclpy.init()
    n = Sub()
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline and n.msg is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.msg is None:
        print(f'no messages on {topic}')
        n.destroy_node()
        rclpy.shutdown()
        return 1
    if single:
        cv2.imwrite(out, to_bgr(n.msg))
        print(f'{out}  ({n.msg.width}x{n.msg.height}) from {topic}')
        n.destroy_node()
        rclpy.shutdown()
        return 0
    h, w = n.msg.height, n.msg.width
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    end = time.monotonic() + secs
    count = 0
    period = 1.0 / fps
    while time.monotonic() < end and not _STOP:
        t0 = time.monotonic()
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.msg is not None:
            vw.write(to_bgr(n.msg))
            count += 1
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    vw.release()
    print(f'{out}  {count} frames  {w}x{h}  ~{count / fps:.0f}s  from {topic}')
    n.destroy_node()
    rclpy.shutdown()
    return 0


def cmd_dual(pat_a, pat_b, out, secs, fps=6.0):
    """
    Record two X windows side by side into one video.

    ffmpeg's x11grab is useless here: this is a GNOME *Wayland* session, so
    XWayland's root window is not composited and grabbing :0 returns black
    (measured mean pixel value 0.02).  Individual windows can still be dumped
    with xwd, so the two panes are grabbed separately and stacked per frame.
    The windows must not overlap on screen — xwd reads the framebuffer region,
    so an occluding window would be captured along with it.
    """
    wa, wb = find_window(pat_a), find_window(pat_b)
    if wa is None or wb is None:
        print(f'missing window: {pat_a}={wa} {pat_b}={wb}')
        return 1
    fa, fb = xwd_frame(wa), xwd_frame(wb)
    if fa is None or fb is None:
        print('capture failed')
        return 1
    h = max(fa.shape[0], fb.shape[0])

    def pad(f):
        out_f = np.zeros((h, f.shape[1], 3), np.uint8)
        out_f[:f.shape[0]] = f
        return out_f

    w = fa.shape[1] + fb.shape[1]
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    end = time.monotonic() + secs
    n = 0
    period = 1.0 / fps
    while time.monotonic() < end and not _STOP:
        t0 = time.monotonic()
        a, b = xwd_frame(wa), xwd_frame(wb)
        if a is not None and b is not None:
            try:
                vw.write(np.hstack((pad(a), pad(b))))
                n += 1
            except ValueError:
                pass
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    vw.release()
    print(f'{out}  {n} frames  {w}x{h}  ~{n / fps:.0f}s')
    return 0


def cmd_topicwin(topic, pattern, out, secs, fps=6.0):
    """
    Record a ROS image topic beside an X window, in one video.

    Used for Phase 7, where the wanted shot is the detector's annotated view
    next to a live tf2_echo.  The detector's own cv2 imshow window cannot be
    dumped — xwd fails it with BadMatch on X_GetImage — but the node mirrors
    the identical annotated frame onto ~/annotated_image precisely so it can
    be read headlessly, so the topic is used as that pane.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    class Sub(Node):
        def __init__(self):
            super().__init__('media_topicwin')
            self.msg = None
            self.create_subscription(
                Image, topic, self._cb,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        def _cb(self, m):
            self.msg = m

    def to_bgr(m):
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
        a = a[:, :m.width * 3].reshape(m.height, m.width, 3)
        return a if m.encoding == 'bgr8' else a[:, :, ::-1]

    wid = find_window(pattern)
    if wid is None:
        print(f'no X window matching {pattern!r}')
        return 1
    rclpy.init()
    n = Sub()
    deadline = time.monotonic() + 40
    while time.monotonic() < deadline and n.msg is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.msg is None:
        print(f'no messages on {topic}')
        n.destroy_node()
        rclpy.shutdown()
        return 1
    left = to_bgr(n.msg)
    right = xwd_frame(wid)
    if right is None:
        print('window capture failed')
        n.destroy_node()
        rclpy.shutdown()
        return 1
    h = max(left.shape[0], right.shape[0])

    def pad(f):
        o = np.zeros((h, f.shape[1], 3), np.uint8)
        o[:f.shape[0]] = f
        return o

    w = left.shape[1] + right.shape[1]
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    end = time.monotonic() + secs
    count = 0
    period = 1.0 / fps
    while time.monotonic() < end and not _STOP:
        t0 = time.monotonic()
        rclpy.spin_once(n, timeout_sec=0.05)
        r = xwd_frame(wid)
        if n.msg is not None and r is not None:
            try:
                vw.write(np.hstack((pad(to_bgr(n.msg)), pad(r))))
                count += 1
            except ValueError:
                pass
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    vw.release()
    print(f'{out}  {count} frames  {w}x{h}  ~{count / fps:.0f}s')
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        sys.exit(2)
    if a[0] == 'window':
        sys.exit(cmd_window(a[1], a[2]))
    if a[0] == 'video':
        sys.exit(cmd_video(a[1], a[2], float(a[3]), float(a[4]) if len(a) > 4 else 8.0))
    if a[0] == 'topic':
        sys.exit(_ros_frames(a[1], float(a[3]), float(a[4]) if len(a) > 4 else 8.0,
                             a[2], False))
    if a[0] == 'dual':
        sys.exit(cmd_dual(a[1], a[2], a[3], float(a[4]),
                          float(a[5]) if len(a) > 5 else 6.0))
    if a[0] == 'topicwin':
        sys.exit(cmd_topicwin(a[1], a[2], a[3], float(a[4]),
                              float(a[5]) if len(a) > 5 else 6.0))
    if a[0] == 'shot':
        sys.exit(_ros_frames(a[1], 0, 1, a[2], True))
    print('unknown subcommand', a[0])
    sys.exit(2)

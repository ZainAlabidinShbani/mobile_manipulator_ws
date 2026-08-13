# Phase 7 — YOLOv8 Perception Node

## Task Prompt for Agent
```text
In mobile_manipulator_perception, write yolo_perception_node.py: a rclpy node
subscribing to /camera/color/image_raw and /camera/depth/image_raw
(message_filters ApproximateTimeSynchronizer to pair them), running
Ultralytics YOLOv8 (yolov8n.pt) inference per synced frame pair, drawing
2D bounding boxes + class label + confidence on the frame and displaying it
via cv2.imshow("Live YOLOv8 Target Detection", annotated_frame). For the
highest-confidence detection matching the target object classes, back-
project the box-center pixel using the depth value at that pixel and the
intrinsics from /camera/color/camera_info into a 3D point in the camera
optical frame, and broadcast it via tf2_ros.TransformBroadcaster as
camera_color_optical_frame -> object_target_frame at 10 Hz. Handle the
zero-detections case without crashing (skip the TF broadcast, keep the
window updating). Verify against the running Gazebo warehouse (Phase 4)
with the robot parked at the pick table: confirm the imshow window renders
correct boxes, and `ros2 run tf2_ros tf2_echo camera_color_optical_frame
object_target_frame` reports a stable transform whose position is within
a few cm of the target object's known spawn position in warehouse.world.
```

---

## Terminal Commands for GNOME Terminal (`gterminal`)

### 1. Build Perception Package
```bash
cd ~/mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mobile_manipulator_perception
source install/setup.bash
```

### 2. Run Perception Node
```bash
ros2 run mobile_manipulator_perception yolo_perception_node
```

### 3. Verify Target Object TF Transform (in new terminal tab)
```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame object_target_frame
```

**Pass Criteria**: OpenCV window renders live bounding box on color image feed; `tf2_echo` prints stable 3D coordinates matching the physical target location on the pick table.

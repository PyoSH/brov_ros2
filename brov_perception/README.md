# brov_perception

ROS 2 Humble package for the BlueROV2 BlueOS H264 stream, checkerboard
intrinsic calibration, and ArUco/AprilTag metric pose estimation.

## Persistent calibration data

Generated calibration is runtime data and is never written into the installed
package share directory. An empty `camera_info_path` or `output_path` resolves
to:

```text
$BROV_DATA_DIR/calibration/camera_intrinsics.yaml
```

If `BROV_DATA_DIR` is unset, the fallback is:

```text
~/.ros/brov/calibration/camera_intrinsics.yaml
```

`config/camera_intrinsics.example.yaml` documents the schema only. Its zero
focal lengths intentionally mark it as uncalibrated.

## Deployed reference marker

The current tank reference is AprilTag `16h5`, integer ID `2`. Its data area is
4 by 4 cells, surrounded by a one-cell black border. With a nominal 70 mm cell,
the detected outer black square is therefore:

```text
(1 border + 4 data + 1 border) * 0.070 m = 0.420 m
```

`config/aruco.yaml` consequently uses:

```yaml
dictionary: DICT_APRILTAG_16h5
marker_id: 2
marker_length_m: 0.42
```

`marker_length_m` is the measured outer-black-edge length. It excludes the
white quiet zone, although that quiet zone must remain physically present for
reliable detection. Replace `0.42` with the finished marker's measured black
edge length if fabrication differs from the nominal value.

The published `/brov/aruco/marker_pose` is the marker pose relative to the
camera optical frame: +x right, +y down, +z forward.

## Camera-to-robot extrinsic

The nominal camera position comes from
`/BROV2_Heavy/Camera_frame` in `brov2_custom_physics.usda`. Only its position is
used; the USD camera rotation is deliberately ignored:

```yaml
base_to_camera_xyz: [0.1575125158, 0.0052856863, 0.0678421631]
base_to_camera_rpy: [-1.5707963268, 0.0, -1.5707963268]
```

The translation assumes `base_link` is the USD `/BROV2_Heavy` root origin and
uses ROS FLU axes (+x forward, +y left, +z up). The RPY is derived separately
from the CV optical convention: camera +z forward, +x right, +y down. It maps
camera +x to base -y, camera +y to base -z, and camera +z to base +x.

With `publish_robot_pose: true`, `/brov/aruco/robot_pose` publishes
`^marker T_base`: the robot `base_link` pose relative to the observed marker.
`publish_robot_tf` remains false so the perception node does not claim the
canonical `marker -> base_link` TF edge.

## Surveyed pool pose

The pool origin is the near/right floor corner. Its axes are +x toward the far
marker wall, +y left while looking along +x, and +z up. AprilTag ID 2 is fixed
to the far wall with its black-square centre at `[3.80, 0.85, 0.24]` metres.
The printed page top points along pool +z and the printed face points into the
pool, along pool -x. The OpenCV debug overlay for this physical print shows the
decoded marker +x axis toward pool +y and decoded +y toward pool -z. In other
words, the decoded marker frame is rotated 180 degrees in-plane relative to the
previous assumption based only on the printed page top. Therefore:

```yaml
pool_to_marker_xyz: [3.8, 0.85, 0.24]
pool_to_marker_quaternion_xyzw: [-0.5, -0.5, 0.5, 0.5]
```

The corresponding marker axes expressed in `pool` are:

```text
marker +X = pool +Y
marker +Y = pool -Z
marker +Z = pool -X
```

For every valid detection the node composes:

```text
^pool T_base = ^pool T_marker * inverse(^camera T_marker)
                                      * inverse(^base T_camera)
```

and publishes the result as `geometry_msgs/PoseStamped` on
`/brov/aruco/robot_pose_pool` with `header.frame_id=pool`. This is an unfiltered
single-frame vision measurement. It does not broadcast `pool -> base_link` and
must not directly replace DVL odometry or command the controller. A later
localization component will quality-gate this measurement and own
`pool -> odom`.

This is a nominal model-derived extrinsic, not an in-water hand-eye
calibration. It is valid only while camera tilt is locked at neutral; moving
the tilt requires a timestamped dynamic camera transform.

## Build

```bash
colcon build --symlink-install --packages-select brov_perception
source install/setup.bash
```

## Run

```bash
ros2 run brov_perception camera_stream_node --ros-args \
  --params-file brov_perception/config/camera.yaml

ros2 run brov_perception checkerboard_calibration_node --ros-args \
  --params-file brov_perception/config/checkerboard.yaml

ros2 run brov_perception aruco_pose_node --ros-args \
  --params-file brov_perception/config/aruco.yaml
```

The camera node publishes `/brov/camera/image_raw` and calibrated
`/brov/camera/camera_info`. The calibration and ArUco nodes consume those
topics; ArUco additionally publishes pose, visibility, debug image, and TF.

The normal integrated run is:

```bash
ros2 launch brov_bringup camera.launch.py aruco:=true
```

## ArUco latest-frame processing backlog

Pool one-shot initialization needs a fresh pose more than it needs every
decoded frame. The current synchronous image callback can fall behind the
camera stream and produce an old pose even while the camera decoder itself is
running at the expected rate. The following changes are required before the
vision path is treated as latency-robust:

1. Use a `KEEP_LAST` image subscription with `depth=1` in the ArUco node.
2. If a new image arrives while detection is running, discard queued older
   images and process the newest available image next. Do not try to catch up
   by processing every historical frame.
3. When `/brov/aruco/debug_image` has no subscribers, skip debug annotation,
   image conversion, and publication so that diagnostics do not consume pose
   processing time.
4. Keep `SUBPIX` as the accuracy-oriented mode, but allow the operator to use
   `corner_refinement: NONE` temporarily during one-shot initialization when
   measured processing latency prevents the freshness gate from passing. This
   degraded mode must be followed by a pose-repeatability check.

The acceptance criterion is based on the age and rate of
`/brov/aruco/robot_pose_pool`, not only the camera decoder FPS. Raising the
localizer message-age limit is not a substitute for latest-frame processing.

Inspect the configured contract and output without enabling vehicle control:

```bash
ros2 param get /brov_aruco_pose_node dictionary
ros2 param get /brov_aruco_pose_node marker_id
ros2 param get /brov_aruco_pose_node marker_length_m
ros2 topic echo /brov/aruco/visible
ros2 topic echo --once /brov/aruco/marker_pose
ros2 topic echo --once /brov/aruco/robot_pose
ros2 topic echo --once /brov/aruco/robot_pose_pool
rqt_image_view /brov/aruco/debug_image
```

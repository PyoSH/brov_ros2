# brov_perception

ROS 2 Humble package for the BlueROV2 BlueOS H264 stream, checkerboard
intrinsic calibration, and ArUco metric pose estimation.

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

## Build

```bash
colcon build --symlink-install --packages-select brov_perception
source install/setup.bash
```

## Run

```bash
ros2 run brov_perception camera_stream_node --ros-args \
  --params-file src/brov_perception/config/camera.yaml

ros2 run brov_perception checkerboard_calibration_node --ros-args \
  --params-file src/brov_perception/config/checkerboard.yaml

ros2 run brov_perception aruco_pose_node --ros-args \
  --params-file src/brov_perception/config/aruco.yaml
```

The camera node publishes `/brov/camera/image_raw` and calibrated
`/brov/camera/camera_info`. The calibration and ArUco nodes consume those
topics; ArUco additionally publishes pose, visibility, debug image, and TF.

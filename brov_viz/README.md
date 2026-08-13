# brov_viz

RViz에서 `pool` 기준 수조 형상, surveyed AprilTag, 그리고
`/brov/aruco/robot_pose_pool`의 raw vision pose를 표시한다.

이 패키지는 시각화 전용이다. TF, waypoint, PWM, arm 명령 또는 control service를
발행하지 않는다. 로봇 표시는 covariance/fusion이 없는 단일 프레임 vision 측정이며,
제어 또는 localization truth로 사용하면 안 된다.

## 실행

먼저 camera/AprilTag를 실행한다.

```bash
make shell
ros2 launch brov_bringup camera.launch.py aruco:=true
```

`make shell`은 rqt용 XQuartz 연결을 준비한다. RViz의 OGRE/OpenGL renderer는
현재 macOS XQuartz 경로의 승인 대상이 아니다. Linux display에서는 아래 명령으로
RViz를 실행하고, Mac에서는 검증된 viewer bridge가 추가되기 전까지
`rviz:=false` headless mode로 scene topic을 확인한다.

새 터미널에서 시각화를 실행한다.

```bash
ros2 launch brov_viz pool_vision.launch.py
```

GUI 없이 scene topic만 검사할 때:

```bash
ros2 launch brov_viz pool_vision.launch.py rviz:=false
ros2 topic echo --once /brov/viz/pool
```

RViz 설정은 `pool` frame 메시지만 Identity transformer로 표시한다. 이는 아직
canonical `pool -> odom -> base_link` TF가 없기 때문이다. 따라서 이 설정에 다른
좌표계의 Marker/Pose/RobotModel을 추가하면 안 된다.

## 표시 의미

- 수조: `[0,4.0] x [0,1.7] x [0,1.1] m`, +X far, +Y left, +Z up
- AprilTag: perception의 `aruco.yaml` survey를 직접 읽으므로 중복 좌표가 없다.
- 로봇: magenta translucent USD-bounds proxy와 base FLU 자세축
- nominal pool 밖의 pose: red
- tag loss 또는 0.5 s pose timeout: robot ghost 삭제

`publish_marker_tf`와 `publish_robot_tf`는 계속 `false`여야 한다. 향후 localization
패키지가 `pool -> odom`을 소유하면 TF transformer와 RobotModel로 이관한다.

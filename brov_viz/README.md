# brov_viz

RViz에서 `pool` 기준 수조 형상, surveyed AprilTag, raw vision pose와
one-shot 정렬 뒤 local odometry로 연속 갱신되는 pool pose를 표시한다.

이 패키지는 시각화 전용이다. TF, waypoint, PWM, arm 명령 또는 control service를
발행하지 않는다. Magenta 로봇은 covariance/fusion이 없는 단일 프레임 vision
측정이며, blue 로봇은 승인된 `pool -> odom`을 local odometry에 적용한 결과다.
둘 다 RViz 표시 자체를 제어 또는 safety truth로 사용하면 안 된다.

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

RViz 설정은 `pool` frame으로 변환해 발행된 Marker만 Identity transformer로
표시한다. 초기화 전에도 수조/원시 비전을 볼 수 있게 하기 위한 제한된 구성이다.
따라서 이 설정에 다른 좌표계의 Marker/Pose/RobotModel을 추가하면 안 된다.

## 표시 의미

- 수조: `[0,4.0] x [0,1.7] x [0,1.1] m`, +X far, +Y left, +Z up
- AprilTag: perception의 `aruco.yaml` survey를 직접 읽으므로 중복 좌표가 없다.
- raw vision 로봇: magenta translucent USD-bounds proxy와 base FLU 자세축
- one-shot aligned odometry 로봇: blue proxy; marker loss 뒤에도 DVL/AHRS로 갱신
- nominal pool 밖의 pose: red
- 각 입력 stream이 0.5 s 이상 stale이면 해당 robot ghost 삭제

`publish_marker_tf`와 `publish_robot_tf`는 계속 `false`여야 한다.
`brov_localization`만 `pool -> odom`을, `brov_base`만 `odom -> base_link`를
소유한다. 추후 URDF가 추가되면 이 canonical tree에서 RobotModel을 표시한다.

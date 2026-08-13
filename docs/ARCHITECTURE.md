# Architecture

## Data flow

```text
BlueOS / ArduSub / DVL
  └─ MAVLink
       └─ brov_base/obs_node
            ├─ telemetry health gate
            ├─ waypoint guidance
            ├─ 16-D observation
            └─ PWM gateway
                  ↑
                  ├─ brov_control/model_based_controller_node
                  └─ brov_control/policy_node

BlueOS H264/RTP
  └─ brov_perception/camera_stream_node
       ├─ checkerboard_calibration_node
       └─ aruco_pose_node
            └─ raw pool-frame PoseStamped
                 └─ brov_viz/pool_scene_node
                      └─ RViz-only pool/marker/robot MarkerArray
```

## Coordinate contract

MAVLink input uses NED world and FRD body coordinates. The policy/controller observation
uses FLU/Z-up body coordinates. The thruster allocation matrix uses SNAME/FRD, so wrench
conversion is explicit before allocation.

The 16-D observation is:

```text
[q_error_wxyz(4), velocity_error_body(3), angular_velocity(3),
 velocity_error_integral(3), quaternion_vector_error_integral(3)]
```

Quaternion error is canonicalized to the `w >= 0` hemisphere.

## Ownership

- `brov_base` owns all hardware access and the only MAVLink connection.
- Controllers are computation-only publishers of `/brov/thruster_pwm`.
- `brov_viz` only republishes visualization markers; it owns no TF, waypoint,
  MAVLink, PWM, arm, or control-service interface.
- `brov_bringup` composes nodes but never starts control automatically.
- Package share is read-only. Mutable calibration/bag/log data belongs under
  `BROV_DATA_DIR`.
- Optional waypoint bounds reject an invalid configured mission before MAVLink
  connection; they are input validation, not a measured-position geofence.

## Planned actuator migration

The current hardware-validated actuator gateway uses RCPassThru and
`RC_CHANNELS_OVERRIDE`. It will remain the fallback while a successor-developed
custom ArduPilot mode is integrated as a separate backend. The ROS-side mode
implementation is intentionally blocked until its firmware source and command
contract are available. See
[ACTUATION_BACKEND_ROADMAP.md](ACTUATION_BACKEND_ROADMAP.md).

## Pool visualization and planned localization

Persistent pool visualization will use a surveyed, Z-up `pool` frame with the
planned ownership chain `pool → odom → base_link`. The current
`start_heading` mission frame remains an ephemeral control frame and must not be
used as a persistent mapping frame. ArUco detection will be treated as a
covariance-bearing localization measurement rather than a competing global TF
authority.

The first visualization stage is implemented by `brov_viz`: it renders nominal
pool geometry, the surveyed AprilTag and a short-lived raw vision robot ghost.
It deliberately uses no canonical TF and removes the ghost when detection is
lost or stale. It is a measurement sanity check, not fused localization.

RViz remains an operator UI: future waypoint edits follow
Draft → Validate → Commit → Shadow → Start and never connect directly to PWM or
control activation. Detailed frame, camera-tilt, mission-editor and acceptance
contracts are in
[RVIZ_POOL_LOCALIZATION_ROADMAP.md](RVIZ_POOL_LOCALIZATION_ROADMAP.md).
Future reconstruction and NBV outputs use the same `pool` frame and are staged
as suggestion-only before any approved execution; see
[NBV_RECONSTRUCTION_ROADMAP.md](NBV_RECONSTRUCTION_ROADMAP.md).

# Architecture

## Data flow

```text
BlueOS / ArduSub / DVL
  └─ MAVLink
       └─ brov_base/obs_node
            ├─ telemetry health gate
            ├─ stamped odom → base_link pose and session identity
            ├─ localization/mission PREPARE → ARM → START gate
            ├─ immutable odom-waypoint guidance
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
                 ├─ brov_viz/pool_scene_node
                 │    ├─ RViz-only raw vision ghost
                 │    └─ one-shot aligned odometry ghost
                 └─ brov_localization/pool_alignment_node
                      ├─ one-shot full-SE(3) pool → odom
                      ├─ pool-frame Odometry (RViz/diagnostics)
                      ├─ atomic aligned Odometry + epoch/session/alignment ID
                      └─ epoch/session/alignment ID + exact transform status
                           └─ brov_mission/mission_manager_node
                                └─ validated pool Path
                                   → immutable odom mission
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

One-shot vision initialization does not add or replace any observation field.
It aligns the outer pool/odometry frame and resolves absolute pool waypoints;
the policy/controller tensor remains the same 16-D contract.

## Ownership

- `brov_base` owns all hardware access and the only MAVLink connection.
- Controllers are computation-only publishers of `/brov/thruster_pwm`.
- `brov_localization` owns only `pool → odom`. It cannot arm or command PWM.
- `brov_mission` validates a pool draft and publishes one immutable mission for
  the current localization epoch/session/alignment ID. It cannot start control.
- `brov_viz` only republishes visualization markers; it owns no TF, waypoint,
  MAVLink, PWM, arm, or control-service interface.
- `brov_bringup` composes nodes but never confirms tilt, initializes/commits,
  prepares, arms, or starts control automatically.
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

## Pool visualization and one-shot localization

Persistent pool localization uses a surveyed, Z-up `pool` frame with the
ownership chain `pool → odom → base_link`. The current
`start_heading` mission frame remains an ephemeral control frame and must not be
used as a persistent mapping frame. ArUco detection will be treated as a
raw localization measurement rather than a competing global TF authority.

The first visualization stage is implemented by `brov_viz`: it renders nominal
pool geometry, the surveyed AprilTag, a short-lived raw vision robot ghost and
the one-shot alignment propagated by local odometry as a separate blue ghost.
After explicit physical camera-neutral confirmation, the separate one-shot
localizer compares synchronized stationary vision/local odometry samples. It
freezes an accepted full-SE(3) transform with a boot-unique `alignment_id` and
invalidates both on reset, odometry session change, or localizer restart. It is
initialization, not continuous fusion.

The Sim2Swim launch uses the same one-shot full-SE(3) alignment as a mandatory
control-start gate while preserving its case-specific `start_heading` mission
and unchanged 16-D RL observation. It deliberately does not require the
position-only resolved pool-mission contract, so case `c` retains its existing
`random_at_waypoint` semantics rather than being silently downgraded.

RViz remains an operator UI: waypoint inputs follow Neutral confirm → Initialize
→ Draft → Validate → Commit → PREPARE/Shadow → ARM → START and never connect
directly to PWM or control activation. Current operator commands and failure gates are in
[POOL_LOCALIZATION_RUNBOOK.md](POOL_LOCALIZATION_RUNBOOK.md). Detailed future
camera-tilt, mission-editor and acceptance contracts are in
[RVIZ_POOL_LOCALIZATION_ROADMAP.md](RVIZ_POOL_LOCALIZATION_ROADMAP.md).
Future reconstruction and NBV outputs use the same `pool` frame and are staged
as suggestion-only before any approved execution; see
[NBV_RECONSTRUCTION_ROADMAP.md](NBV_RECONSTRUCTION_ROADMAP.md).

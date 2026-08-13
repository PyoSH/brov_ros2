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
- `brov_bringup` composes nodes but never starts control automatically.
- Package share is read-only. Mutable calibration/bag/log data belongs under
  `BROV_DATA_DIR`.

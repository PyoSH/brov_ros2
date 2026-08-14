# brov_base

`brov_base` owns the BlueROV2 hardware-facing runtime used by the BROV ROS 2
stack:

- the single MAVLink connection and actuator gateway;
- telemetry validation and the 16-element observation contract;
- waypoint guidance and mission-relative frame handling;
- the packaged BlueROV2 Heavy geometry and T200 thruster model;
- the guarded thruster-map diagnostic utility.

The observation node is safe by default: `send_pwm` and `arm` both default to
`false`. `arm=true` only permits an explicit `/brov/arm_control` request; node
construction and `/brov/start_control` never arm the vehicle. A legacy relative
mission uses ARM -> START. A committed pool mission uses PREPARE -> ARM -> START,
with PREPARE loading a frozen preview before hardware arming.

The node publishes `/brov/odometry/local_with_session`
(`brov_interfaces/OdometrySession`) as the localization input. Each DDS sample
atomically binds local odometry to its odometry-session identity. The standard
`/brov/odometry/local` and latched `/brov/odometry/session_id` topics remain
available for diagnostics only.

```bash
ros2 run brov_base obs_node --ros-args \
  -p connection:=udpout:192.168.2.2:14550 \
  -p send_pwm:=false \
  -p arm:=false
```

For hardware output, verify `success=True` after every applicable lifecycle
service. Normal shutdown closes the output gate first and explicitly disarms:

```bash
ros2 service call /brov/stop_control std_srvs/srv/Trigger '{}'
ros2 service call /brov/disarm_control std_srvs/srv/Trigger '{}'
```

`/brov/stop_control` sends neutral but does not replace the DISARM step.

The diagnostic executable has an additional physical-spin interlock. Run its
help before using it on hardware:

```bash
ros2 run brov_base diag_thruster_map --help
```

The vehicle YAML is available both through
`brov_base.vendor.params.load_brov2_yaml()` and under the installed ament share
directory at `share/brov_base/config/brov2_heavy.yaml`.

Tank missions can enable fail-closed waypoint input validation with
`waypoint_bounds_enabled`, `waypoint_min_xyz`, and `waypoint_max_xyz`. Bounds
are inclusive in the selected mission frame, and equal minimum/maximum values
can lock an axis to a constant depth or centerline. They reject invalid mission
files before MAVLink connection; they do not monitor the measured position and
must not be treated as a runtime geofence.

# brov_base

`brov_base` owns the BlueROV2 hardware-facing runtime used by the BROV ROS 2
stack:

- the single MAVLink connection and actuator gateway;
- telemetry validation and the 16-element observation contract;
- waypoint guidance and mission-relative frame handling;
- the packaged BlueROV2 Heavy geometry and T200 thruster model;
- the guarded thruster-map diagnostic utility.

The observation node is safe by default: `send_pwm` and `arm` both default to
`false`, and control remains frozen until `/brov/start_control` is called.

```bash
ros2 run brov_base obs_node --ros-args \
  -p connection:=udpout:192.168.2.2:14550 \
  -p send_pwm:=false \
  -p arm:=false
```

The diagnostic executable has an additional physical-spin interlock. Run its
help before using it on hardware:

```bash
ros2 run brov_base diag_thruster_map --help
```

The vehicle YAML is available both through
`brov_base.vendor.params.load_brov2_yaml()` and under the installed ament share
directory at `share/brov_base/config/brov2_heavy.yaml`.

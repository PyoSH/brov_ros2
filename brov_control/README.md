# brov_control

`brov_control` contains the two mutually exclusive control backends used by
the BROV runtime:

- `policy_node`: TorchScript policy inference and wrench-to-thruster mapping.
- `model_based_controller_node`: explicit PI/PD wrench control, preview
  diagnostics, operator start/stop services, and watchdog shutdown.

The package does not open MAVLink or write servo channels. It publishes
normalized commands on `/brov/thruster_pwm`; the hardware gateway in
`brov_base` owns final actuation and its safety state.

## Executables

```bash
ros2 run brov_control policy_node --ros-args \
  -p policy_path:=/absolute/path/to/policy.pt

ros2 run brov_control model_based_controller_node --ros-args \
  --params-file $(ros2 pkg prefix brov_control)/share/brov_control/config/model_controller.yaml
```

For model control, first start base control and then explicitly enable this
controller:

```bash
ros2 service call /brov/start_control std_srvs/srv/Trigger '{}'
ros2 service call /brov/model_based/start std_srvs/srv/Trigger '{}'
```

Stop in the reverse order. Never run the policy and model controller at the
same time; the model controller refuses activation if another PWM publisher is
visible.

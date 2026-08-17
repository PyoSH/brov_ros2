# brov_control

`brov_control` contains the two mutually exclusive control backends used by
the BROV runtime:

- `policy_node`: TorchScript policy inference and wrench-to-thruster mapping.
- `model_based_controller_node`: explicit PI/PD wrench control, preview
  diagnostics, operator start/stop services, and watchdog shutdown.

The package does not open MAVLink or write servo channels. Both controllers
publish inspectable previews before activation. The RL node always publishes
`/brov/policy/thruster_pwm_preview`, but publishes normalized commands on
`/brov/thruster_pwm` only while `/brov/control_active` is true. The hardware
gateway in `brov_base` owns final actuation and its safety state.

## Executables

```bash
ros2 run brov_control policy_node --ros-args \
  -p policy_path:=/absolute/path/to/policy.pt

ros2 run brov_control model_based_controller_node --ros-args \
  --params-file $(ros2 pkg prefix brov_control)/share/brov_control/config/model_controller.yaml
```

For a legacy relative mission, explicitly arm the hardware gateway, start base
control, and only then enable the model controller. `arm=true` is permission for
the first service; it never arms during launch:

```bash
ros2 service call /brov/arm_control std_srvs/srv/Trigger '{}'
ros2 service call /brov/start_control std_srvs/srv/Trigger '{}'
ros2 service call /brov/model_based/start std_srvs/srv/Trigger '{}'
```

For a committed pool mission, call `/brov/prepare_control` before ARM and inspect
the frozen target/action preview; the complete lifecycle is PREPARE -> ARM ->
START -> controller start. RL has no controller-specific start service, but it
still requires the base ARM -> START boundary. Before START, its action and PWM
preview remain visible while the actual PWM topic receives no policy command.

The RL node applies an operational envelope after inference.  The generic
profile keeps the exported-policy limits (`action_abs_limit=1`,
`pwm_abs_limit=1`) and disables PWM slew limiting for backwards compatibility.
High-risk profiles such as pool random-attitude v2 must use a dedicated YAML
with stricter per-axis action limits, a lower absolute PWM limit, and a positive
`pwm_slew_rate_per_s`.  The preview contains the post-envelope command, and the
slew state is reset to neutral on every `/brov/control_active` edge.

Close the hardware output gate first, then stop the controller and explicitly
disarm:

```bash
ros2 service call /brov/stop_control std_srvs/srv/Trigger '{}'
ros2 service call /brov/model_based/stop std_srvs/srv/Trigger '{}'
ros2 service call /brov/disarm_control std_srvs/srv/Trigger '{}'
```

Never run the policy and model controller at the same time; the model controller
refuses activation if another PWM publisher is visible.

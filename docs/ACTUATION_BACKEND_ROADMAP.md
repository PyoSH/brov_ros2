# Actuation backend roadmap

## Status

**Planned / blocked on the custom ArduPilot mode handoff.**

The current, hardware-validated actuator path is intentionally retained:

```text
brov_control
  -> /brov/thruster_pwm (8 normalized commands)
  -> brov_base/obs_node
  -> MAVLink RC_CHANNELS_OVERRIDE
  -> SERVO1..8_FUNCTION = RCPassThru
  -> ESC / thrusters
```

`brov_base` temporarily backs up and changes `SERVO1..8_FUNCTION`, isolates the
RC7/RC8 camera options, and restores those parameters during normal shutdown.
This direct PWM path is the verified fallback, but it is not the intended final
ArduPilot integration.

The target is to command the custom ArduPilot mode written by the successor
developer. That mode's source, firmware build, MAVLink command contract, and
parameter set are not yet available, so no placeholder transport or guessed
mode implementation should be added.

## Required handoff information

Implementation can start only after the following items are supplied and
versioned:

1. ArduPilot/ArduSub fork URL, branch, commit SHA, and reproducible firmware
   build/flash instructions.
2. Custom mode name and numeric mode ID, supported vehicle/board, and required
   ArduPilot parameter diff.
3. Input interface: MAVLink message or command ID, target system/component,
   field definitions, units, valid ranges, frame convention, and required rate.
4. Whether the mode accepts body wrench, velocity/attitude setpoints, normalized
   thruster commands, or another control quantity.
5. Entry/exit sequence, arming prerequisites, command acknowledgement, and
   observable mode/health feedback.
6. Command timeout, neutralization, failsafe, disarm, leak/pressure failsafe, and
   companion-computer-loss behavior implemented inside ArduPilot.
7. Mixer ownership, `SERVO1..8_FUNCTION` expectations, thruster order/direction,
   and whether `RC_CHANNELS_OVERRIDE` must be completely disabled.
8. SITL test environment or recorded MAVLink fixtures and at least one known-good
   hardware test procedure.

Coordinate frames and units must be explicit. The current ROS controller uses a
FLU/Z-up body wrench internally, whereas ArduPilot commonly uses NED/FRD-related
conventions. Conversion must live at one documented boundary and be covered by
tests.

## Planned ROS 2 work

After the handoff contract is frozen:

1. Introduce an actuator-backend boundary in `brov_base`; keep
   `RCPassThruBackend` as the existing fallback and add a separate custom-mode
   backend.
2. Add an explicit `actuation_backend` configuration/launch argument. Unknown or
   incompatible values must fail closed; there must never be two active
   actuator backends.
3. Implement custom-mode discovery, mode switching, ACK verification, command
   encoding, feedback monitoring, timeout handling, neutral command, and clean
   exit without guessing undocumented behavior.
4. Keep the single-MAVLink-owner rule: only `brov_base` communicates with the
   autopilot; controller nodes remain computation-only ROS publishers.
5. Add parameters and diagnostics for requested mode, observed mode, command
   age/rate, ACK status, backend health, and failsafe reason.
6. Add unit tests for frame/unit conversion and message encoding, integration
   tests for mode rejection/timeouts, and SITL tests for entry, command loss,
   neutralization, exit, and disarm.
7. Validate in this order: offline fixtures -> SITL -> props-off bench ->
   restrained tank test -> low-authority waypoint test -> full mission.
8. Deprecate RCPassThru only after the new backend passes the same thruster-map,
   stop, crash-recovery, and in-water regression tests.

## Acceptance criteria

The custom-mode backend is ready for normal use only when all of the following
are demonstrated:

- It does not rewrite `SERVO1..8_FUNCTION` during normal operation.
- It does not use thruster `RC_CHANNELS_OVERRIDE` unless the handed-off contract
  explicitly requires and justifies it.
- Mode entry is positively confirmed before non-neutral commands are accepted.
- Stale/missing commands produce a bounded-time neutral or ArduPilot failsafe.
- ROS/container loss cannot leave an indefinitely active thrust command.
- Thruster order, direction, saturation, frames, units, and command rate match
  the firmware contract.
- `/brov/estop`, normal stop, mode rejection, communication loss, and disarm are
  tested on SITL and hardware.
- RCPassThru remains selectable as a recovery backend until the migration is
  explicitly signed off.

Until these conditions are met, demonstrations and experiments continue to use
the existing RCPassThru backend and its current safety runbook.

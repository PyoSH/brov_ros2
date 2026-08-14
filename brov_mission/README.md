# brov_mission

`brov_mission` is a fail-closed boundary between an RViz waypoint draft and a
resolved controller input. It validates a position-only `pool` path and uses
the exact frozen alignment carried in `LocalizationStatus` to express an
immutable copy in `odom`.

It deliberately does **not**:

- open a MAVLink connection;
- publish PWM, wrench, or policy observations;
- call `/brov/start_control`, `/brov/stop_control`, or an arming service;
- continuously re-transform an active mission after localization corrections.

## Inputs

| Topic | Type | Contract |
|---|---|---|
| `/brov/mission/draft_path` | `nav_msgs/Path` | reliable/volatile; `header.frame_id=pool`, finite poses within the configured `min_waypoints`/`max_waypoints` bounds |
| `/brov/localization/status` | `brov_interfaces/LocalizationStatus` | fresh `INITIALIZED` epoch, session, boot-unique alignment ID, and exact `pool_T_odom` |
| `/brov/localization/odometry_pool_with_alignment` | `brov_interfaces/AlignedOdometry` | fresh `pool -> base_link` pose atomically bound to the exact epoch/session/alignment used by the first-point gate |

The standalone `/brov/localization/odometry_pool` remains available for RViz
and diagnostics, but mission validation does not subscribe to it. The atomic
envelope identity must exactly match the fresh `LocalizationStatus` before its
position can enter the first-waypoint distance gate.

Waypoint orientation is intentionally unsupported. Every draft pose must carry
the valid identity quaternion `[0,0,0,1]` (or the equivalent negative
quaternion). `orientation_support_enabled=true` fails node startup rather than
silently ignoring requested attitudes.

The default `brov_pool_position_mission_v1` contract supports deterministic
`straight`, `align`, and the demo-specific `takeoff_then_align`. The latter
requires exactly three points and `loop=true`: segment 0 is used once with the
initial level heading, then points 1 and 2 form the LOS-aligned loop. It excludes
legacy `upright`: yaw zero in a start-relative mission frame is not generally
pool yaw zero under a full-SE(3) localization alignment.

`brov_pool_position_mission_v2` is a separately versioned, random-attitude
contract. It requires `heading_mode=random_at_waypoint`, `loop=true`, and a
hashed `random_attitude` object in `canonical_plan_json`. That object binds the
integer seed, `pool_zup_flu` reference frame, deterministic
`sha256_counter_uniform_rpy_v1` generator, RPY bounds, maximum attitude slew,
attitude/angular-speed arrival tolerances, dwell time, maximum duration, and
maximum lap count. Waypoint pose orientations remain identity: v2 describes a
versioned attitude-generation and termination policy, not arbitrary attitude
fields supplied by a draft `Path`.

`min_waypoints` defaults to two for backward compatibility. Profiles that
define a fixed geometry set it equal to `max_waypoints`; a four-corner looping
mission therefore uses four unique vertices and does not repeat the first
vertex at the end. The closing edge is validated explicitly when `loop=true`.

### Deterministic generator wire algorithm

`sha256_counter_uniform_rpy_v1` is defined byte-for-byte as follows. For each
non-negative `event_index` and each `axis_index`, encode the ASCII string below
without a trailing newline:

```text
sha256_counter_uniform_rpy_v1:{seed}:{event_index}:{axis_index}
```

Each placeholder is its unsigned base-10 integer representation with no `+`
sign or zero padding.

Take the first eight bytes of its SHA-256 digest as one unsigned big-endian
64-bit integer and divide by `2^64` to obtain `u` in `[0,1)`. Axis indices
`0`, `1`, and `2` mean roll, pitch, and yaw respectively. Map each sample into
its hash-bound interval with `angle = min + u * (max - min)`. Convert the
resulting roll/pitch/yaw using the ZYX composition `Rz(yaw) Ry(pitch) Rx(roll)`
to a normalized `[w,x,y,z]` quaternion. Canonicalize the equivalent `q/-q`
pair by negating all components only when `w < 0`.

With `cr=cos(roll/2)`, `sr=sin(roll/2)`, and likewise `cp/sp` and `cy/sy`,
the pre-normalization components are exactly:

```text
w = cr*cp*cy + sr*sp*sy
x = sr*cp*cy - cr*sp*sy
y = cr*sp*cy + sr*cp*sy
z = cr*cp*sy - sr*sp*cy
```

The protocol-conformance vector below uses the legacy full-range bounds
roll/pitch `[-pi/2, pi/2]` and yaw `[-pi, pi]`; it is not the narrower Case C
operational envelope:

```text
seed=20260814, event_index=0
q_wxyz=[0.36995846, 0.15418720, 0.61874908, -0.67565274]
```

Producer and consumer tests reconstruct this vector independently with an
absolute tolerance of `1e-7`. Changing the payload encoding, byte order, axis
order, Euler convention, or quaternion sign rule requires a new
`generator_version`.

## Workflow

1. Publish/edit a draft path. Editing never changes a committed mission.
2. Call `/brov/mission/validate` (`std_srvs/Trigger`).
3. Inspect the returned hash and RViz draft.
4. Call `/brov/mission/commit` (`std_srvs/Trigger`).

Commit re-runs every validation and rejects an epoch/session/alignment change
between validation and commit. The transform comes from that same typed status
message, so a stale TF cache cannot be paired with a new generation. One
process permits one immutable commit. Idempotence requires both identical plan
content and identical localization identity; otherwise the mission manager
must be restarted before a new commit.

## Outputs

All outputs use reliable, transient-local QoS.

| Topic | Type | Meaning |
|---|---|---|
| `/brov/mission/active_path_pool` | `nav_msgs/Path` | immutable operator-approved pool path |
| `/brov/mission/resolved_path_odom` | `nav_msgs/Path` | same points under the commit-time `odom <- pool` TF |
| `/brov/mission/resolved` | `brov_interfaces/ResolvedMission` | typed immutable mission and localization identity |
| `/brov/mission/status` | `std_msgs/String` | compact JSON operator status |

The SHA-256 `plan_hash` covers the canonical pool points and guidance settings,
but excludes timestamps, mission UUID, and localization identity. Epoch,
odometry session, and the boot-unique `alignment_id` are explicit fields in
`ResolvedMission`.

The existing `ResolvedMission` ROS type carries both versions through its
`contract_version`, `canonical_plan_json`, `heading_mode`, and `loop` fields;
no parallel message type is used. Consumers must apply a version-specific
allowlist and reject v1+random, v2 without complete random metadata, or any
typed/canonical mismatch. The v2 counter-based sequence is reproducible only
when the consumer implements the named generator exactly.

## Configuration and execution

The supplied safe box applies to authored waypoint centres and the straight
segments between them. It is a conservative nominal cruising box, not a
certified hull/tether geofence. The measured current pose only has to be a
finite XYZ value: it may start just outside the box, such as on the pool floor
below the minimum cruising height. The first waypoint must still be inside the
box and within `max_first_point_distance_m` of that measured pose, so this
exception permits only a short, bounded entry into the waypoint box. Survey
the pool and configure swept-volume, tracking, and localization margins before
a water trial.

```bash
ros2 run brov_mission mission_manager_node --ros-args \
  --params-file $(ros2 pkg prefix brov_mission)/share/brov_mission/config/mission_manager.yaml
```

## Downstream control boundary

`ResolvedMission` remains in `odom`; it is not the legacy relative
`ned/start_heading` string. `brov_base/obs_node` is the sole actuation-side
consumer: when both localization/mission gates are enabled it verifies the
`localization_epoch`, `odometry_session_id`, `alignment_id`, dynamics bounds,
and hash form, then converts the immutable points to the configured legacy
mission frame. `/brov/prepare_control` performs that conversion while output is
still frozen so the target and controller action can be inspected. Arming and
start remain separate explicit steps. This package still never starts control,
and commit is not evidence that a runtime geofence or control loop is active.

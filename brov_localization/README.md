# brov_localization

`brov_localization` performs an operator-approved, one-shot alignment of the
continuous local odometry frame to the surveyed pool frame. It owns no MAVLink
connection, does not start control, and cannot publish PWM.

## Frame and math contract

`A_T_B` means a rigid transform that maps B-frame coordinates into frame A.
The inputs must already obey ROS conventions:

- `/brov/odometry/local_with_session`: `brov_interfaces/OdometrySession`. Its
  nested `odometry` is pose `odom_T_base`, `header.frame_id=odom`,
  `child_frame_id=base_link`; its twist is expressed in `base_link` as required
  by `nav_msgs/Odometry`. The same DDS sample carries the non-empty
  `odometry_session_id` that owns that pose.
- `/brov/aruco/robot_pose_pool`: `geometry_msgs/PoseStamped`, pose
  `pool_T_base`, `header.frame_id=pool`.
- `/brov/aruco/visible`: the current detector visibility flag.

`/brov/odometry/local` and `/brov/odometry/session_id` remain standard
diagnostic outputs from `brov_base`, but this package intentionally does not
correlate them. A changed identity in the atomic envelope invalidates the
alignment before the enclosed odometry is accepted, eliminating cross-topic
DDS delivery-order ambiguity.

For every fresh, time-paired, stationary observation the node computes

```text
pool_T_odom[i] = pool_T_base[vision, i] * inverse(odom_T_base[odom, i])
```

Translation is initialized by a component-wise median. Rotation is seeded by a
quaternion medoid, hemisphere-normalized, then averaged with the normalized
Markley eigenvector method. Translation/rotation residual gates are applied and
the inliers are fitted again. The accepted result is full SE(3); final absolute
roll and pitch have separate gates because both `pool` and `odom` are expected
to be gravity-aligned. It is not silently projected to yaw-only.

Once initialized, `pool_T_odom` is frozen. Marker loss does not invalidate the
alignment and marker reacquisition cannot move it. Reset or an odometry session
change stops pool odometry and TF publication until a new explicit
initialization. This is intentionally not continuous sensor fusion.

Every successful initialization creates a new UUID `alignment_id`, including
after a process restart. `epoch`, `odometry_session_id`, and `alignment_id` must
all match before a resolved mission can be reused. Reset, an empty/changing
odometry session, and any other alignment invalidation clear `alignment_id`.

## Timestamp and safety contract

The pairing logic compares acquisition timestamps and chooses the nearest local
odometry sample. It rejects zero, stale, future, non-finite, moving, excessive
skew, duplicate-image, frame-mismatched, and residual-outlier samples. A
decode-arrival timestamp on the H264 receiver does **not** recover camera
capture time; continuous/moving fusion must not be enabled until camera and
odometry source times share a verified clock. One-shot initialization should be
performed with the robot physically still and camera tilt fixed at its
calibrated neutral pose.

With the default `require_camera_tilt_neutral_confirmation=true`, the node is
fail-closed: it will neither collect alignment samples nor initialize until the
operator calls `/brov/localization/confirm_camera_tilt_neutral`. Confirmation
is accepted only after a non-empty odometry session exists and clears every
previous measurement/sample, ensuring the fitted set was acquired after the
physical check. Reset and session invalidation clear this confirmation.

## Outputs and services

- `/brov/localization/odometry_pool_with_alignment`
  (`brov_interfaces/AlignedOdometry`): canonical machine-consumption output.
  Each DDS sample atomically binds `pool_T_base` to its localization epoch,
  odometry session, and boot-unique `alignment_id`. It is published only while
  `INITIALIZED` and `output_valid=true`.
- `/brov/localization/odometry_pool` (`nav_msgs/Odometry`):
  standard RViz/diagnostic copy of `pool_T_base = pool_T_odom * odom_T_base`.
  Pose covariance is rotated into `pool`; twist and twist covariance remain
  unchanged because they are in the unchanged `base_link` child frame. The
  current PoseStamped vision input has no covariance, so this output does not
  claim to include marker survey, intrinsic, or extrinsic systematic
  uncertainty.
- dynamic TF `pool -> odom`, stamped with the corresponding odometry acquisition
  time. There is no TF before initialization. The old dynamic TF can remain in a
  consumer's finite TF cache briefly after reset; `/brov/localization/valid` is
  authoritative.
- `/brov/localization/status` (`brov_interfaces/LocalizationStatus`), transient
  local: state, epoch, session identity, `alignment_id`, the exact accepted
  `pool_to_odom`, atomic `output_valid`, accepted sample count, and reason. When
  invalid/uninitialized, `alignment_id` is empty and the payload transform is
  identity.
- `/brov/localization/valid` (`std_msgs/Bool`), transient local: true only while
  an alignment exists for the current session and local odometry is fresh.
- `/brov/localization/initialize_pool` (`brov_interfaces/InitializePool`): use
  buffered samples to install the one-shot alignment. `min_samples=0` selects
  the configured default. A positive request below `default_min_samples` is
  rejected; the request may only raise the configured safety floor. Calling it
  again requires an explicit reset first.
- `/brov/localization/confirm_camera_tilt_neutral` (`std_srvs/Trigger`): explicit
  physical-state acknowledgement for the current odometry session. It clears
  all samples so no pre-confirmation image can enter the estimate.
- `/brov/localization/reset` (`std_srvs/Trigger`): invalidate and clear all
  samples. The epoch advances on reset, session invalidation, and successful
  initialization so consumers cannot reuse a mission tied to an old mapping.

`/brov/aruco/robot_pose_pool` remains a raw perception/debug measurement. The
canonical TF chain has one owner per edge:

```text
pool --(this package)--> odom --(odometry source)--> base_link
```

## Build and run

```bash
cd /workspace/brov_ros2
colcon build --symlink-install \
  --packages-select brov_interfaces brov_localization
source install/setup.bash

ros2 run brov_localization pool_alignment_node --ros-args \
  --params-file /workspace/brov_ros2/brov_localization/config/localization.yaml
```

Keep the robot stopped, verify the status sample count, then initialize:

```bash
ros2 service call /brov/localization/confirm_camera_tilt_neutral \
  std_srvs/srv/Trigger "{}"
ros2 topic echo /brov/localization/status
ros2 service call /brov/localization/initialize_pool \
  brov_interfaces/srv/InitializePool "{min_samples: 20}"
```

Reset before changing the survey/extrinsic or intentionally realigning:

```bash
ros2 service call /brov/localization/reset std_srvs/srv/Trigger "{}"
```

Do not use the standalone `/brov/localization/odometry_pool` as a canonical
safety input. Consumers must use the atomic aligned-odometry envelope and also
match it against a fresh, `output_valid` localization status.

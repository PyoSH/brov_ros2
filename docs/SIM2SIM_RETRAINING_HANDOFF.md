# Sim2Sim and Retraining Handoff

## Status: superseded by the A/B/C/D retrain investigation

This document is a historical handoff written before the A(episode
redesign)/B(DVL sensor DR)/C(thruster DR)/D(raw-actor-overflow penalty)
retrain investigation. It analyzes the original legacy `demo_policy`
(SHA-256 `0d89f327...`, section 2 below), not any of the MK2-contract
`sim2swim_deploy_v2..v5_mk2_s42_i299` bundles.

Section 10's investigation plan and section 12's pre-pool-trial acceptance
criteria describe work that **has since been carried out**: retraining
across v2-v5, Case-A Gazebo GT/DVL-EKF validation, and an independent
Case-C (5 m square) cross-validation. None of the open hypotheses in
section 8 or the acceptance criteria in section 12 were re-verified
against this document's own checklist when that work happened, so treat
this page as background/methodology reference only, not as the current
status or an unstarted TODO list. For current status, results, and the
real-vehicle decision, see each policy bundle's own
`artifacts/policies/sim2swim_deploy_v*_mk2_s42_i299/README.md` (v5's in
particular records the case a-2 real-vehicle approval) and
[SIM2SWIM_DEMO.md](SIM2SWIM_DEMO.md). The Ubuntu/ROS setup steps in
section 9 remain generally applicable.

## 1. Purpose

This document records the current analysis of the gap between:

1. the Sim2Swim paper;
2. the local IsaacLab training and `test_policy.py` implementation;
3. Gazebo SITL sim2sim execution through `brov_ros2`; and
4. pool sim2real execution through `brov_ros2`.

It is also the handoff for a new development conversation on a separate Ubuntu
Linux computer. That computer is not the macOS/Docker host used for the pool
trials. It will run Ubuntu 22.04 with ROS 2 Humble, clone the repositories from
GitHub, build `brov_ros2` locally, and perform sim2sim and retraining work.

This document uses the English terms **interface specification**, **observation
specification**, and **runtime behavior**. The Korean word `계약`, previously
used as a direct translation of software `contract`, is intentionally avoided.

## 2. Repositories and artifacts

```text
ROS 2 runtime: https://github.com/PyoSH/brov_ros2.git
IsaacLab code: https://github.com/PyoSH/underwater_NBV.git
Training path: underwater_NBV/step_2_BROV
Policy path:   brov_ros2/artifacts/policies/demo_policy/policy.pt
```

The analysis baseline was:

```text
brov_ros2 revision:      ee4f3bf16c6c92842fe76f5bd3efc0bed6c5e147
underwater_NBV revision: 78d38b494cba6253c618d8b1a7bc07cb53c04560
policy SHA-256:          0d89f3270f46214f1569b7d48dcb5e25363b1d9b7353b82ced0fc67c0093a472
```

At analysis time, the exported training policy and ROS runtime policy had that
same SHA-256 digest. Record the digest and exact revisions again after cloning;
the remote branches may have advanced:

```bash
git -C ~/ws/underwater_NBV rev-parse HEAD
git -C ~/ws/brov_ros2 rev-parse HEAD
sha256sum ~/ws/brov_ros2/artifacts/policies/demo_policy/policy.pt
```

The current artifact metadata identifies a `(1,16)` observation and `(1,6)`
action, but does not fully identify the source training commit/checkpoint. A
new export must record training revision, checkpoint, observation version,
randomization profile, timestep behavior, and checksum.

## 3. What is already established

### 3.1 The policy is not only a stationary regulator

Local training starts each episode from zero velocity, but immediately provides
a nonzero, time-varying desired body velocity:

```text
v_d^b(t) = q_cmd * [0.5, 0.5 sin(0.2 t), 0.3 cos(0.2 t)]
```

Its desired speed is approximately `0.58--0.68 m/s`. IsaacLab
`test_policy.py` uses a default LOS cruise speed of `0.5 m/s`, so its reference
magnitude is close to the training distribution. The original pool trial at
`0.1 m/s` was substantially below that distribution and exposed thruster
deadband, small-command errors, estimator noise, and integral buildup more
strongly. Increasing speed changes only this distribution mismatch; it does not
remove plant, estimator, or runtime differences.

### 3.2 A sharp return attitude reference is intentional

The straight-line back-and-forth demonstration deliberately reverses the path
direction and changes desired heading at the endpoint. The investigation must
explain why IsaacLab tracks it while Gazebo and the real plant show poor coupled
translational behavior, rather than treating the reference change as a defect.

### 3.3 The second real trial did rotate successfully

In the second 1.5 m trial:

- desired attitude changed by approximately `109.8 deg` at the return;
- quaternion error increased from approximately `7.4 deg` to `114.4 deg`;
- it fell to approximately `5.9 deg` within about two seconds;
- no odometry-session fault occurred in that trial;
- the vehicle nevertheless moved substantially laterally and vertically and
  did not converge to the return waypoint.

The main observed failure is therefore poor simultaneous translational tracking
and DOF decoupling. It was already visible before the return transition.

### 3.4 The nominal software allocation is internally consistent

Replaying recorded six-dimensional actions through the nominal allocation and
inverse-thrust software produced desired-versus-allocated wrench correlations
close to one, with small nominal residuals. This lowers the probability of a
simple matrix-order bug, but does not validate:

```text
normalized PWM -> individual real thruster force -> vehicle 6-DOF wrench
```

Thruster direction, gain, forward/reverse asymmetry, deadband, response time,
mount error, saturation, and wake interaction remain unverified.

### 3.5 Sensor hardware is not missing

The real vehicle has the same classes of sensors described in the paper: Water
Linked A50 DVL, Bar30 depth sensor, and INS/IMU. Deployment consumes ArduPilot
estimates rather than raw sensor samples:

```text
DVL + Bar30 + INS/IMU
  -> ArduPilot estimator
  -> LOCAL_POSITION_NED and ATTITUDE_QUATERNION
  -> MAVLink
  -> ROS latest/latest snapshot
  -> observation builder
```

Vision performs a one-shot `pool -> odom` alignment and does not continuously
correct ArduPilot local position after initialization.

## 4. Paper versus local training implementation

The paper describes simultaneous tracking of time-varying velocity and
time-varying orientation references. Desired orientation follows the
Frenet--Serret frame of a trajectory. It also describes randomization of mass,
volume, and CB--CM offset.

| Item | Paper | Local training |
|---|---|---|
| Desired velocity | Time-varying | Time-varying sine/cosine template |
| Desired orientation | Time-varying trajectory frame | Random once, then fixed for each 5 s episode |
| Mass randomization | Present | Explicitly postponed/not implemented |
| Volume randomization | Present | Present, approximately nominal +/-10% |
| CB--CM offset | Randomized | Randomized |
| Added mass | Parametric uncertainty addressed | Rotational added mass only is randomized |
| Damping | Robustness required | No corresponding broad randomization |
| Thruster dynamics | Real transfer target | No gain/deadband/time-constant randomization |
| State sensing | Real estimator in experiments | Exact synchronous simulator state |
| Sensor imperfections | Real | No representative latency/noise/dropout model |

The local policy sees large initial attitude errors because the robot starts
near its default attitude while `q_d` is randomized. It does not systematically
see a new `q_d` step while velocity, angular rate, and integral states are
already nonzero. LOS evaluation therefore tests generalization outside the
exact temporal command distribution used for training.

## 5. IsaacLab test versus deployed runtime

Both paths use a 16-element observation:

```text
[q_error(4), body_velocity_error(3), body_angular_velocity(3),
 velocity_error_integral(3), attitude_error_integral(3)]
```

Their numerical and temporal behavior differs:

| Item | IsaacLab training/test | ROS deployment |
|---|---|---|
| State | Exact simulator ground truth | ArduPilot/MAVLink estimate |
| Synchronization | One simulation state | Latest attitude plus latest local-position message |
| Interval | Fixed `0.04 s` | Receive-time-based variable interval |
| Quaternion error | Raw quaternion product | `quat_unique`, positive-`w` representative |
| Integral duration | At most 5 s in training | Long mission duration |
| Integral bounds | No equivalent clamp | Runtime clamp at +/-5 |
| Delay/dropout | Absent | MAVLink/DDS latency and jitter |
| Test physics | Forced back to nominal | Gazebo or real hydrodynamics and actuators |

The positive-`w` quaternion representation is a reasonable runtime safeguard,
but it is not numerically identical to training. Do not assume the existing
policy is invariant to `(q_error, z_q) -> (-q_error, -z_q)` without replay.

The real mission also accumulated integral values for much longer than a
five-second training episode. A waypoint reversal under these states is not
represented by the training reset distribution.

## 6. IsaacLab plant versus Gazebo and real plants

The deployed path is longer than the nominal IsaacLab force/torque application:

```text
policy action
  -> per-DOF wrench scaling
  -> inverse thrust allocation
  -> eight normalized PWM commands
  -> RC override / ArduPilot output
  -> ESC and thrusters, or Gazebo actuator model
  -> hydrodynamic plant
```

Potential gaps include:

- mass, inertia, negative buoyancy, and CB/CM locations;
- translational/rotational added mass and linear/quadratic damping;
- tether, appendage drag, and cross-coupled forces/moments;
- PWM deadband, neutral offset, and forward/reverse asymmetry;
- per-thruster gain, installation direction, and response time;
- allocation loss under simultaneous saturation.

The shared poor behavior in Gazebo SITL and the real vehicle suggests
prioritizing components shared by those paths but absent from IsaacLab
`test_policy.py`: MAVLink state handling, deployment observation generation,
runtime integrators, wrench/PWM conversion, and non-Isaac plant models.

## 7. Navigation-state and timestamp limitations

`LOCAL_POSITION_NED` supplies position and velocity;
`ATTITUDE_QUATERNION` supplies attitude and angular rate. Runtime caches them
separately and constructs a latest/latest snapshot.

ROS Odometry is stamped at ROS processing time rather than MAVLink
`time_boot_ms`. An attitude-only update can cause publication with repeated
position. Finite-differencing current ROS Odometry position can therefore
produce zeros and short-interval spikes. The observed difference between
position finite-difference speed and reported velocity is a warning, not proof
that the DVL or ArduPilot estimator is wrong.

Use ArduPilot DataFlash and compare unique source-time samples:

```text
r_k = (p_(k+1) - p_k) / (t_(k+1) - t_k) - (v_k + v_(k+1)) / 2
```

Record DVL bottom-lock/quality, depth, estimator innovations, lane/reset events,
and source timestamps. A future diagnostic message should carry attitude and
position `time_boot_ms` atomically instead of replacing them with receive time.

## 8. Confirmed differences, observations, and open hypotheses

### Confirmed implementation differences

1. Paper training uses time-varying desired orientation; local training holds
   `q_d` fixed within each episode.
2. Paper training randomizes mass; local training does not.
3. Training and runtime use different quaternion-error representations.
4. Training uses fixed timestep/short integrals; runtime uses variable receive
   timing, long integrals, and clamps.
5. IsaacLab uses exact synchronous state; deployment uses asynchronous
   estimator messages.
6. IsaacLab straight-line evaluation uses nominal physics; Gazebo and real
   execution use different plant and actuator paths.

### Confirmed experimental observations

1. Tracking was poor before the return attitude transition.
2. The robot completed the large attitude change, demonstrating rotational
   authority.
3. Translational drift became large during coupled rotation and translation.
4. Several action/PWM channels approached saturation.
5. Integral observations accumulated far longer than a training episode.
6. One trial contained a large received attitude discontinuity; a later trial
   did not reproduce it, so it is not the general explanation.

### Open hypotheses

1. Physical or Gazebo thruster gain/deadband/direction differs from IsaacLab.
2. Real and Gazebo mass, inertia, damping, or added mass lie outside training
   randomization.
3. MAVLink latency and latest/latest state construction cause meaningful delay
   or inconsistency.
4. Long-duration integral states take the policy outside training distribution.
5. A remaining NED/FRD to Z-up/FLU semantic difference exists despite unit
   tests; this requires end-to-end replay, not more isolated sign inspection.

## 9. Ubuntu 22.04 / ROS 2 Humble setup

The new computer must not assume previous macOS paths. Use fresh Git clones and
record revisions.

### 9.1 ROS runtime workspace

```bash
mkdir -p ~/ws
cd ~/ws
git clone https://github.com/PyoSH/brov_ros2.git
cd brov_ros2
git rev-parse HEAD
```

Install ROS 2 Humble Desktop on Ubuntu 22.04 using the official ROS procedure.
The repository Dockerfile remains the dependency reference for native use.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-pip python3-rosdep \
  python3-gi python3-opencv python3-pytest python3-yaml \
  gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0 \
  gstreamer1.0-libav gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-tools \
  ros-humble-camera-calibration ros-humble-cv-bridge \
  ros-humble-desktop ros-humble-image-proc ros-humble-image-transport \
  ros-humble-rosidl-default-generators ros-humble-tf2-ros

python3 -m pip install --user -r docker/requirements.txt
python3 -m pip install --user \
  --index-url https://download.pytorch.org/whl/cpu \
  'torch>=2.1,<3'
```

Initialize `rosdep` if needed:

```bash
sudo rosdep init
rosdep update
```

Build and test locally:

```bash
cd ~/ws/brov_ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y \
  --skip-keys "python3-torch python3-pymavlink-pip"
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The skipped ROS dependency keys are installed explicitly above: PyTorch from
the PyTorch wheel index and `pymavlink` from `docker/requirements.txt`. Verify
both imports instead of silently omitting either dependency:

```bash
python3 -c 'import torch, pymavlink; print(torch.__version__)'
```

### 9.2 IsaacLab training repository

```bash
cd ~/ws
git clone https://github.com/PyoSH/underwater_NBV.git
cd underwater_NBV
git rev-parse HEAD
```

IsaacLab, Isaac Sim, NVIDIA driver, CUDA, and RSL-RL versions must match the
training environment. Do not infer them from ROS Humble. Record:

```text
Ubuntu version
NVIDIA driver
CUDA runtime
Isaac Sim version
IsaacLab revision
RSL-RL version
PyTorch version
GPU model
```

Training work is under:

```bash
cd ~/ws/underwater_NBV/step_2_BROV
```

Before retraining, reproduce the existing checkpoint with `test_policy.py` and
verify its digest against the runtime artifact.

## 10. Required sim2sim investigation order

### Stage A: exact IsaacLab baseline

Run the existing checkpoint with the deployed Case-A behavior:

- level takeoff to the equivalent of pool `z=0.7 m`;
- 1.5 m horizontal line, `loop=true`, `takeoff_then_align`;
- lookahead `0.4 m`, reach threshold `0.15 m`;
- cruise speeds `0.1`, `0.2`, `0.3`, and `0.5 m/s`;
- duration at least 60 s with no hidden episode reset.

Extend `test_policy.py` to expose these exact parameters rather than claiming
equivalence from a visually similar test. Record at every policy timestep:

```text
time, position, actual/desired quaternion, actual/desired body velocity,
body angular velocity, quaternion/velocity error, both integrals, action,
desired wrench, and waypoint index
```

### Stage B: observation equivalence replay

Feed one IsaacLab ground-truth sequence into both the training observation
calculation and ROS `ObservationBuilder`, after explicit frame conversion.
Compare all 16 fields and all six actions for:

- straight motion;
- yaw `+/-90` and `180 deg`;
- endpoint velocity reversal;
- `q` versus `-q` representations;
- fixed `0.04 s` versus recorded variable intervals;
- integral reset and clamp behavior.

The acceptance criterion is equal policy input and action for the same physical
state/reference history, not isolated unit-vector agreement.

### Stage C: Gazebo SITL stage-by-stage trace

Run the same path/speed cases and record:

```text
Gazebo ground-truth pose/twist
ArduPilot estimated pose/twist and source timestamps
desired pose/velocity
exact 16-D observation
policy action
scaled desired wrench
allocated normalized PWM
simulated thruster force, if available
```

Compare truth versus estimate, target versus actual, action versus desired
wrench, desired versus allocated wrench, and allocated wrench versus Gazebo
acceleration. Locate the first stage with a large divergence before tuning.

## 11. Retraining specification

Preserve the 16-D low-level observation unless a separately versioned change is
approved. Add robustness within that structure:

1. Generate time-varying desired orientation during training.
2. Add mid-episode `90--180 deg` attitude changes and velocity reversals while
   velocity, angular rate, and integrals are nonzero.
3. Train across `0.05--0.7 m/s`.
4. Match training/runtime quaternion canonicalization exactly.
5. Match timestep, integral update, clamp, and reset behavior exactly.
6. Extend effective history beyond five seconds or randomize reachable initial
   integral states.
7. Randomize mass, inertia, buoyancy, CB/CM, translational/rotational added
   mass, and linear/quadratic damping.
8. Randomize thruster gain, asymmetry, deadband, time constant, orientation
   error, saturation, and command delay.
9. Add DVL/INS sample-and-hold, latency, skew, noise, outliers, and dropout.
10. Validate on nominal/randomized IsaacLab and Gazebo SITL before export.

Every release must record:

```text
source revision, checkpoint, training configuration digest,
observation specification version, runtime behavior assumptions,
action/wrench scaling, TorchScript SHA-256, and evaluation summaries
```

## 12. Acceptance criteria before another pool trial

1. Exact Case A succeeds in nominal and randomized IsaacLab at `0.1--0.5 m/s`
   without hidden resets.
2. Training/deployment observation replay agrees field-by-field within declared
   tolerances, or every intended difference is versioned and tested.
3. Gazebo truth and ArduPilot estimate are separately plotted.
4. Target/actual body velocities and quaternions are logged at policy rate.
5. No unexplained action/PWM saturation persists before reversal.
6. A full return completes within path tolerance and time budget.
7. Policy provenance and checksum are recorded.
8. ROS remains fail-closed for stale state, estimator-session changes, missing
   controller output, and duplicate command publishers.

The next conversation should locate the first divergent stage rather than tune
multiple layers simultaneously.

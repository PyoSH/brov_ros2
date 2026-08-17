# brov_interfaces

Typed messages used at the safety-critical boundaries between local odometry,
one-shot pool localization, and committed pool-frame missions.

- `LocalizationStatus`: localization state, boot/session/alignment identity,
  epoch, and the exact accepted `pool_to_odom` transform.
- `AlignedOdometry`: pool-frame odometry atomically bound to the localization
  epoch, odometry session, and boot-unique alignment identity that produced it.
- `OdometrySession`: one DDS sample containing local odometry and the exact
  odometry-session identity that owns it. Localization consumes this atomic
  envelope instead of correlating independently delivered topics.
- `ResolvedMission`: an immutable pool mission resolved once into the current
  `odom` frame and bound to that exact alignment identity.
- `InitializePool`: explicit operator-approved full-SE(3) one-shot alignment.

Changing any field is an interface-version change and requires rebuilding all
dependent packages.

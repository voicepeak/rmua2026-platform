# Foundation acceptance — initial evidence

Decision: **No-Go** for PWM controller training and claims of official equivalence.

The local 12.0.0.5 Linux executable SHA-256 matches the expected
5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800.
An extracted settings file is preserved and hashed; it has not been established
that a particular official run actually loaded it. Launch arguments, map, seed,
PWM runtime behavior, body/attitude conventions and motor directions remain
insufficiently verified. A separate historical seed-123 settings snapshot
matches that run's preflight hash; this is not proof of the present runtime setup.

| Area | Status | Evidence / missing work |
| --- | --- | --- |
| Version | U | Executable hash checked; complete runtime configuration not captured. |
| Coordinate | U | Historical GDB/ROS pairing had motion residuals up to 56 cm; a new unforced short vertical trial is closer before player exit but has post-exit outliers. Origin, reference point and clock alignment remain unverified. |
| Map/collision | U | Inventory covers 12 static exports; OL1 has 12 gates, 13 factories and 13 ports, with two missing locations. True collision geometry and contact tests incomplete. |
| Motor and 6DOF | U | Published parameters and independent candidate exist; D00–D06 and held-out calibration absent. |
| Wind | U | Fresh injected keyframes reproduced byte-for-byte in 18 recorded cases; natural zones and force coupling not validated. |
| Force field | U | Boundary, action chain and timing not verified. |
| Vehicles and lines | U | New configurable swept collision proxies and basic reproducibility tests exist; placement feasibility and official size/velocity coverage remain unverified. |

One new isolated passive ground-start capture is recorded in
`foundation/datasets/d00_passive_20261107/`. It contains 499 IMU and pose
samples over 4.98 simulated seconds with no backwards or duplicate header
timestamps, and the disposable container was stopped. The pose was constant;
the mean body acceleration was approximately (−0.0038, 0.00005, −9.8069)
m/s². This is a collector and ground-state check, not an airborne D00 or a
measurement of action/actuator latency.

Exploratory one-second replay windows of a prior closed-loop seed-123 run have
maximum position discrepancies 0.057–0.063 m and maximum attitude discrepancies
0.076–0.495 degrees. These use posthoc position-derived initial velocity, an
assumed initial motor state, and unverified command timing; the same recording
is neither independent excitation nor a held-out calibration set. See
`seed123_*replay*.json` for hashes, assumptions and per-sample errors.

Gate A and B remain open. Next: safe self-terminating PWM acquisition, motor
directions and D00–D04. Do not infer physical equivalence from static parameter
values, these replays, or coordinate round trips.

## Initial PWM experiments (2026-11-07)

Fresh process, seed 123, NullRHI, verified binary and settings hashes. Each run
uses `debug/pose_gt` only for a research guard and offline evaluation, publishes
bounded commands and terminates its own identity-checked container.

* `d01_collective_20261107`: 0.12/0.20/0.16/0 common PWM, 97 commands;
  0.20 stage median debug PWM was approximately 0.20 and vertical position
  changed from ground-start. 0.12 stage stayed on the ground, so those IMU
  readings do not measure free-flight thrust. A 0.25 s airborne exploratory
  replay has 0.0047 m max position error, using fitted initial velocity,
  assumed motor state and no verified command delay. It is not an acceptance.
* `d03_differential_20261107`: first waveform exceeded the 0.8 m motion
  guard; aborted, sent zero commands and stopped the owned container. Retained
  as a failed experiment, not used as an acceptance case.
* `d03_differential_retry_20261107`: reduced common thrust, four sequential
  +0.004 single-motor pulses; completed. IMU angular-rate fitted slopes had
  signs (-,+,+), (+,-,+), (+,+,-), (-,-,-), matching the provisional FRD
  candidate. Roll/pitch magnitudes were close to its instantaneous estimate;
  yaw magnitude differed, particularly on motor 3. See `phase_analysis.json`
  for values, sample counts and source hashes. These are short sequential
  pulses with unknown delay and prior motion; no claim of identified lag,
  full operating range, fully validated rotor physics or held-out accuracy.

Subsequent work must improve post-command guard coverage, independently repeat
and vary the waveform, synchronize action edges/physical response, and collect
D02/D04–D06. Wind, force, collision-map and gate acceptance remain open.

An independent fresh seed-456 differential repeat again produced the four
expected sign patterns; the owned simulator was stopped immediately on player
exit and the interrupted collector is explicitly reported. This still uses the
same waveform and is not a distinct held-out excitation.

The seed-2026 diagonal yaw experiment adds a different short excitation. Its
positive/negative yaw slopes and candidate predictions are recorded in
`d04_yaw_seed2026_20261107/phase_analysis.json`; the replay and limitations are
documented in `dynamics_candidate.md`. It does not close D04 or Gate B by itself.

Two additional fresh-process opposing roll/pitch trials at different amplitudes
show matching acceleration signs and close instantaneous predicted slopes, but
0.45 s trajectory replays reach 0.693/1.064 degrees maximum attitude error.
Initial state, physical latency and full-envelope held-out checks remain open.

The first road collision extraction now includes actual cooked local PhysX
triangles for `Segment` and `Segment_Half`, plus 483 uncomposed OL1 actor
attachment chains. See `road_collision_extraction.md`. They are not yet a
world-aligned or contact-verified collision map, so map acceptance remains U.

A candidate composed road placement and finite-segment query now exist. The
static spline check exposed a low vertical-ray hit rate (1/61), so the candidate
is explicitly unverified and incomplete. Two local factory meshes are also
extracted; their world transforms and effective collision require validation.

`foundation.sim.PhysicsSim` now joins the uncalibrated quad, explicit external
force replay, and swept line/vehicle proxies with deterministic substeps and
terminal handling. It explicitly reports that static map collision, official
wind force and degradation are missing; it does not constitute Gate A or B.

An optional candidate-map sphere sweep now detects triangle surface crossings
when supplied with an explicit frame origin; it does not establish official
static collision and does not test containment or calculate exact contact
time. See `road_collision_extraction.md` for coverage and limitations. Wind
force, degradation and both acceptance gates remain open.

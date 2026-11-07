# Official simulator policy diagnostic: seed 283441

Random seed 283441 was drawn once. Three fresh OL1 single-instance launches
used the same seed; they are diagnostic retries, not three independent seeds.
Checkpoint: `runs/stage1_audited_seed7_20261106/best.pt`, SHA-256
`be989bcfb432b7d067aad919ad04b99d44867fae6e5556975ae2f008ae7617c1`.
Only actor tensors and observation scaling were exported to NumPy for the ROS
container. A unit test checks NumPy inference against PyTorch.

The bridge used in the original three attempts used debug pose
truth, a filtered finite-difference velocity, zero angular velocity, inferred
local gate/force state, 20 Hz command publication, and a yaw controller capped
at 1 rad/s. It is **not** a deployable actor, official-equivalence test, or a
faithful reproduction of the training observations and plant. It has no
official-state subscription; those experiments were manually stopped after
`Finished`. No derived simulation output after `Finished` is a valid result.
The first two launches had the wrong vertical command sign, so their 0 scores
are adapter errors, not model failures.

In the third launch the sign was corrected and a rule-based vertical takeoff
preamble was used before handing control to the actor. The preamble reached
about 1.68 m altitude at 2.4 s. During the remaining start phase, the drone
stayed close to the origin: maximum x about 1.03 m, maximum altitude about
4.56 m (separately sampled maxima). Internal Score remained 0 and the game
state changed from `Pending1` to `Finished` approximately 29 s after score
monitor startup. The clocks of the monitor and policy log were started
separately; timestamps are not precisely synchronized. The event is consistent
with a failure to depart, but the exact termination cause has not been traced.

An independent same-seed control flight using the existing rule-based
`route_follower`, with `debug/pose_gt` and its own static vision/guides, took
off and reached x≈102.8 m; its own gate events are diagnostics, not official
scoring. This distinguishes the runtime from a general failure to launch but
does not isolate the cause of the model/bridge behavior. No stage completion
or official competition performance is claimed.

Raw local records are in `data/audits/policy_seed_283441/`; remote copies are
under `/home/tianbot/RUMA-by-helinjun/beta/official_policy_smoke_283441/`.
The sim and bridge were stopped after the experiment; the container remained
running. The bridge's velocity-sign error has been corrected: official
`vel_body_cmd.vz` uses up-positive, while pose `z` is NED down-positive.

Next, evaluate and independently validate an observation/command adapter:
startup behavior and ramping, velocity estimation and command limits, full
attitude vs yaw-only training assumptions, update rate, and inferred gate
state. Log comparable controller/pose traces; do not claim the policy itself
is solely responsible for this result. A final actor must use sensors/EKF,
not `debug/pose_gt`.

## Controlled departure and staged handover

The following fresh single-instance runs used the same seed 283441. They are
controlled diagnostics, not independent-seed acceptance. The bridge now
requires a fresh read-only official-state probe file, commands zero while the
probe is absent/stale, and stops on Finished or stage 2. The probe checks the
exact binary hash before reading memory. This privileged internal state is
instrumentation, not a deployable interface. A read-only GDB breakpoint at
`0x2fb7110` records one type of collision callback; its absence does not rule
out every contact. GDB all-stop can affect timing.

| Run | Control while actor runs | Last controlled sample | Internal state / score |
|---|---|---|---|
| `departure_shadow` | Rule-based takeoff, altitude and ≤2 m/s forward; actor shadow only | x=12.01 m, z-up=1.80 m, 10.55 s | State1 / 1 |
| `horizontal_guarded` | Actor horizontal after x≥12 m, capped at 4 m/s and 1 m/s²; rule altitude | x=30.15 m, z-up=1.81 m, 16.10 s | State1 / 2 |
| `vertical_guarded` | As above; actor vertical after x≥20 m, clipped ±0.4 m/s and to a fixed 1.4–3.0 m band | x=30.04 m, z-up=2.78 m, 15.90 s | State1 / 2 |
| `road_guarded` | As above; vertical clipped ±0.7 m/s in an envelope around the approximate local path | x=60.12 m, z-up=5.00 m, 24.05 s | State1 / 3 |
| `extended_guarded` | Approximate road envelope; horizontal cap 8 m/s and 1 m/s² | path progress≈200 m, x=198.42 m, z-up=11.71 m, 38.95 s | State1 / 10 |

Each run stopped itself at a prescribed short-range checkpoint while State1;
none establishes full-stage success. The simulator was stopped after each
run. In the horizontal run it continued after the bridge stopped and later
entered Finished; that later transition is not evidence of failure during the
controlled interval. The collision breakpoint recorded a callback in state 0
near startup in these runs; the source and meaning were not decoded, so do not
claim a collision-free run based on these traces.

In the shadow run actor forward outputs ranged approximately 12.2–22.7 m/s and
vertical outputs 1.65–4.63 m/s while the controller held about 2 m/s and 1.8 m
altitude. Official State1 and the locally inferred start-phase observation
disagreed for 6–8 sampled frames across these runs near x≈2.6–3.1 m; score and
inferred gate count also have brief polling/sampling differences. These
observations motivate calibration, but do not isolate a unique failure cause.

Raw synchronized-monotonic traces, collision logs and the frozen earlier
guarded bridge are in `data/audits/policy_seed_283441/`. The horizontal and
vertical guarded bridge SHA-256 was
`f489d854f4f7aea667e6a1ac4d745fc5d2236cd7ab68aa3f8686a85cd1a0e221`;
the road run used
`dddb183b4535e9c0236e92f0968f3b85c67bb57b84e760bf18629d71ae444e70`.
The extended run used
`31dcd1d501b8a3939f0d8e46fb7ad43a2769f67832e4a505ad51e86221942fe6`.
The state probe SHA-256 was
`357ad8bbf37c5ce1d9194ab2818796a47dfa3945ad70121ddb109a7b3ec5f4e1`.
All actor runs used debug pose and estimated/inferred state. Gate score and
phase polling are not a substitute for an official competition result.

In the extended run, after the bridge stopped at its checkpoint, the simulator
continued running and approximately 15 s later a collision callback and
`State1`→`Finished` request occurred. This happened after the controlled
interval, but shows that stopping the bridge alone does not end the simulator
or guarantee safe stopping. That bridge published a single zero-velocity
message without a timestamp on exit.

A subsequent 60 m `road_guarded` retry used bridge SHA-256
`8cb143836dea01b0f94ec5fb142a73a8446e7a641d507763d3c3efd9158fa466` and
published 20 timestamped zero-velocity messages after its checkpoint. During
the 0.95 s sampled braking interval the drone moved about 2.98 m; this does
not establish a full stop. The wrapper's normal TERM request did not end the
simulator while GDB was attached, so GDB was terminated and the simulator was
explicitly killed. Future experiment orchestration must verify process exit
and stop the diagnostic simulator immediately; do not assume zero command or
TERM succeeded. Raw `brake_*` and `extended_*` records are retained.

Next: validate the monitor/collision interpretation and state estimator,
calibrate the local road and vertical dynamics against official traces, then
extend cautiously with fresh seeds. Full stage, hub, stage transition, and a
deployable sensor-only actor remain unverified; retraining should be decided
after the simulator-to-local discrepancies are measured.

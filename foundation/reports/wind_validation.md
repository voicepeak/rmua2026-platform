# Wind generator: constrained validation

## Integrated recorded-effects replay (2026-11-08)

`RecordedEffects` accepts explicitly time-aligned NED wind samples and
externally supplied field flags; wind is linearly interpolated and flags held
until the next timestamp. `PhysicsSim` can now consume this schedule, requiring
an aerodynamic model when wind samples are provided, split steps at schedule boundaries, apply the
observed candidate 0.25/1.0 rotor-target scaling, and optionally accumulate
the separately supplied `OutTimeCounter`. A timeout request is reported but
does not automatically end the simulation. Out-of-range schedules fail before
state advances. This interface does not infer natural wind zones, natural
field membership, game-state reset behavior, or official failure semantics.

The integrated engine replayed the same forced-stage wind recordings as the
previous direct `Quad` replay, with constant field flag 1. On seed 123/456,
the recorded-wind horizontal position RMSE was 0.000300/0.000094 m over the
short chosen windows; the zero-wind comparison was 0.001747/0.000913 m.
The seed-456 integrated and direct recorded-wind RMSE values differ by about
2.4e-8 m; their discretizations differ because the integrated engine splits at
wind sample timestamps. Full errors and hashes are in
`wind_stage4_integrated_seed123.json` and `wind_stage4_integrated_seed456.json`.
This is a pipeline consistency check and same-recording exploratory replay,
not an independent held-out W01 or F01 acceptance. The original GDB, forced
stage, initial-state and arrival-time limitations remain.

`foundation.wind_generator` wraps the unchanged vendored reference generator,
requires the hash-verified 12.0.0.5 ELF for its constants, and supports fresh
seed generation or recorded keyframe bytes. Out-of-range sampling is rejected.
`validate_wind.py` independently compares generated keyframe bytes against the
historical GDB logs for stages 4–6, seeds 123/456/2026 and two repeats: all
18 arrays match exactly. Source-log hashes and counts are in
`wind_generator_validation.json`.

Those data use injected stage initialization with no prior libc random calls.
They do not establish natural-stage random stream state, spatial boundaries,
wind-to-force coupling, or dynamic trajectory accuracy. The sampler uses host
trigonometric functions and is not asserted byte-identical to glibc `sincosf`.
`ForceReplay` is an explicit time-and-region-gated replay of supplied force in
newtons, with no extrapolation. It is not derived from wind velocity and no
official force data have been claimed or invented. W01 and force-field F01
remain unfulfilled; neither dynamic model is calibrated for PWM training.

## Explicit aerodynamic candidate

`foundation.aero.DragFaces` implements an independently parameterized candidate
based on the public AirSim `FastPhysicsEngine` at commit
`1ca93f6f77e4e8a39b2b241c1fe2764da4d7dd41`:
https://github.com/microsoft/AirSim/blob/1ca93f6f77e4e8a39b2b241c1fe2764da4d7dd41/AirLib/include/physics/FastPhysicsEngine.hpp
It transforms vehicle velocity relative to world wind into body coordinates,
adds angular velocity at each supplied drag face, applies a quadratic normal
force only above a supplied positive-face velocity threshold, sums body torque,
and rotates the resulting force into world coordinates. The model requires the
caller to supply face locations, unit normals, drag factors, air density and
cutoff; it does not assign them official values.

`PhysicsSim` can explicitly accept this model and a world wind vector per step;
it rejects a wind vector when no aerodynamic model is supplied. Unit tests
exercise relative velocity, rotation, angular velocity, force, torque and
simulator integration. The shipped simulator's corresponding implementation,
face parameters, density, wind application and time synchronization have not
been verified. This is a research candidate and W01 remains open.

## Read-only drag-face snapshots

The SHA-verified 12.0.0.5 executable's `MultiRotorPhysicsBody` virtual
functions at `0x26987f0` and `0x2698810` expose a candidate vector at body
offset `0x170`, with 128-byte entries. Two isolated, read-only GDB runs captured
six entries with identical decoded initial positions, normals and candidate
drag factors. The seed-123 snapshot was captured before the current vertex
fields were initialized; the seed-456 snapshot waited until all six current
position/normal fields matched their initial fields. Full raw bytes, source
hashes and decoded values are in `drag_faces_seed123.json`,
`drag_faces_seed456.json` and their dataset logs.

The paired faces are at ±0.11050 m on X, ±0.04815 m on Y, and ±0.04515 m on
Z. The candidate factors at offset `0x78` are respectively 0.00608117,
0.00791099 and 0.11017096 for each axis pair. The position/normal layout is
supported by the shipped rotor reset disassembly and the public upstream
`PhysicsBodyVertex` layout; the loop observation below independently supports
the drag-factor interpretation. The explicit
`DragFaces.from_observed_snapshot` loader accepts only the initialized
six-face snapshot for this binary hash, and still requires callers to provide
density and velocity cutoff. This narrows the candidate parameter search but
does not establish wind coupling, natural zones, aerodynamic model parity or
W01 acceptance.

## Native drag-loop operand check

The shipped physics loop calls the observed drag-vertex accessors and uses
the face's current position, normal and `0x78` factor. Its comparison
constant at ELF virtual address `0x6c0e44` is float32 approximately 0.1;
the saved `drag_loop_disassembly.txt` shows the velocity-component threshold
and quadratic scalar operations. A fresh seed-123 read-only breakpoint at
`0x2669755` captured 45 native scalar calculations. The scalar reproduced as
float32 `-factor × density_operand × component²` with maximum absolute
difference below 4.5e-9 in the logged values; the density operand ranged
approximately 1.21070–1.21078 during that capture. Raw values, binary/log
hashes and per-sample errors are in `aero_force_seed123.json`.

This validates the observed scalar calculation and sampled face factor in
one debugger-instrumented process, not the complete wind-to-motion model:
the velocity and wind inputs were not independently paired, the density
operand's source was not traced, startup/contact may influence the samples,
and natural wind zones remain unknown. W01 remains open.

## Injected wind and input-to-face check

Two additional seed-123 processes used the same bounded ground-start PWM
waveform. A debugger breakpoint at `0x2669530` overwrote the physics function's
wind-vector argument with either (3, 0, 0) or (0, 0, 0) m/s; a second breakpoint
captured function-entry velocity, wind, angular velocity and quaternion plus
the individual face operands and scalar in that same thread. The first wind
attempt failed before readiness because an initial probe read a register after
it had been reused; its failed log is retained. The corrected nonzero and zero
runs completed and stopped their dedicated containers.

For 47 injected-wind samples and 44 zero-wind samples, the reconstructed
body-frame normal velocity matched the native value with maximum absolute
errors of 1.20e-7 and 0 m/s, respectively; the resulting face scalar errors
were below 8.4e-8. For example, one nonzero-wind sample with zero captured
vehicle velocity and angular rate gave normal component 3 m/s on the negative
X face and scalar -0.06626, producing a positive-X face force. Per-sample
operands and hashes are in `aero_paired_wind3_seed123.json` and
`aero_paired_wind0_seed123.json`; `drag_input_disassembly.txt` records the
observed velocity-minus-wind input path.

This supports the candidate relative-velocity and quadratic face calculation
for the sampled states. All logged angular rates and quaternion vector parts
were zero, so this sample does not validate rotation or angular drag. It does
not prove the natural game
wind reaches that argument, because the breakpoint writes the argument directly;
nor does it validate a full simultaneous net wrench, natural zone behavior,
airborne trajectory parity, or W01. Ground contact, startup and debugger
pauses limit the two trajectory comparisons.

## Forced stage-4 wind delivery

Two further fresh seed-123/456 processes forced stage-4 initialization and
state 7, initialized its wind clock, and suppressed overtime; unlike the
physics-entry injection above, they did not overwrite the physics wind vector.
In each run, 30 nearest-time pairs between the decoded ROS debug wind and the
read-only physics-entry vector were nonzero. Their maximum vector differences
were 9.1e-8 and 8.7e-8 m/s respectively, using a 60 ms maximum arrival-time
gap. Individual pairs and hashes are in `wind_delivery_stage4_seed123.json`
and `wind_delivery_stage4_seed456.json`.

The observed vectors were effectively constant in these short captures, so
these results support matching values at the initial area in the controlled
stage but do not measure delivery latency or dynamic transitions. Stage and
state injection, all-stop GDB, ground start, and nearest subscriber-arrival
pairing limit extrapolation; natural course progression, wind-zone boundaries,
and W01 trajectory accuracy remain unverified.

An additional seed-456 run used injected (3, 0, 0) m/s wind and a small
roll/pitch PWM waveform. Its 46 sampled active-face calculations include
angular rates up to 0.409 rad/s and quaternion vector-part norm up to 0.0367.
Using the recorded quaternion and angular rate, reconstructed normal velocity
matched the native loop with maximum absolute error 1.22e-7 m/s; the scalar
force matched within 6.4e-8 in the logged units. Four distinct face-normal
directions appeared in the sampled data. See
`aero_paired_wind3_tilt_seed456.json` for every sample and source hash. This
supports the rotation and angular-velocity terms over that short injected
excitation; it does not establish natural-zone activation, independent net
wrench, or end-to-end trajectory accuracy.

## Six-face instantaneous wrench comparison

A separate seed-2026 injected-wind tilt trial captured the native sum at the
end of the six-face loop, before force rotation to world coordinates. For 23
samples, the candidate uses the six-face snapshot from seed 456 and each
update's observed density, velocity, wind, orientation and angular velocity.
The largest body-frame force-vector discrepancy was 3.01e-8 in the logged
force units. The maximum angular-rate norm was 0.414 rad/s and the largest
quaternion vector-part norm was 0.0346. Full per-sample data and hashes are in
`aero_net_tilt_seed2026.json`.

Both observed and candidate torque were zero throughout: these six observed
face positions are parallel to their normals, so this does not validate a
nonzero aerodynamic torque. Wind was injected at the physics entry, and GDB
affects scheduling; this comparison does not verify natural wind delivery,
full trajectory dynamics, collision effects or W01 acceptance.

## Controlled changing-wind delivery

A further seed-456 trial forced the same stage-4 state but initialized its
wind clock five seconds in the past to sample a changing portion. Of 30
paired ROS/physics-entry observations, all were nonzero and 16 native
vectors were distinct; the first-to-last vector difference was 0.538 m/s.
The nearest-arrival gap was at most 9.96 ms, and the maximum vector mismatch
was 9.45e-8 m/s. Per-pair data and hashes are in
`wind_delivery_stage4_late_seed456.json`.

This supports delivery of the observed changing values in that forced run.
The clock offset, forced stage/state and debugger make it unsuitable for
measuring natural onset or latency; nearest-arrival pairing, one seed, and
limited initial-area coverage also prevent W01 acceptance.

## Exploratory forced-stage trajectory comparison

Three approximately 0.54 s windows from the forced stage-4 captures were
replayed from their own posthoc estimated initial states, using the observed
six-face candidate and either the recorded debug wind or zero wind. Horizontal
position RMSE with recorded wind was 0.300, 0.094 and 0.320 mm; the same
models with zero wind gave 1.747, 0.913 and 1.272 mm. Maximum total position
errors with recorded wind were 1.75, 1.62 and 1.47 cm. The third run used the
five-second wind-clock offset. Full samples, assumptions and hashes are in
`wind_stage4_replay_seed123.json`, `wind_stage4_replay_seed456.json` and
`wind_stage4_late_replay_seed456.json`.

These are short same-recording diagnostics, not independent holdout tests.
Initial velocity comes from the same posthoc pose, initial motor state and
constant density are assumed, and wind and pose use subscriber arrival times.
Force-field state, contact, actuation latency and exact physics time remain
unverified. The millimetre-scale horizontal differences are sensitive to
those assumptions; neither these windows nor forced-stage data establish
natural boundary behavior or W01 acceptance.

## Historical trajectory repeatability

`analyze_wind_repeatability.py` compares the stage-6 injected experiments on a
50 ms grid relative to the first command header. Within-seed pose separation
reached 0.558 m (seed 123), 1.252 m (456) and 0.438 m (2026); the corresponding
maximum wind-vector separation was 1.137, 8.112 and 0.039 m/s. The sampled
published commands matched on that grid, but this does not establish that
applied motor inputs or all command edge times matched. Full statistics and
input hashes are in `wind_repeatability_stage6.json`.

These are injected-stage, ground-start recordings and include possible contact;
the wind samples and physical state may not be synchronized exactly. In
particular the seed-456 repeats did not experience matching wind on the chosen
command-relative clock. Differences across seeds therefore cannot isolate a
wind force law; controlled paired airborne trials remain necessary.

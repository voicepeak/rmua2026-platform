# Wind generator: constrained validation

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

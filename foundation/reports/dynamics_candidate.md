# Candidate dynamics, not calibrated

The separately implemented `foundation.quad` uses NED world and FRD body
coordinates with a first-order filtered control signal proportional to thrust,
and full rigid-body angular coupling. It rejects out-of-range inputs; neither
official saturation nor timeout is established by that choice. External forces
are explicit inputs and no wind, collision, or force-field model is asserted.

The thrust relationship follows the public AirSim RotorActuator source at
commit `1ca93f6f77e4e8a39b2b241c1fe2764da4d7dd41`:
https://github.com/microsoft/AirSim/blob/1ca93f6f77e4e8a39b2b241c1fe2764da4d7dd41/AirLib/include/vehicles/multirotor/RotorActuator.hpp
The shipped simulator's exact AirSim revision, integration and parameterization
have not been verified, so this is a candidate rather than a claim of equivalence.
The update order now follows the observed binary path described below, while
the correspondence between candidate and official clocks is not established.
The historical allocator treats
the published coefficients as per-RPS-squared, producing a hover command near
0.176, but that interpretation and motor lag remain provisional.

Unit tests check signs, hover and deterministic no-force motion for a supplied
parameter set; they do not validate those parameters against the simulator. The
old platform quad instead maps command to rotor speed and squares that speed;
the two models must be compared against independent official excitation data.

For two exploratory differential recordings, a 0.4 s replay starting 0.35 s
after the first command yielded maximum position errors 0.0118/0.0116 m,
attitude errors 0.642/0.465 degrees, and angular-rate errors 0.042/0.034 rad/s.
Initial velocity is estimated from posthoc pose, rotor state is assumed equal to
command, and time alignment assumes command and observation clocks agree. The
seed-456 run repeats the same excitation; neither is an independent held-out
waveform or a validated physical delay test.

The command-to-debug-output edge measurements are stored as `pwm_edges.json`
beside each capture. First matching debug arrivals in these three cases span
roughly 12.6–62.5 ms. Command times are after publication and debug times are
subscriber arrivals; this is not an actuator, motor or physics latency. Short
phase windows and repeated patterns may make matching ambiguous.

An additional seed-2026 run used a different, opposing diagonal PWM waveform
(`d04_yaw_seed2026_20261107`). Positive and negative phases produced fitted
body yaw acceleration +0.187 and -0.196 rad/s²; the provisional instantaneous
coefficient-based predictions were +0.192 and -0.192 rad/s². A 0.30 s replay
window yielded maximum position/attitude/angular-rate errors of 0.0056 m,
0.079 degrees and 0.0096 rad/s. This excitation was not used in the earlier
single-motor fits, but a single short run, assumed initial rotor state, derived
velocity, unverified physical latency and ground-start transient prevent it
from satisfying the plan's held-out, multi-amplitude acceptance gate.

A separate seed-456 run with a smaller diagonal amplitude ±0.002 produced
fitted yaw slopes +0.1216/-0.1347 rad/s² versus ±0.1280 predicted. Its 0.30 s
exploratory replay had maximum position error 0.0052 m and attitude error
0.059 degrees. This is an additional amplitude, still subject to the same
unverified clocks, initial conditions and short ground-start trial limitations;
it is not enough to accept the model across the requested operating envelope.

Three fresh seed-123 processes ran the same short collective waveform at
requested 20/50/100 Hz. Observed median command intervals were 50.19/20.24/10.23
ms. Aligning pose arrival by the first published command, with each initial
position removed, the 20 and 100 Hz position traces differed from the 50 Hz
trace by at most 0.005 and 0.003 m over 0.05–1.49 s. Edge timing differed by
up to about 13 ms, and the 0.12 phase included ground contact. One run per
frequency, unknown initial rotor state, unsynchronized physical actuation and
subscriber arrival latency preclude claims about command timeout, actual physics
step or general frequency invariance. Full data/hashes: `d06_cadence_pilot.json`.

An exploratory first-order fit to the 50 Hz run's .20→.16 PWM transition used
IMU body-z acceleration and host monotonic arrival time. The best search-grid
delay was 16 ms and the time constant hit the 2 ms lower search bound: the time
constant is therefore **unresolved**, not an identified motor parameter. The
same frozen curve had 0.151 and 0.046 m/s² IMU RMSE on the separate 20/100 Hz
processes respectively. These are different-rate repeats of the same waveform,
not independent excitation. Transport/scheduler/sensor delays, start-state
differences and possible contact are inseparable here; no parameter is promoted
to the physical model. See `vertical_step_pilot.json` for full metrics/hashes.

Two new independent opposing roll/pitch pulse trials use amplitudes ±0.003
(seed 2026) and ±0.002 (seed 456), each 0.14 s per direction. At ±0.003,
roll slopes were +4.016/-4.021 versus ±4.084 rad/s² predicted; pitch slopes
were +2.747/-2.743 versus ±2.763. At ±0.002, roll slopes were +2.677/-2.709
versus ±2.723; pitch slopes were +1.825/-1.814 versus ±1.842. The 0.45 s
exploratory replays had maximum attitude error 0.693/1.064 degrees and maximum
angular-rate error 0.0716/0.0900 rad/s, respectively. See the two
`d03_roll_pitch*` dataset directories for raw data, hashes, edge timing and
full residuals. They use posthoc initial estimates, do not isolate physical
latency, and cover only a narrow ground-start envelope; they do not meet the
full D03 or held-out Gate B acceptance requirements.

An exploratory 35-point replay sensitivity grid selected a zero command delay
and 2 ms motor filter on the ±0.003 roll/pitch run by angular-rate RMSE; both
values lie at the search boundaries. RMSE fell from 0.0445 to 0.0307 rad/s on
that run, from 0.0557 to 0.0453 on the smaller roll/pitch repeat, and from
0.00535 to 0.00492 on the separate yaw waveform. The improvement is not proof
of either a zero physical delay or a 2 ms motor time constant: the initial rotor
state is assumed, command and sensor clocks may differ, and the grid/metric were
chosen after data collection. Neither model parameter is changed based on this
exercise; see `replay_delay_sensitivity.json` for all candidates and hashes.

Two new independent seed-123/456 processes exercised short pitch pulses followed
by near-hover and reduced common thrust (`d05_braking_*`). Both completed with
the 0.8 m position and 0.5 rad tilt guards and their dedicated containers stopped.
The seed-123 pose-derived velocity estimate changed from approximately
(-0.125, 0, -0.407) m/s at 0.75 s to (-0.172, 0, -0.292) m/s at 0.90 s
relative to the first command. Thus vertical ascent slowed, while horizontal
speed increased: this waveform does not establish horizontal braking. The
velocity estimates use two-sided quadratic fits and can straddle command edges.

For the 0.30–0.95 s exploratory interval, maximum candidate position errors
were 0.0302/0.0290 m and attitude errors 0.847/0.368 degrees. Position-derived
initial velocity, assumed motor state, unverified physical latency and possible
contact limit the interpretation; these runs do not complete D05 or validate
drag. These metrics used the earlier filter-before-force ordering; after
correcting rotor order, the same replays produce maximum position errors of
0.0312/0.0298 m and attitude errors of 0.986/0.500 degrees, respectively, in
the `replay_rotor_order.json` files. The initial seed-123 launch crashed before readiness and sent no PWM;
its official log is retained separately in `d05_braking_seed123_20261107`.

Two new seed-123/456 processes ran a 100 Hz small collective staircase: .18 →
.19 → .18 → .17 → .18 after takeoff. The median body-z IMU differences
before versus after each 0.01 step were -0.534/+0.522/+0.542/-0.555 m/s² and
-0.555/+0.529/+0.536/-0.531 m/s², respectively. The provisional instantaneous
model predicts a magnitude of 0.557 m/s² under its mass and thrust assumptions.
This is encouraging local consistency, not identification of thrust or proof
of free flight and force isolation. `d02_small_steps.json` records source hashes,
sample counts, fit details and limitations.

Exploratory fits over each step yield effective post-publish-to-IMU-arrival
delays of 10–16 ms and time constants of 2–8 ms; six of eight time constants
hit the 2 ms lower grid bound. These values include communication, scheduling
and sensor effects, and are not certified motor parameters. The two runs use
the same short waveform; D02 and Gate B remain open until time alignment,
actuator response, held-out inputs and acceptance thresholds are established.

## Native rotor update observation

In a dedicated, SHA-verified 12.0.0.5 process, read-only GDB breakpoints at
`0x269ab90` recorded 530 pre-update samples across four rotor objects during a
small-step waveform. The filter parameter at rotor offset `0xe8` remained
0.00499999989 in all samples. Previous filtered outputs at `0x128` ranged up
to 0.2, and prior thrust at `0x118` divided by that output had median 12.5376
for all four rotors. See `rotor_probe_summary.json` and source log hashes.

The saved `rotor_update_disassembly.txt` shows the update first copies the
filtered state at `0xec` to output at `0x128` and scales it for thrust and
torque, then calls the filter update. The latter reads the parameter at filter
offset `0x30` (rotor `0xe8`), previous state at `0x34`, input at `0x38`, and
updates `state = exp(-elapsed_ns/(1e9*tau))*state +
(1-exp(-elapsed_ns/(1e9*tau)))*input`; the two constants were read from the
verified ELF's mapped virtual addresses. `foundation.quad` now computes force
before updating the exponential filter. This establishes the observed rotor
calculation and its local configured constant, not end-to-end latency, actual
physics step, all force/torque coupling, or behavior without debugger timing
perturbation. The two initial probes at `0x269af00` captured reset, not ongoing
updates, and are retained separately.

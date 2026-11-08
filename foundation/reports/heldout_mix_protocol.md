# Combined-axis held-out pilot protocol

Predeclared before data collection: run `heldout_mix` in two fresh isolated
official 12.0.0.5 processes, seeds 31415 and 27182. Use the bounded 50 Hz PWM
player with its existing displacement, tilt, stale-pose and cleanup guards.
The mixed roll/pitch/yaw waveform is defined in `probes/pwm_collective.py`;
its precise bytes are hashed in each run manifest. Do not fit model parameters
to these recordings.

For a fixed exploratory comparison, convert the captured ROS records using
`foundation.convert_capture` and replay with the existing `Quad` model for
0.50 s beginning 0.30 s after the first command, using the observed native
5 ms rotor time constant, zero assumed command delay and the existing 2 ms
integration step. Initial velocity is estimated from the recording and rotor
state is assumed; neither represents a directly measured identical initial
state. Preserve every attempt and report failures, guards, data gaps and
contact uncertainty. Use the same procedure on both recordings without
choosing a better window after inspecting the result.

This pilot has no frozen gate-derived acceptance thresholds, complete time
synchronization, validated initial state, or established absence of contact.
It tests a waveform not used in the earlier pulse fits; it is not a Gate B
acceptance set and no pass/fail claim is authorized by this protocol.

## First attempts and smaller-amplitude follow-up

Both original waveforms reached the motion guard at approximately 1.01 s;
their separate process cleanup succeeded. Subscriber pose samples during the
player lifetime reached approximately 0.60/0.59 rad tilt and 0.36/0.36 m
displacement (these sampled maxima are not the exact player guard readings).
The predeclared replay window was covered. Seed 31415/27182 maximum position
errors were 0.0192/0.0214 m, attitude 1.637/1.999 degrees and angular rate
0.120/0.156 rad/s. All measurements and aborts remain in the datasets; this
does not establish acceptance or invalidate the safety guard.

Before collecting further data: run `heldout_mix_small` using the same two
seeds and procedure. This retains the exact sequence and divides all four
mixed-phase PWM deviations from 0.18 by three; no safety limits are relaxed.
Apply the same predeclared 0.30–0.80 s replay window and fixed model, and
retain the two aborted attempts separately. The smaller amplitude is a safety
adaptation after observing the first attempts, so the follow-up is not a blind
or statistically independent waveform selection.

Both smaller-amplitude processes completed and their dedicated containers
stopped. In the fixed 0.30–0.80 s window, seed 31415/27182 maximum errors were
0.0146/0.0137 m in position, 0.0610/0.0624 m/s in velocity (derived from pose),
0.393/0.460 degrees in attitude, and 0.031/0.036 rad/s in angular rate.
The maximum pose-sampled tilt over the player lifetime was 0.352/0.370 rad;
maximum displacement was 0.358/0.357 m. These are subscriber observations,
not exact guard measurements. See `heldout_mix_small_seed31415_20261108` and
`heldout_mix_small_seed27182_20261108` for raw capture, conversion manifests,
source hashes, per-sample replay errors and cleanup logs. The acquisition and
replay limitations above remain; smaller-amplitude success is not permission
to omit or reinterpret the earlier two guard aborts.

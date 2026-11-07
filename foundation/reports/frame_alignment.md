# Candidate coordinate alignment: not accepted

`analyze_frame_alignment.py` matches posthoc ROS pose timestamps to the IMU
header timestamps recorded by the sensor clock observer, then pairs the latter's
monotonic arrival time to the nearest `DRONE_WORLD` GDB sample (maximum 30 ms).
For `vehicle_parked_pwm_pilot_20261105`, 190 pairs yield a median candidate
UE origin of approximately (0.008, 0.001, -7.000) cm if UE X/Y and NED X/Y
are aligned and Z is inverted. The residual from this median is below 1 cm for
165 pairs, but the p95 is 42.5 cm and maximum 56.1 cm. The median absolute
arrival-time match gap is 2.2 ms; this is not proof that measurement times
matched, especially with debugger pauses and independently generated headers.

Two other historical injected vehicle pilots gave the same median candidate
translation but p95 residuals of 16.5 cm and 38.1 cm. Their records and hashes
are preserved in the adjacent `frame_alignment*.json` files. Many near-identical
samples occur in static intervals, so the median cannot be treated as an
independent estimate for every sample. The debugger's actor root may not be the
same physical reference point as AirSim pose; running-world transforms,
injections and sensor timing require additional verification. No candidate
origin has been promoted into the official map or collision system.

The 190 paired samples contain only three distinct GDB component-root positions
(the most frequent is repeated 90 times), whereas the paired ROS pose has 28
distinct positions. With a 0.2 m/s finite-difference threshold, 23 of 24
intervals with changing ROS pose also have more than 5 cm origin residual;
25 pairs exceed that residual overall. `frame_alignment_motion.json` preserves
the calculation and source hash. This shows that the apparent sub-centimetre
median is dominated by repeated values, while motion has a systematic mismatch.
It does not establish whether the debugger sampled the wrong reference point,
UE updates stopped, ROS state continued independently after contact, or timing
was affected by debugger pauses. The captured vehicle fixture produced actual
contact callbacks and stage injection; it is unsuitable for validating a clean
free-flight coordinate transform.

## New read-only, unforced ground-start trial

`frame_clean_seed123_20261107` used a fresh simulator, fixed common PWM and a
read-only GDB breakpoint at the previously identified actor-root access, with no
vehicle fixture or stage-state injection. One actor yielded 43 nearest direct
ROS-pose-arrival pairs. Their median inferred translation was approximately
(0.0078, 0.0010, -7.0000) cm; overall p95 and maximum residual were 0.47 and
21.70 cm. Of these pairs, 34 occurred before the player emitted its end event;
within that operational window the p95 and maximum were 0.115 and 0.501 cm.
The worst overall residual occurred after the player end event, before the
container stop; this temporal split is not proof of collision or absence of it.
All-stop debugger scheduling, ROS arrival vs source time, actor identity and
reference-point equivalence remain unresolved. The run sampled a short
ground-start vertical movement only; no general origin, orientation or map
equivalence has been certified. The full output, hashes, capture, breakpoint
script, binary check and cleanup logs are stored in the dataset directory.

Two further read-only fresh-process trials used seed 456 with opposing
roll/pitch pulses and seed 2026 with collective vertical input. Before player
exit, the two trials had 25 and 32 paired samples, maximum translation residuals
0.60 and 0.85 cm, and the same median candidate translation to within numerical
rounding. The seed-456 paired pose ranges were approximately 0.079 m X,
0.266 m Y and 0.361 m Z; the vertical trial had essentially no horizontal
motion. With the candidate UE-to-NED world and body Z reflection, the seed-456
pre-exit attitude difference had p95 0.0064° and max 0.0932°. Full-window
translation maxima were 9.5 and 14.6 cm, respectively, including samples after
the player exited. These tiny, ground-start movements, source-arrival pairing,
all-stop GDB and unproven actor identity limit the conclusion. They support a
candidate for further tests; they do not establish a globally valid map origin,
body-frame convention across larger rotations, or absence of contact.

In the parked-vehicle pilot, the captured `DRONE_WORLD` actor and component
pointers also appear as the drone actor and component in `HIT_SOURCE` events.
This supports identifying the sampled component with the callback's drone
component in that run, but does not prove its transform tracks the AirSim pose
reference continuously. The movement discrepancies overlap the period in which
the fixed PWM sequence and contact callbacks occur; this further limits that
fixture's use as a clean coordinate calibration.

# Force-field evidence and unknowns

The locally preserved extracted rules text `D:/RMUA2026-re/rmua_doc_0.txt`
identifies itself as V1.0.0 dated 2026-01-30; its SHA-256 is
`784cddc24b675a8d004b822e7f2bdc8e5bff9684a8bdbdd978340711eeeaade4`.
It describes reduced power outside roads and the central hub, recovery on
return, and failure after cumulative field exposure exceeds two seconds. This
is a rule statement, not verification of the simulator's implementation or the
applicable final competition rule version.

The platform's `ForceField.evaluate` uses a corridor approximation and fixed
0.35 PWM scale; neither the corridor nor this scaling point/value is established
as official. Do not carry those assumptions into the foundation simulator as
verified effects. The field boundary geometry, natural entry/exit delay, counter
reset and complete action chain remain U. The forced-flag probe below provides
initial evidence for the scaling point and factor. Existing `force_wind_*` experiment
names refer to forced wind-stage injection, not tests of this static field.

## Observed out-time branch

The historical raised-connector trace in
`D:/RMUA2026-re/repo/experiments/hub_predicate_trace_20261104/` contains
three recorded zero-predicate intervals alternating with three one-predicate
intervals. The counter rose approximately 0 → 0.4 → 1.3 → 1.9 seconds over
the zero intervals and held its previous value during intervening one intervals.
The independent `out_time_trace_validation.json` checks those qualitative
observations and retains the source hash. Predicate transitions were recorded
before the stored flag update, so they are not a complete per-tick signal;
the run used GDB and one particular route/stage.

The archived `hub_lift_cause_20261104/outtime_branch.objdump.txt` shows that
the observed branch adds a float32 tick delta to `GameState+0x1ac` only when
`+0x180` is zero; it requests state 12 when the accumulated value is strictly
greater than `GameConfig+0x2cc` and `+0x2d4` is zero. The corresponding
`foundation.out_time.OutTimeCounter` is an explicitly driven candidate for
this branch, including cumulative time and the suppression flag. It does not
derive the flag from position, decide when stages reset the counter, implement
the entire game state machine, or simulate thrust degradation. Neither the
historical trace nor this implementation closes F01.

## Forced-flag rotor comparison

In fresh isolated seed-456 processes, the breakpoint at `0x2fb9184` forced the
predicate return to 1 or 0 on every observed tick. Both runs set the overtime
suppression byte and issued the same ground-start common PWM waveform. With
flag 1, the sampled rotor's settled input/filter/output for PWM 0.20 were
approximately 0.20/0.20/0.20 and prior thrust 2.50752; with flag 0 they were
0.20/0.05/0.05 and 0.62688. For PWM 0.16, the corresponding thrust values
were 2.00602 and 0.50150. A separate seed-2026 forced-zero run also recorded
the filter target itself: 0.05 for input 0.20 and 0.04 for input 0.16.
The first seed-123 forced pair additionally showed unchanged debug PWM while
the flag-1 run lifted and the flag-0 run remained on the ground. See
`field_rotor_forced_comparison.json` and the five source dataset directories.

These observations support a **0.25 target scale before the first-order rotor
filter** in the tested forced condition, rather than the previous 0.35 PWM
assumption. The unfiltered input/debug PWM remained unchanged. `Quad.step`
and `PhysicsSim.step` accept an explicit `rotor_target_scale` for research;
no spatial zone automatically enables it. Only one rotor was sampled, GDB
changed timing, the flag and timeout suppression were injected, and the runs
were ground-start. Natural boundary logic, event-to-rotor code path, transitions,
F01 acceptance remain unverified. The controlled four-rotor follow-up below
adds coverage of other rotors and a commanded restoration.

## Controlled entry and recovery of the flag

Two additional isolated ground-start trials held all four commands constant
while the debugger forced the predicate 1 → 0 → 1 on a preset host-monotonic
schedule and suppressed overtime. In seed 123 at PWM 0.12, all four recorded
rotor targets and filtered values settled at 0.12 → 0.03 → 0.12; their prior
thrust values settled at approximately 1.50451 → 0.37613 → 1.50451. In seed
456 at PWM 0.15, all four settled at 0.15 → 0.0375 → 0.15, with prior thrust
approximately 1.88064 → 0.47016 → 1.88064. Inputs remained unchanged in the
three sampled windows. The transition times, per-rotor samples and hashes are
in `field_toggle_seed123.json` and `field_toggle_high_seed456.json`.

These observations support four-rotor attenuation and restoration under the
injected flag. The vehicle remained on the ground, the natural predicate was
1 at both injected transitions, GDB paused the simulator, and samples were
limited to at most one per 60 ms per rotor. They do not establish natural
spatial boundaries, actual transition latency, airborne dynamics, or full F01
behavior.

## Event-to-rotor path

A third injected-toggle trial added hardware write watchpoints and an event
consumer breakpoint. On both forced transitions the observed order was:
predicate override, type-30 event consumer with the matching 0/1 value, a
write to the sampled rotor's scale at `0x26f2fd5`, and a write to its filter
target at `0x2698992`. The watchpoint stop PC is the following instruction.
See `field_event_chain_seed2026.json` and the associated raw dataset.

The saved disassemblies show the predicate-change branch emits event type 30
with the flag byte, the observed consumer forwards that byte to `0x26f2ea0`,
and that function chooses a per-rotor scale of 0.25 for zero or 1.0 for
nonzero. The constants were read from the SHA-verified 12.0.0.5 ELF at virtual
addresses `0x6c0e48` and `0x6c0a14`; it iterates the rotor array and writes
the selected scale. The target writer multiplies the raw control value by this
scale before clamping and writing the filter target, while storing an
independently clamped raw value. `field_event_producer_disassembly.txt`,
`field_event_consumer_disassembly.txt`, `rotor_scale_writer_disassembly.txt`
and `rotor_target_writer_disassembly.txt` preserve the instructions.

This connects the forced event to the observed write in this build and trial;
the all-stop watchpoints perturb timing and only one rotor's writes were
watched. It does not establish natural zone membership or airborne F01 parity.

## Exploratory airborne attenuation replay

Two fresh seed-123/456 runs commanded common PWM 0.20, with the same injected
1 → 0 → 1 predicate schedule and overtime suppression. The vehicle rose and
subsequently returned to ground; four rotor observations again showed the
0.20 → 0.05 → 0.20 target pattern. A short arrival-time window approximately
0.36–0.83 s after the player start was replayed twice from the same estimated
initial state: the candidate with 0.25 target scaling had maximum position
errors 0.0674/0.0427 m, versus 0.2404/0.1845 m with scaling disabled. The
target switch was estimated from the median first low-target observation among
four rotors (approximately 0.620/0.638 s); raw target and pose samples, all
metrics and source hashes are retained in `field_air_replay_seed123.json`,
`field_air_replay_seed456.json` and the corresponding datasets.

This is an exploratory same-recording comparison, not an acceptance or
independent holdout: the initial velocity is derived from posthoc pose, the
initial rotor state is assumed, subscriber arrival times include latency,
target samples are throttled, and GDB pauses execution. The chosen replay
window precedes the later observed ground return, but absence of earlier
contact is not independently proven; the candidate has no ground contact
response. The runs force the predicate instead of crossing a natural boundary,
and do not validate airborne recovery or F01.

## Short forced recovery after takeoff

Two additional seed-123/456 trials used constant 0.20 PWM and a shorter
injected flag-zero window (scheduled 0.60–0.76 s after player start). During
the 0.40–1.20 s analysis window, measured NED z ranged from 0.088–0.242 m
and 0.116–0.255 m respectively; the ground-start reference was approximately
0.31 m. This supports a period of elevation but does not independently rule
out earlier contacts or establish collision clearance.

The replay uses the median first sampled low and restored rotor targets as
the two switching times, the same posthoc initial state for each comparison,
and the previously identified 0.25/1.0 scale. Maximum position errors for the
two runs were 0.041/0.109 m with attenuation and recovery, 0.653/0.523 m with
no attenuation, and 0.425/0.544 m with no recovery. See
`field_recovery_replay_seed123.json` and `field_recovery_replay_seed456.json`
for times, samples, hashes and full metrics. The models omit collisions and
wind, the first sample of each target can lag the physical update, initial
velocity comes from posthoc pose, and all-stop GDB perturbs timing. This is
an exploratory same-run comparison under injected flags, not a natural
boundary test, a held-out calibration, or F01 acceptance.

F01 still requires an independently timed controlled crossing with pose, raw
commands, IMU, debug rotor observations and state transitions, including two
separate excursions with a return between them. Boundary/contact/field effects
must be distinguished, and the observed delay cannot be inferred from the
command-to-debug PWM timings alone.

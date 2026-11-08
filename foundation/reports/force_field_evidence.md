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

The saved predicate disassembly (`field_predicate_disassembly.txt`) further
shows an early raw float-field comparison, followed by a weak-reference flag
and four short-circuit object-array checks. The called accessors read either a
byte flag (`+0x220` or `+0x238`) or perform a signed comparison between two
32-bit fields (`+0x288 > +0x2b4`); the supporting instruction listings are in
`field_regionA_disassembly.txt`, `field_regionB_disassembly.txt` and
`field_regionC_disassembly.txt`. This explains why geometry alone cannot yet
reproduce the predicate: the membership state and object types feeding these
fields must also be established. The float operands and object fields remain
unlabeled; their spatial meaning is not inferred from these instructions.

## Candidate road box volumes and event comparison

The serialized `BP_Segment_Rot` Blueprint contains three `BoxComponent`
templates: `ChargeArea`, `TriggerEnter` and `TriggerExit`. Their respective
local half-extents are (1400, 550, 350), (990, 600, 350), and
(32, 550, 350) UE cm. Composing template and serialized attachment chains
produces 1,413 candidate boxes over 471 road actors, with no unresolved chains;
the map and Blueprint hashes are in `OL1_road_box_volumes_candidate.json`.
`CandidateBoxVolumes` queries point containment and signed distance under an
explicit UE/NED origin. It does not equate those boxes with force-field zones.

In the existing seed-123 unmodified-stage segment-event trace, all 99 events
have an accepted nearest GPS callback. Under the candidate startup origin,
98 of those points fall inside at least one `ChargeArea` and 93 inside at
least one `TriggerEnter`; none fall inside any `TriggerExit`. The complete
breakdown and source hashes are in `road_box_event_comparison.json`. This is
an exploratory broad union across *all* road actors, not a match to each event's
spline or segment; callback arrival and GPS position are not synchronized, and
the drone is not a point. In particular, the absence of point containment in
`TriggerExit` is an unresolved mismatch, not proof that those components fail
to generate overlaps. Blueprint defaults, runtime transforms/profiles, actor
association and object membership still require validation before using these
boxes to drive degradation or claiming F01.

### Spline and tentative segment association

The recorded event spline's FName pair matches an independently captured
runtime inventory entry whose root position uniquely matches serialized spline
`1_1`; restricting the above search to its 33 road actors leaves 99 candidate
boxes and the same broad-union counts. This is a cross-process FName/root
correspondence, not decoded object identity. The actors' serialized numeric
suffixes form the consecutive sequence 242–274. A hypothesis selected after
examining this trace maps event indices 32–0 to actor suffixes 242–274.

Under that unverified mapping, the 33 enter-type points have a median signed
distance of -0.207 m from their associated `ChargeArea` boundary, with 0.425 m
95th-percentile absolute distance; the 33 exit-type points have median signed
distance +0.271 m and 0.493 m 95th-percentile absolute distance. The third
event type lies deeper inside. Nearest GPS-arrival gaps reach 42.6 ms and the
drone has finite extent, so a nonzero residual is expected but cannot by itself
be assigned to either cause. Full data and hashes are in
`road_box_events_segment_hypothesis.json`.

This substantially narrows a plausible road-overlap mechanism but the mapping
was chosen from the same data, and other predicate branches exist. Independent
runtime actor-to-segment identification and synchronized enter/exit tests are
needed before those boxes can control the field automatically. `TriggerExit`
point containment remains unverified and F01 remains open.

A separately launched 4 m/s repeat with the mapping fixed in advance observed
33 events on the first 12 segments. Enter/exit point-to-boundary absolute
distance p95 was 0.261/0.354 m, respectively. The full protocol, partial
coverage and remaining uncertainties are in `segment_repeat_protocol.md`;
this supports the tentative association but does not close F01.

### Road, port and factory union as an opt-in proxy

The serialized OL1 port and factory Blueprints each contain an `Extent`
`BoxComponent`. The same attachment resolver produced 12 port and 12 factory
candidate boxes, without unresolved chains, alongside the 471 road
`ChargeArea` boxes. In the historical raised-route predicate trace, the
startup sample lies inside the candidate `1_1_Port` extent while the road
union does not contain it; the snapshot also has a passing `array40` object,
previously root-matched to a port. That initial GPS sample arrives about 1.57 s
after the predicate snapshot and is excluded from the timely comparison.

Among the seven later transition snapshots with nearest GPS arrival within
0.1 s, road-union point containment agrees with the observed predicate in six;
one predicate-zero sample is about 0.07 m inside a road box. Adding the port
and factory boxes does not change those seven results. The full record and
source hashes are in `field_road_port_factory_predicate_comparison.json`.
These are selected transition observations, not a continuous confusion matrix,
and point position, clock pairing, the vehicle footprint and other branches
remain uncertain.

`CandidateRegionUnion` and the optional `PhysicsSim(candidate_field=...)`
provide an explicit training approximation combining the extracted road, port
and factory boxes. The simulator samples the vehicle center at each physics
substep, applies the candidate 0.25/1.0 target scale, and can report a separate
out-time request. It does not run by default and its status remains unverified:
there is no exact overlap geometry, crossing time, side-route/weak-reference
handling, event latency or official state-machine equivalence. F01 remains open.

An independent repeat on the same nominal route yielded five matches and five
mismatches in ten timely paired transition observations, contradicting any
claim of exact point-union prediction. See `field_natural_repeat_protocol.md`.
The original seven-sample result is retained as a separate pilot, and the
training approximation must not be promoted to an official field model.

### Blueprint detection sphere hypothesis

The packaged `BP_FlyingPawn` Blueprint has a `SphereComponent` named
`JianCeQiu`, with serialized radius 2 cm, relative offset (0, 0, 6) cm and
`OverlapAll` profile. Its construction-script node is a child of `Mavic3`.
The `Mavic3` mesh imports `Drone2024R_Body`, whose serialized simple collision
has one convex element with local bounds approximately (-12.99..11.88,
-15.18..15.16, -0.07..12.94) cm. These are Blueprint/asset defaults, not yet
verified runtime instance settings. Source hashes and geometry are preserved
in `blueprints/BP_FlyingPawn.json` and `drone_simple_collision_candidates.json`.

An exploratory hypothesis treats the cached runtime component as the sphere's
parent, rotates the 6 cm offset using the nearest accepted ROS orientation,
and tests a 2 cm sphere against the candidate road/port/factory boxes. In the
three earlier transition recordings it matched all 43 eligible events; three
events lacked an accepted orientation within 0.1 s. In two previously recorded
periodic datasets it matched all 626 eligible samples in each, with 11 and 21
samples respectively excluded by the orientation-gap rule. Per-sample data,
exclusions and hashes are in `sphere_field_direct_pose_*` and
`sphere_field_periodic_*`/`sphere_field_lift6_*` reports.

This geometric hypothesis was chosen after examining the prior results. The
cached component has not been independently identified as `Mavic3`, the actual
overlap event has not been traced to `JianCeQiu`, and frame conversion, instance
scale, geometry and asynchronous orientation remain assumptions. The repeated
samples are correlated and cover similar routes; none of this proves F01 or
justifies replacing the current default with this hypothesis. A new predeclared
experiment and direct event-component attribution are required.

Two further fresh seed-123/456 startup processes inventoried pointers owned by
the drone actor during the contact callback. The actor's `+0x130` pointer
matches the callback's drone component; `+0x348` points to a component whose
position is exactly 6 cm above that root in these near-identity startup poses.
The `+0x340` component is 6.444708 cm above it, matching the Blueprint
`Outermost` box default to the printed precision. Results and source hashes are
in `drone_components_seed123.json` and `drone_components_seed456.json`.
This strengthens the candidate association of `+0x348` with `JianCeQiu`, but
FNames/classes have not been decoded and neither the sphere radius nor its
participation in natural field events has been observed through those pointers.
The startup matches are not independent checks under rotation or flight.

A new seed-123 startup observation records the candidate `+0x348` object's
virtual-table pointer as `0x10c66b8`; the same verified binary exports
`vtable for USphereComponent` at `0x10c66a8`. The 16-byte difference matches
the usual Itanium C++ vtable address-point offset, strengthening the component
type attribution. The raw component snapshot contains float32 2.0 at several
offsets, including `0x460`, but none is independently typed as the radius;
do not promote that value as a runtime measurement. Hashes, symbol evidence and
the complete list are in `drone_sphere_vtable_seed123.json`. The type match
still does not prove this component supplies the natural field overlap event.

`CandidateSphereRegionUnion` now exposes this sphere/box hypothesis as an
explicit opt-in spatial field to `PhysicsSim`. The caller must supply the UE
local offset, radius and origin; the implementation uses the candidate UE/NED
reflection and unit component scale, and evaluates overlap at the start of each
physics substep. Point-based `CandidateRegionUnion` remains available. Unit
tests exercise sphere/box corners, body offset and simulator integration.
The spatial field remains unverified and does not imply exact crossing time,
overlap events, full zone coverage or official state-machine behavior.

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

The same two records have now also been replayed through the integrated
`PhysicsSim`/`RecordedEffects` interface with no wind model: flags switch at
the same estimated target-observation times as in the direct replay. Maximum
position errors agree to displayed precision (0.0411/0.1090 m for attenuation
plus recovery), and the no-attenuation and no-recovery alternatives also agree.
The pipeline results and source hashes are in
`field_recovery_integrated_seed123.json` and
`field_recovery_integrated_seed456.json`. This verifies implementation
consistency only: both methods use the same captured runs and estimated switch
times, with all the debugger, initial-state and contact limitations above.

F01 still requires an independently timed controlled crossing with pose, raw
commands, IMU, debug rotor observations and state transitions, including two
separate excursions with a return between them. Boundary/contact/field effects
must be distinguished, and the observed delay cannot be inferred from the
command-to-debug PWM timings alone.

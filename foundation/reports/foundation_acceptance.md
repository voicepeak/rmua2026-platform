# Foundation acceptance — V2 status and historical evidence

## Current V2 status

The governing plan is [V2.0](implementation_plan_v2_zh.md); the current module-by-module record is [V2 execution status](v2_execution_status_zh.md). The status below is the 2026-11-09 evidence-based closeout; the historical sections after it are retained as provenance, not as current PASS evidence.

**Decision: overall PARTIAL. No module currently has the complete independent evidence and frozen test thresholds needed for a V2 PASS. G1 parameter identification is BLOCKED.**

| Gate | Status | Current evidence | Remaining limitation / allowed use |
|---|---|---|---|
| G0 baseline, frame, PWM contract | PARTIAL | 12.0.0.5 binary hash, settings-semantic comparison, frame/attitude round-trip tests, PWM message snapshot; four clean free-fall tails (two from aborted guard runs) support the pose source time base to about 0.2% | Runtime settings were not read back; accepted PWM range/clipping/hold/timeout and the shipped AirSim revision remain unknown. Use only as a candidate contract. |
| G1 wind-free collision-free PWM physics | PARTIAL; parameter identification **BLOCKED** | P01–P06 captures with repeats, fixed-window repeatability, two calibration rounds, sealed three-run holdout; exploratory free-fall clock check; four-motor differential x/y signs 12/12 over three processes | Physical command time, motor state, initial velocity, contact and wind remain unexcluded; yaw/z motor signs unresolved. The two rounds stop on grid boundaries; no further parameter grid is allowed. Do not claim calibrated high-frequency PWM training. |
| G2 map and collision | PARTIAL | OL1 and OL2–OL12 serialized candidates; default 508 instances; all optional OL1 12,609 and OL2–OL12 12,597; 12 gates; 12 building + 480 rack cooked meshes; 12/12 gate, 64/64 road, 64/64 wall and 25/25 complex-as-simple self-probes; new official contact-probe runs match a road segment (`BP_Segment_Rot_C_242`, 939 grouped callbacks, hit-point residual ≤3.6e-4 cm) and the `1_1_Port/Building_Port` wall (115 callbacks, ≤2.1e-4 cm); a route-following run reached the hub and contacted `BP_Building_Junction_7`; static RE of activation `0x2fc13d0` shows the named route-gate profile toggles between `NoCollision` and `BlockAll` (solid when active), not a trigger/overlap aperture; 33+30 runtime scoring components are a traceable detection-gate candidate layer; diagnostic paths crossed road1 gate 32 within 3.07 cm and the early sequence 32→31→30→29 with zero post-command contacts (closest 0.176/0.018/0.014/0.020 m); a full candidate stage-1 score probe observed all road1 indices 0–32 and score 0→33 (state 1→12 at score 32) | Detection-gate aperture/trigger semantics, factory entrance, road3's 30 gates and general free-space clearance remain unvalidated; the full score run exited on its maximum-flight-duration bound and its local commands file is missing; score events are not one-to-one with candidate gates; the route is not a gate-by-gate geometric alignment (median 2.4–2.7 m offset); the 12 named gate actors are solid profile toggles when active and are not established detection gates; the route run was stopped by the hub about 9.6 m from `1_1_Gate`; `meter_report` returned success without a factory-entry log and is not validation. Candidate hit-point offset and source identity remain inferred. No free-space guarantee. |
| G3 wind and degradation | PARTIAL / **UNVERIFIED** | 18/18 injected-stage keyframes; controlled drag-face and six-face force agreement; forced 0.25/1.0 target scale; controlled W02/F01/F02 interface benchmark; candidate 2 s counter | Natural wind generation/delivery, W01, natural force-zone boundary and F02 remain unverified; G3 depends on G1. Treat only as an explicitly labelled approximation. |
| G4 obstacles and batch | PARTIAL | Seeded line/vehicle generation; 90 synthetic high-speed sweeps; 768 actual-map high-speed crossings with zero candidate misses; single/batch consistency; vectorized free flight; dense-map benchmark; sub-box vehicle clearance audit; official real-time factor about 1.0000–1.0004 | No same-operation official throughput comparison was obtained. Official runs advance near real time, so local environment-command rates cannot be converted to an official 64/256 throughput ratio. Vehicle sphere cover remains conservative; official vehicle/road contact is unverified. |
| Overall | **PARTIAL** | [run_smoke_test.py](../../run_smoke_test.py) and the gate reports above | Do not combine PARTIAL/BLOCKED modules into a claim of a complete 1:1 simulator. |

### Prohibited claims at this status

- Do not claim official PWM dynamics equivalence, calibrated motor time constants, or reliable high-frequency PWM policy training.
- Do not claim complete OL1/OL2–OL12 collision coverage or free-space guarantees from a candidate miss.
- Do not claim official natural wind, force-zone boundaries, 0.25 scaling, 2 cm sphere, 6 cm offset or 3.8 cm margin as official parameters.
- Do not claim an official throughput ratio or official same-machine comparison from the local benchmarks.

### Verified deliverables

- PWM-only single and batch API: `foundation.runtime.RmuaPhysics`, `foundation.batch.BatchRmuaPhysics`.
- Candidate maps, collision manifests and gaps: `foundation/maps/OL1/collision_manifest.json`, `foundation/maps/OTHER_MAPS.md`; cooked-mesh audit `v2_map_cooked_mesh_audit.json`; geometry probe `v2_map_geometry_probe.json`.
- Wind/force-field layering and UNVERIFIED intervals: `v2_g3_evidence_matrix.md`; controlled W02/F01/F02 interface benchmark: `v2_g3_controlled_benchmark.md` / `.json`.
- Calibration identifiability record: `v2_identifiability.md` / `.json`; exploratory free-fall clock consistency: `v2_ballistic_clock_audit.md` / `.json`; motor direction audit: `v2_motor_direction_audit.md` / `.json`; official isolated run record: `v2_official_run_20261109.md`; official real-time factor: `v2_official_realtime_factor.json`.
- Batch benchmarks: `v2_batch_reference_benchmark.json`, `v2_batch_vectorized_benchmark.json`, `v2_batch_scenario_benchmark.json`, `v2_batch_map_contact_benchmark.json`, `v2_batch_dense_map_benchmark.json`, `v2_map_sweep_miss_benchmark.json`; vehicle clearance audit `v2_vehicle_map_clearance_audit.json`.
- Official G2 contact probes: `g2_official_contact_20261109.md`, `g2_contact_road_seed123_20261109.json`, `g2_contact_portwall_seed123_20261109.json`, `g2_contact_gate1_seed123_20261109_v2.json`; runner `foundation/run_contact_waypoints_local.py`, pilot `foundation/probes/contact_waypoints.py`, analyzer `foundation/analyze_contact_waypoints.py`.
- G2 direct-function checks: `g2_gate_collision_re20261109.md` (route-gate `NoCollision`/`BlockAll` toggle) and `datasets/g2_meter_report_probe_seed123_20261109/` (inconclusive `meter_report` service probe).
- G2 detection-gate candidates: `maps/OL1/detection_gate_candidates.json`, `foundation/detection_gates.py`, `v2_detection_gates.md`, `v2_detection_gate_alignment.json`; first probe `g2_detection_gate32_20261109.md`, `g2_detection_gate32_seed123_20261109.json`; early sequence `g2_detection_gate_sequence_20261109.md`, `g2_detection_gate_sequence_seed123_20261109.json`; score-trigger probe `g2_detection_gate_score_20261109.md`, `g2_detection_gate_score_seed123_20261109.json`; full road1 sequence `g2_score_route_20261109.md`, `g2_score_route_seed123_20261109.json`.
- Granular sub-item ledger: `v2_evidence_ledger_20261109.md`.
- Smoke test: `.\.venv\Scripts\python.exe run_smoke_test.py` passes as a synthetic integration check only.
- Test command: `.\.venv\Scripts\python.exe -m pytest -q foundation\tests` — 159 tests passed at the recorded closeout.

### Conditions to reopen a gate

1. G0/G1: new independent, low-perturbation measurement of physical command time and motor state, with an independently measured initial velocity and contact/wind exclusion. The sealed `v2_holdout_axis` data remain unread until thresholds are frozen.
2. G2: official contact/clearance observations for a detection-gate aperture and a factory entrance remain missing; road and wall contacts are now recorded, but collision channels and free-space coverage are not resolved.
3. G3: natural-course continuous wind/force-zone and trajectory data, or an explicit decision to close G3 as approximate/UNVERIFIED.
4. G4: explicit consent to start isolated official simulator containers for a same-machine throughput method, or an explicit decision to close G4 without that comparison.

### 2026-11-10 coverage round

- A machine-readable coverage matrix now exists: `foundation/reports/v2_coverage_matrix_20261110.md` / `.json`.  It records OL1 stage1 road1 33/33, road3 30/30 under injected stage 2 (30/30 component triggers under stage 1), and OL1 stage3–6 plus OL2–OL12 runtime detection/score/contact/factory as UNVERIFIED/PARTIAL.
- Six OL1 stage route manifests and diagnostic waypoint files were generated from `data/tables/routes.csv` (`foundation/maps/OL1/stageN_route_manifest.json`).  These are reconstructions, not official trajectories.
- Road3 reached 30/30 under a controlled stage-2 injection: v7 routexy baseline + gate-normal corridor gates 11-7 at +1 m and 6-0 at +3 m. This is not official stage progression and stage1 direct remains 0/30.  Details: `foundation/reports/g2_road3_attempt_20261110.md` and `g2_stage2_inject_probe_20261110.md`.
- G1 remains BLOCKED after two bounded non-stopping attempts; `perf` is absent from the probe image and `process_vm_readv` did not yield decoded official rotor state.  Details: `foundation/reports/v2_g1_native_attempt_20261110.md`.
- `foundation/training_env.py` adds `RmuaPhysicsVecEnv` as an explicitly approximate PWM/6DOF batch sandbox with truth-research observations and an `observable_state` placeholder.
- Stage 3 injection declared 31 components but produced `nan` transforms and score 0; OL2 map redirect worked but produced no flyable state or SEG/DUMP.  Both are recorded gaps.
- No gate status is upgraded by these items.  Overall remains PARTIAL.

The rest of this document retains the original historical evidence; its earlier table and “next” actions are not the V2 current status.

## Initial evidence snapshot

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

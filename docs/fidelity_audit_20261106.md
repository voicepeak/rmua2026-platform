# Fidelity audit and recovery plan — 2026-11-06

Scope: static review of the local stage-1 training/scene/evaluation code,
existing upstream reference models and the recorded OL1 runtime diagnostics.
This is not a complete proof of the official simulator or every future stage.
No new official flight or training was run during this audit.

## Evidence and boundaries

| Area | Evidence today | Open gap |
|---|---|---|
| Route and gates | OL1 map spline data, measured end goal and 33+30 ordered scoring components from seed-123 runtime inspection | Hub connector is constructed; trigger volume/normal, stage transition and generalization beyond OL1 remain unverified. Some audit callbacks were after Finished. |
| Plant | Local policy was trained on 100 Hz kinematic world-velocity control; official diagnostics reached 200 m with limits and truth-assisted bridge | Official velocity-loop dynamics, update rate, tilt, inertia, startup and braking have not been identified against the training plant. The separately available `quad` model is also not validated as equivalent. |
| State and control | NED pose z vs up-positive velocity command has been checked; truth diagnostics and logged shadow actions exist | Sensor/EKF adapter does not exist as a validated deployment path; velocity/attitude/omega, gate/phase estimates, timestamps and world/body conversions need independent replay tests. |
| Geometry/collision | Static map and component root transforms are exported | Road mesh, detection volume and true collision shapes are not reconstructed in the training plant. Current corridor and swept-plane tests are approximations. |
| Rules | Ordered local gate handling and separate start/end zones are implemented | Exact start/end state transitions, cumulative force-field rule, failure causes and six-stage continuity need direct differential verification. |
| Randomness | Held-out local seeds 1201/1302/1403/1504/1605 were not training seeds | These are the same OL1 track with wind and proxy obstacles disabled; they do not validate official random trees, vehicles, lines, sensor noise or maps. Official actor diagnostics used one seed, 283441. |
| Safety and evidence | Monitor checks a fixed simulator binary hash; bounded guarded tests and raw logs retained | 200 m stop was followed by simulator drift and later collision. Zero-command braking and simulator process termination are not yet reliable; GDB changes scheduling. |

## Concrete local findings

1. In `src/rmua/envs/sim_fast.py`, kinematic mode computes `wind` and the
   `ForceField` factor but neither affects the kinematic update. In quad mode
   those effects are applied differently. This does not affect the documented
   stage-1 run's disabled wind, but makes future kinematic robustness tests
   misleading if they assume the enabled effects influence motion.
2. The force timer uses `np.where(inside_corridor, 0, ...)`, resetting on
   return. The available [rules-v1 text mirror](https://github.com/guo1hao/RMUA2026/blob/main/doc/RoboMaster2026_rules_v1.txt)
   describes cumulative time outside; verify against the applicable official
   version and runtime before changing behavior. The current corridor itself
   is a proxy.
3. Collision handling checks a lower vertical bound and lateral threshold,
   not an official road mesh or ceiling. `in_corridor` currently uses
   `abs(vertical) <= corridor_height_m` with default 5 m; this is a proxy
   interval around the path, not proof of the official 5 m-high road volume.
4. Evaluation continues stepping all vector environments until all finish.
   Terminal phases are protected from further failure transitions, but their
   physics and projected progress still update. A direct local check with an
   already-DONE env and velocity 5 m/s moved it 0.05 m in one step; repeated
   terminal `done` was false. This can affect post-terminal progress/aggregate
   metrics and must be fixed or snapshotted per episode. It does not by itself
   invalidate the recorded success rate or prove the reported speed wrong.
5. `training/checkpoint.py` says it stores a geometry fingerprint, but metadata
   contains map/mode, gate count, dimensions and length with 0.5 m tolerance,
   not a path/gate/connector hash. Different geometry can pass those checks.
   Bind runs and artifacts to deterministic geometry, environment code/config,
   observation and adapter hashes, preserving versioned compatibility.
6. `docs/data_contract.md` describes 30 observations and older route status;
   the implementation currently exposes 33, including path-ahead. Align the
   documentation and version the observation schema, units and frames.
7. The diagnostic bridge must treat `pose_gt`, internal memory Score and state
   as calibration-only instrumentation. In the 200 m guarded run the actor did
   not own unrestricted control; no complete official stage has been achieved.

## Recovery order and acceptance gates

1. Preserve the current workspace and the existing checkpoint as a local
   reference, not a release. Make diagnostic runs self-terminating: fresh
   process per trial, timestamped commands, stale-observation fail-safe,
   bounded observation and confirmed simulator exit even with GDB attached.
2. Build a deterministic experiment manifest with source/checkpoint/binary,
   scene and config hashes; seed, control mode, timestamps and full traces.
   Validate coordinate, velocity-sign, quaternion and body/world transforms
   with known input cases and official telemetry. Separate observation and
   action conversion errors from flight dynamics.
3. Measure official response using bounded, safe VEL experiments: hover,
   horizontal/vertical step and reverse, yaw step, braking and delay. Compare
   local and official at matched initial state, input history and time base;
   record position/velocity/attitude, saturation, latency and repeatability.
   Use calibration and independent validation seeds/trials. Test both the
   kinematic approximation and 6-DOF candidate; select or fit based on
   held-out errors rather than assuming the more complex model is correct.
4. Trace and calibrate start/gate/end and force/collision events and the hub;
   add regression cases for the concrete issues above. Compare official and
   local trigger ordering, location, outcomes and times on matched trajectories.
   Treat separately unobserved stage transitions and confidence limits.
5. Freeze measurable thresholds only after collecting a pilot baseline: e.g.
   state/trajectory error distributions and event agreement over held-out
   trials, plus seed and map coverage. Track validity and unsafe contacts as
   first-class metrics; mean speed alone is insufficient.
6. Train/retrain only after the adapter and calibrated plant meet those gates.
   Start with a deployable sensor/EKF observation contract and fixed seed
   partitions; use domain randomization only over measured/plausible ranges.
   Evaluate first on local held-out seeds, then fresh official processes with
   <=4 simultaneous instances. Keep pose_gt as explicitly labeled calibration
   instrumentation, never as the final actor input.

Existing local BC/DAgger checkpoint is a useful experimental baseline and its
reported local success is not an official result. It should not be discarded
or promoted without the above comparisons.

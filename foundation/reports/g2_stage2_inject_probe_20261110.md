# OL1 stage-2 injection probe (2026-11-10)

Status: **stage-2 scoring queue activated; road3 reverse traversal increments
score.**  This is a diagnostic stage injection, not official progression.

## Method

- GDB script: `foundation/probes/stage2_teleport.gdb`.
- At native stage init `0x2fbac30`, when `$ecx==1`, the probe sets the route
  arguments to `(3,5,2)`:
  `INJECT_STAGE2 old=1,3,1 new=3,5,2`.
- On the first contact callback, the probe writes the drone component position
  to the road3 end approach:
  `TELEPORT component=... old_cm=0,0,-37.91 new_cm=54570,51960,3288`.
- Plan: `foundation/config/g2_stage2_inject_probe_20261110.json`, six gates
  `29..24` in reverse order.

## Result

The live GDB trace showed:

- `SEG idx=29 enter=1 score=0`
- `SEG idx=29 enter=0 event=1 score=0`
- `SEG idx=28 enter=1 score=1`
- `SEG idx=27 enter=1 score=2`

So the stage-2 queue is active and road3 component indices are consumed in
reverse order.  This is the first score evidence outside road1.

## Limitations

- Stage injection mutates native call arguments; it is not official stage
  progression.
- Teleport is a diagnostic GDB intervention, not a reset or race start.
- The short plan covers only six gates; a full 30-gate run is required before
  claiming road3 score coverage.
- GDB all-stop changes timing.

## Follow-up full-road3 runs

1. **Gate-centre path** (`foundation/config/g2_stage2_full_road3_20261110.json`):
   the injected stage-2 process reached score **15**, then stopped making progress
   between road3 gate 14 and gate 15 at `(729.76, 299.99, -76.11) m`.  The
   straight gate-centre segment met a blocking surface; the run timed out.
2. **Official route3 x/y + gate-height z** (`..._routexy_20261110.json`): x/y
   from the official `route_1_3` road3 samples, z interpolated on the 30 runtime
   gate centres.  This run reached score **18** in 360 s and observed indices
   **11–29** (19 indices); it hit the `maximum flight duration exceeded` bound
   before consuming the last components.  Parsed result:
   `foundation/reports/g2_stage2_routexy_seed123_20261110.json`.
3. **Longer routexy rerun** (`foundation/datasets/g2_stage2_routexy_long_seed123_20261110/`):
   the same 18/30 score and indices 11–29 were reproduced, then the vehicle
   remained near `(726.1, 227.4, -110.9) m` and could not reach gate 11.  The
   run was stopped after the repeated block.  Parsed result:
   `foundation/reports/g2_stage2_routexy_long_seed123_20261110.json`.
4. **Road3 score 18/30 at this stage; later v4/v5 reached 20/30.**  All 30 component
   centres were still traversed in the stage-1 teleport run, but that run's
   queue was inactive.  The remaining 12 score components are blocked by the
   gate-11 approach geometry; no 30/30 score is claimed.

The first run's artifacts are in
`foundation/datasets/g2_stage2_full_road3_seed123_20261110/`; the 18/30 run is in
`foundation/datasets/g2_stage2_routexy_seed123_20261110/` (including
`run/simulator.log`).

## Gate-11 fix and later attempts

5. **v4 routexy** (`..._routexy_v4_20261110.json`): route3 x/y lifted 4 m,
   4 m west around gate 11, speed 5 m/s.  Score reached **20/30**, observed
   indices **9–29**.  It passed gate 11 and stopped at the east post of gate 9
   (`BP_Segment_Rot_C_2996 Cube3` at `(716.9, 189.8, 120.9) m`).
   Evidence: `foundation/reports/g2_stage2_routexy_v4_seed123_20261110.json`.
6. **Gate-centre +4 m** (`..._gatecenters_lift4_20261110.json`): the teleport
   target was 4 m above the road3 end and the vehicle could not climb off the
   ground; no score.
7. **Gate-centre +3 m ramp** (`..._gatecenters_ramp_20261110.json`): reached
   score 7 and stalled near gate 23; stopped.
8. **v5 routexy** (`..._routexy_v5_20261110.json`): shifts 5 m west near
   gate 9.  It reproduced **20/30** (indices 10–29) but stalled at
   `(718.1, 196.2, -103.2) m`, about 18 m below the gate-9 centre; the vehicle
   could not climb to the gate.  Evidence:
   `foundation/reports/g2_stage2_routexy_v5_seed123_20261110.json`.

9. **v7 early-climb routexy** (`..._routexy_v7_20261110.json`): climbed above
   the gate-9/10 road deck before descending.  Score reached **22/30**, observed
   indices **7–29**.  It stopped at the gate-7 east post
   (`BP_Segment_Rot_C_2998 Cube3` at `(704.5, 149.0, 127.0) m`).  Evidence:
   `foundation/reports/g2_stage2_routexy_v7_seed123_20261110.json`.

Best road3 score is **22/30**, with 8 score components still missing.  The
remaining blocker is the gate-7 east post.

10. **v9 gate-7 bypass** (`..._routexy_v9_20261110.json`): centred the gate-7
    passage at `(699.2, 149, -131.4) m`.  It regressed and stalled at gate 9
    with score 20; v7 remains the best run.  No 30/30 claim.

11. **Full +5 m and +8 m gate-centre ramps** (`..._lift5_ramp...`,
    `..._lift8_ramp...`): both stalled around gate 23–25 with score 5–7.
    The gate-centre path is worse than the v7 routexy path; v7 remains best at
    **22/30**.

12. **v7 repeat** (`..._v7_repeat...`): reproduced score **21/30** with indices
    8–29, stalling at gate 8.  The repeated method shows run-to-run sensitivity
    around gate 7–9; **22/30** remains the single best result, not a stable 30.
    Evidence: `foundation/reports/g2_stage2_routexy_v7_repeat_seed123_20261110.json`.

## Gate-normal corridor attempt

- Short probe from gate8 through gate7 at **+1 m**, using the gate's local-X
  normal and 8 m pre/post points: SEG `idx=7 enter=1` was observed.  This
  validates the gate-normal idea for the previously blocked gate 7.
  Evidence: `foundation/datasets/g2_gate7_normal_lift1_probe_seed123_20261110/`.
- Full 30-gate normal corridor
  (`..._normal_corridor_20261110.json`): the vehicle stalled at the gate-29
  post, observed only `idx=29`, score **0**.  Evidence:
  `foundation/reports/g2_stage2_normal_corridor_seed123_20261110.json`.
- Therefore the gate-normal corridor needs a better start/sequencing; it does
  not yet supersede v7's **22/30**.

## Gate-normal corridor v2 (pre → center only)

- Plan: `foundation/config/g2_stage2_normal_corridor_v2_20261110.json`.
- Teleport directly to gate29 pre, then pre→center for gates 29→0 at +1 m
  (no post points, no first-gate backtrack).
- Result: score **7/30**, observed indices 23–29; the vehicle passed gate22 pre
  without further score increments.  Evidence:
  `foundation/reports/g2_stage2_normal_corridor_v2_seed123_20261110.json`.
- The pre→center corridor is not sufficient; v7 routexy remains best at **22/30**.

## v11 hybrid: v7 baseline + gate11→0 normal corridor

- Plan: `foundation/config/g2_stage2_routexy_gate11_8_corridor_20261110.json`.
- Uses v7 routexy x/y down to gate12, then gate-normal pre/center points at
  +1 m for gates 11→0.
- Result: score **23/30**, observed indices **7–29**.  It passed gate 11 and
  gate 9 and stopped on the gate-6 approach.
  Evidence: `foundation/reports/g2_stage2_routexy_gate11_corridor_seed123_20261110.json`.
- This is the new best road3 score; 7 components remain.

## Final road3 30/30

- v12 plan: `foundation/config/g2_stage2_routexy_gate11_0_corridor_v12_20261110.json`.
- v7 routexy baseline, gate-normal corridor gates 11–7 at +1 m and 6–0 at +3 m.
- Result: score **30/30**, indices **0–29**, player exit 0.
  Evidence: `foundation/reports/g2_stage2_routexy_gate11_0_corridor_v12_seed123_20261110.json`.
- This closes the road3 score coverage under controlled stage-2 injection; it is
  not an official stage-2 run.

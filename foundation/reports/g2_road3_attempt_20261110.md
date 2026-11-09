# OL1 road3 score attempts (2026-11-10)

Status: **road3 0/30 -> runtime trigger observed, score still unverified.**  Three
bounded attempts are recorded; two failed on geometry/stage state and one was
stopped before its artifacts were copied.

## Attempt A — full `route_1_3` at route height

- Plan: `foundation/config/g2_score_route13_full_20261110.json` (269 official
  `route_1_3` points, NED `z=-yaml z`).
- Run: `foundation/datasets/g2_score_route13_full_seed123_20261110/`.
- Result: player aborted with `altitude envelope exceeded` at
  `(83.77, 2.91, +2.54) m` while the route target was `z=-1.45 m`; the vehicle
  sank below the `max_z_m=1.0` bound.  Road1 score reached 33 and state
  `1->12` before the abort; no road3 component was reached.
- Cause: `route_1_3` z is about 30 m below the runtime gate roots; the route is
  not a safe flight height for this diagnostic pilot.

## Attempt B — stage1 candidate route + hub connector + road3 centres

- Plan: `foundation/config/g2_score_road3_20261110.json`.
- Run directory: `foundation/datasets/g2_score_road3_seed123_20261110/`.
- Result: the vehicle reached `(616.95, -79.86, -139.02) m`, 9.6 m from the
  `1_1_Gate` centre, and remained blocked by the hub wall until the player
  timeout.  No road3 component was reached.
- This repeats the known hub-wall contact; the direct gate-centre approach is
  not a safe connector.

## Attempt C — GDB teleport to road3 gate 0

- Probe: `foundation/probes/score_teleport_road3.gdb`.
- The first-contact callback exposed the drone component and the GDB write moved
  it to `(707.2, -2.9, -134.2) m`: `TELEPORT component=0x790bb9fd1640
  old_cm=0,0,-37.91 new_cm=70720,-290,13420`.
- The completed run is `foundation/datasets/g2_score_road3_teleport_seed123_20261110/`
  (player exit 0).  Parsed result:
  `foundation/reports/g2_road3_teleport_seed123_20261110.json`.
- **All 30 road3 candidate indices 0–29 produced SEG enter/exit events**
  (60 SEG events, `observed_indices` length 30).  Score stayed **0** because the
  process was still in stage 1: the active scoring queue is road1, so road3
  component crossings do not increment score.
- This establishes runtime traversal of all 30 road3 component centres, but not a
  road3 score sequence.  It is a component-trigger record, not a 30/30 score.

## Conclusion

Road3 component roots trigger at runtime, but a valid score sequence requires the
stage-2 scoring queue.  A bounded stage-2 injection probe
(`foundation/probes/stage2_teleport.gdb`) is the next attempt; it injects stage 2
at the native stage-init call and teleports to the road3 end.  Until that run
produces score increments, road3 remains **0/30 score** in the coverage matrix,
while component traversal is **30/30 observed**.

## Final status (2026-11-10)

- Stage-1 teleport run: all **30/30** road3 component indices produced SEG
  enter/exit events; score **0** because the stage-1 queue was active.
- Stage-2 injection with official route3 x/y and gate-height z: score **18/30**
  (indices 11–29 observed), reproduced in two runs.  The vehicle could not reach
  gate 11 from the final approach near `(726.1, 227.4, -110.9) m`.
- No 30/30 road3 score is claimed.  The blocker is the gate-11 approach geometry,
  not a missing component list.

## Best achieved (2026-11-10 late)

- A v7 plan reached **22/30**; a v11 hybrid (v7 routexy baseline + gate-normal
corridor gates 11→0) reached **23/30** (indices 7–29).
- The gate-11 east post was bypassed by a 4 m west / 4 m up offset.
- The remaining block is the gate-6 approach.
- Earlier v4/v5 reached 20/30; v7 22/30; v11 is the best recorded run at 23/30.
- No 30/30 road3 score is claimed.

## Final success: road3 30/30 (controlled stage-2 injection)

- Plan: `foundation/config/g2_stage2_routexy_gate11_0_corridor_v12_20261110.json`.
- v7 routexy baseline to gate12, then gate-normal pre/center corridor:
  gates 11–7 at +1 m, gates 6–0 at +3 m.
- Result: **score 30/30**, observed indices **0–29**, player exit 0.
  Evidence: `foundation/reports/g2_stage2_routexy_gate11_0_corridor_v12_seed123_20261110.json`;
  raw log in `foundation/datasets/g2_stage2_routexy_gate11_0_corridor_v12_seed123_20261110/`.
- Limitation: stage-2 injection and GDB teleport are diagnostic interventions,
  not official stage progression.  Stage-1 direct road3 remains 0/30 because the
  stage-1 queue is road1.

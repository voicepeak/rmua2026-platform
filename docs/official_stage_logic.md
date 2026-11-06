# Official stage/route logic (2026-11-06 correction)

This page corrects an error in the earlier local proxy. Do not use the
`1_1 + 2_1` / two-gates-per-segment model for official stage validation.

## Stage logic

Official six stages: `1-3, 3-5, 5-8, 8-12, 12-10, 10-7`.

The course is a shuttle, not one route per stage:

- **Stage 1**: marker 1 -> central hub -> end of route 3. Reaching route 3's
  end triggers stage 2.
- **Stage 2**: return along route 3 to the hub -> route 5. Reaching route 5's
  end triggers stage 3.
- **Stage 3**: return along route 5 to the hub -> route 8.
- **Stage 4**: return along route 8 to the hub -> route 12.
- **Stage 5**: return along route 12 to the hub -> route 10.
- **Stage 6**: return along route 10 to the hub -> route 7 (finish).

Each stage is therefore **"previous stage's end route reversed + next route
outbound"**. It is not a single `RouteConfig` and not `1_1 + 1_2`.

Evidence:

- Official rules v1: six fixed continuous paths; each starts at the previous
  endpoint, passes the central hub and ends at the next marker.
- Remote `route_1_3.yaml` comment: `spline0 + reverse(spline2)`; this is stage 1.
- Remote `/airsim_node/end_goal` for stage 1:
  `(545.56, 519.85, -30.21)` (NED).
- The remote `route_follower` flew stage 1 at about 10 m/s and logged gates;
  this is a runtime smoke test, not a scored run.

## Gate count is not settled

- The map UAsset contains only 12 objects named `*_Gate` (`1_1_Gate` ..
  `6_2_Gate`). These look like hub/route gates, not the full detection-gate set.
- Remote `gates_vision_1_3.yaml` has 10 gates, but it is a **vision-derived
  partial list**, not the official full gate table.
- A count of 60-70 gates is plausible as the **whole-course** detection-gate
  count (roughly 10 per stage); it is not yet verified for a single stage.
- Do not use 2, 10, or 12 as the official detection-gate count until the
  official runtime gate events / Trigger actors are audited.

## Factory logic

- Stage 1-3: no factory.
- 8-12: one factory before the hub.
- 12-10: one factory before the hub and one after the hub.
- 10-7: one factory before the hub and one after the hub.
- Exact entry/exit and reporting triggers are not confirmed. Keep this as a
  configurable state machine; do not hardcode a guessed sequence.

## Status of previous local results

The 1.47 km `1_1 + 2_1` route with two gates is a **proxy**. The reported
100% / 20.6 m/s stage-1 result is only a pipeline smoke test and is **invalid
for official stage 1**. The official route/gate model must be rebuilt from
`route_1_3.yaml` and the official gate events before retraining.

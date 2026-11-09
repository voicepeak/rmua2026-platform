# OL1 stage-3 return probe (2026-11-10)

Status: **component count observed, score 0/31.**  Failure record, not accepted
score evidence.

## Method

- Probe: `foundation/probes/stage3_return.gdb`.
- Native stage-init injection `old=1,3,1 -> new=5,8,3`.
- Teleport at first contact to the route-5 endpoint
  `(146.58, 286.23, 10.57) m` (UE cm `14658.5, 28623.4, 1056.9`).
- Plan: `foundation/config/g2_stage3_return_probe_20261110.json`; fly the
  route-5 return (route index 2) from its endpoint to the hub.

## Result

- `DUMP_BEGIN actor=0x70b3c51ec100 count=31` — the route actor declares **31**
  detection components.
- The dumped component transforms are `nan` / zero quaternion, so this probe did
  not yield usable component roots.
- SEG events were observed for indices **16–30**, but score stayed **0**.
- `STATE old=0 new=12` appeared immediately after the teleport; state 12 likely
  terminates or bypasses the normal scoring queue, but this is not decoded.
- Parsed result: `foundation/reports/g2_stage3_return_seed123_20261110.json`.
  Raw GDB log recovered from the truncated tar:
  `foundation/datasets/g2_stage3_return_seed123_20261110/run/simulator.log`.

## Gaps

- Stage-3 injection directly from stage 1 may not arm the scoring queue; a
  sequential stage-2 -> stage-3 transition is the next method to try.
- Component roots need a valid transform; the `nan` dump must not be used as
  geometry.
- No stage-3 score is claimed.

## Second attempt: 12 m pre-endpoint approach

- Probe: `foundation/probes/stage3_return_v2.gdb`, plan
  `foundation/config/g2_stage3_return_v2_20261110.json`.
- Teleport target: 12 m before the route-5 endpoint
  `(13468.15, 28702.18, 927.05) cm`.
- Teleport occurred, but no SEG event appeared.  The vehicle remained at
  `(134.68, 287.02, -9.34) m` with commands published toward the route start
  `(146.58, 286.23, -10.57) m`; it did not move off the ground.
- Stage 3 remains **31 declared components, score 0**.  No runtime component
  roots or score evidence.  Artifact: `foundation/datasets/g2_stage3_return_v2_seed123_20261110/`
  and `foundation/reports/g2_stage3_return_v2_seed123_20261110.json` (no SEG/DUMP).

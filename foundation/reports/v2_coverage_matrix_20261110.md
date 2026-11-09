# V2.0 coverage matrix (2026-11-10)

Coverage index only. Static candidates do not count as runtime detection components.

> 2026-11-10 correction: OL2–OL12 与 OL1 的赛道几何在刚体变换下同构，不是 12 条
> 不同赛道；见 [v2_map_clone_20261110.md](v2_map_clone_20261110.md)。下表中
> OL2–OL12 的“0 runtime roots”仍表示运行时未审计，不表示静态布局不同。

| scope | detection components | score | official contact | factory |
|---|---|---|---|---|
| OL1 stage1 road1 | 33/33 roots (runtime) | 33/33, indices 0-32 | road rail 939 / port wall 115 / hub wall 234 | N/A |
| OL1 stage1 road3 | 30/30 roots (runtime) | 0/30 score; 30/30 component triggers observed under stage1 (inactive queue) | none | N/A |
| OL1 stage2 | road3 30 roots known; route5 roots UNVERIFIED | 30/30 road3 score (injected stage2, controlled) | hub wall only | N/A |
| OL1 stage3 | 31 declared components; roots unusable (nan) | 0/31 (observed indices 16-30) | UNVERIFIED | N/A |
| OL1 stage4 | UNVERIFIED (attempt: no SEG) | UNVERIFIED (attempt: stuck at approach) | UNVERIFIED | UNVERIFIED (1 before hub) |
| OL1 stage5 | UNVERIFIED | UNVERIFIED | UNVERIFIED | UNVERIFIED (before+after hub) |
| OL1 stage6 | UNVERIFIED | UNVERIFIED | UNVERIFIED | UNVERIFIED (before+after hub) |
| OL2 | 0 runtime roots (12 static candidates; 12/12 runtime transform matches; map redirect observed, no SEG/DUMP) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL3 | 0 runtime roots (12 static candidates; map redirect observed, no SEG/DUMP) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL4 | 0 runtime roots (12 static candidates; map redirect observed, no SEG/DUMP) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL5 | 0 runtime roots (12 static candidates) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL6 | 0 runtime roots (12 static candidates) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL7 | 0 runtime roots (12 static candidates) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL8 | 0 runtime roots (12 static candidates) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL9 | 0 runtime roots (12 static candidates) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL10 | 0 runtime roots (12 static candidates) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL11 | 0 runtime roots (12 static candidates) | UNVERIFIED | UNVERIFIED | UNVERIFIED |
| OL12 | 0 runtime roots (12 static candidates) | UNVERIFIED | UNVERIFIED | UNVERIFIED |

## Evidence

- OL1 detection runtimes: `foundation/maps/OL1/detection_gate_candidates.json` (SHA-256 `fca3743516708f26bb269a081a143ee2c2f0642bf683607f1b1361e34a20b30d`).
- OL1 road1 score: `foundation/reports/g2_score_route_seed123_20261109.json` (SHA-256 `7f4939308df939ffe860d9632e79c6d9bda73828976bd1956a24065130667a59`).
- OL2 gate runtime transform matches: `foundation/maps/OL2/gate_runtime_evidence.md`.

## Gaps

- 12 named *_Gate actors are solid route gates, not detection gates
- OL1 road3 score 30/30 under injected stage2 (controlled); stage1 direct remains 0/30 and component triggers 30/30
- OL1 stage2 score is 30/30 under controlled injection; stage3-6 runtime roots/score and route5 components remain unverified
- OL2-OL12 runtime detection roots and scores not yet audited
- factory entrance actor/trigger not located; 12 static candidate volumes exist, no event evidence
- OL1 stage1 direct road3 score is 0/30; stage2 injection reaches 30/30; this is not official stage progression

Machine-readable: `foundation/reports/v2_coverage_matrix_20261110.json`.

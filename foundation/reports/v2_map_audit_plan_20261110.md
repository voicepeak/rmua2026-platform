# OL2–OL12 runtime map audit plan (2026-11-10)

> 2026-11-10 correction: OL2–OL12 与 OL1 的赛道几何在刚体变换下同构，不是 12 条
> 不同赛道；不要按“每图独立逆向”执行本计划。保留本页仅作为地图重定向/score-dump
> 工具和历史记录。运行时等价性/配置差异检查见
> [v2_map_clone_20261110.md](v2_map_clone_20261110.md)。

## Tooling

- `foundation/probes/map_score_dump_OL2.gdb` … `OL12.gdb`:
  redirect the default Lobby→OL1 load to the target map at `0x593dd30`, then
  dump the native routing object's component array (`+0x240`, count `+0x248`)
  and every `SEG` score event.
- `foundation/probes/map_ol2_teleport.gdb`: the OL2 variant with a diagnostic
  teleport to the route-1 start, because the static OL2 route starts far from
  the origin.
- `foundation/config/g2_ol2_route1_20261110.json` and
  `..._teleport_20261110.json`: OL2 route-1 plans from `data/tables/routes.csv`.
- `foundation/parse_score_dump.py`: converts a dump log into a component/score
  manifest.

## Per-map procedure

1. Start one isolated `rmua/re-probe:20261104` container through the existing
   `run_contact_waypoints_remote.py` runner.
2. Use the target map GDB script; verify `RMUA_MAP_REDIRECT old=OL1 new=...`
   and `DUMP_BEGIN` / `SEG` lines.
3. Record component roots, observed indices and score sequence; hash the log.
4. If two consecutive runs show no map load or no SEG event, record the gap and
   move to the next map.
5. Do not copy OL1 component counts to another map; each map's `RouteConfigs`
   and runtime roots are independent.

## Status

- OL2: plan and teleport probe prepared; runtime run not yet completed.
- OL3–OL12: GDB variants prepared; no runtime run yet.
- Static gate candidates: 12 per map; OL2 has 12/12 historical runtime gate
  transform matches, but that is not a detection component or score.
- No map yet has runtime detection roots, score, official contact or factory
  entry evidence.

## Limitations

- The map redirect is a development intervention; the official launcher only
  exposes `seed`, not a map selector.
- GDB all-stop changes timing and is not a policy or race result.
- A `DUMP` row is a candidate component root, not a trigger aperture.

# OL1 stage-4 return probe (2026-11-10)

Status: **no SEG/DUMP; gap.**

- Probe: `foundation/probes/stage4_return_v2.gdb` (inject `(8,12,4)`, teleport 12 m before the route-8 endpoint).
- Plan: `foundation/config/g2_stage4_return_v2_20261110.json`.
- Observed `TELEPORT new_cm=135872.041,-10274.299,880.040`, then no SEG.
- The reported pose stayed at `(1358.72, -102.74, -8.87) m` while commands were published toward `stage4_000`; the vehicle did not move off the ground.
- Output: `foundation/datasets/g2_stage4_return_v2_seed123_20261110/`.
- Stage 4 runtime component roots/score remain **UNVERIFIED**.

## Second attempt: teleport 15 m above the endpoint

- Probe: `foundation/probes/stage4_return_v3.gdb`, target z=1500 cm.
- Teleport occurred, but no SEG.  The vehicle stayed at
  `(1358.72, -102.74, -15.07) m` while commanding descent to `-8.87 m`; it did
  not move.
- Two stage-4 attempts now show no SEG/DUMP; stage 4 remains UNVERIFIED.

## Ground-lock diagnosis (2026-11-10)

- The teleport reports the component at the target, but the vehicle does not
  move under any velocity command.
- A climb-first probe (`climb` waypoint 21 m above the endpoint, speed 6 m/s)
  still reported position `(1358.72, -102.74, -8.87) m` with desired vertical
  velocity -6 m/s; the command had no effect.
- The `rosservice call /airsim_node/drone_1/takeoff` attempt failed because the
  probe container does not have `airsim_ros/Takeoff` message types
  (`ERROR: Unable to load type [airsim_ros/Takeoff]`).
- Conclusion: the teleported vehicle is frozen/held at the target in this
  configuration; stage 4 needs a different injection/reset path, not another
  waypoint plan.  Stage 4 remains UNVERIFIED.

## Takeoff-service attempts after mounting the ROS workspace

- Mounted `/home/tianbot/RUMA-by-helinjun/beta/rmua_ws` at
  `/workspace/rmua_ws:ro` in the probe container.
- `rosservice info /airsim_node/drone_1/takeoff` then reported
  `Type: airsim_ros/Takeoff` and the correct argument.
- `waitOnLastTask: true` and `false` both timed out after 15 s; the reported
  vehicle position stayed frozen at the teleport target.
- Therefore the service type issue is resolved, but the simulator/vehicle
  physics is frozen after the component-write teleport.  This is not solvable by
  another waypoint plan; it needs a physics-body state write or a different
  reset/injection path.

## 50 m teleport test

- Probe: `foundation/probes/stage4_return_v4.gdb`, target z=5000 cm.
- Teleport occurred; the reported position stayed `(1358.72,-102.74,-50.07) m`
  while commanding descent to the route start.  No motion and no SEG.
- Even a high, clear teleport does not restore velocity control, so the failure
  is not only a local wall/roof overlap.  It is consistent with a component-only
  write that leaves the physics body/state frozen.
- Next requirement: a physics-body state write (AirSim `PawnSimApi` /
  `MultirotorPawnSimApi` pose+kinematics) or a reset path that reinitializes the
  body at the target.  Another waypoint plan will not help.

## Reset-service path attempt

- Added a ROS `/airsim_node/reset` call after the contact teleport and a GDB
  root write to the route-8 endpoint.
- `rosservice call /airsim_node/reset` returned `success: False`.
- The run then showed **stage-1 road1** components (`count=33`, SEG idx 31/32,
  score 1–2), not stage 4; it aborted on the altitude envelope.
- No `RESET_ENTRY` / `RESET_ROOT_WRITE` lines appeared, so the reset callback
  did not execute the intended root write.
- Conclusion: the reset path does not preserve the injected stage and is not a
  usable target teleport.  Stage 4 remains UNVERIFIED.

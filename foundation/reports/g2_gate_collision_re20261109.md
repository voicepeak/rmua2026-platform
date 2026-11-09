# G2 direct-function checks — gate collision profile and meter_report

Date: 2026-11-09. These are static/RE and service-level observations; they do not
make G2 pass.

## 1. Route-gate collision profile (direct function, static)

`ARouteGate2024_1` activation is at `0x2fc13d0` (Ghidra decompile in the
upstream RE dossier, `artifacts/rea/followup_state.c`). The decompiled branches
build a profile name and call the component virtual at `+0x668`, then call
`0x52bb600(component, active, 1)`:

- inactive branch: profile string at `.rodata` `0x820f7c` = **`NoCollision`**;
- active branch: profile string at `.rodata` `0x74213e` = **`BlockAll`**.

The second call manipulates the component collision-enabled flags at offset
`0x14c` (the same region used by Unreal collision enable APIs). The relevant
names are present in the binary string table:
`SetActorEnableCollision`, `SetCollisionEnabled`,
`SetCollisionProfileName`, `SetCollisionProfileNameProperty`.

**Observation:** the 12 named `*_Gate` actors are toggled between no collision
and a solid `BlockAll` collision profile. This is a collision profile toggle,
not a trigger/overlap aperture. The runtime route-gate transforms already match
the candidate table, but this does not establish the detection-gate aperture or
free-space semantics.

Limitations:

- The call target `0x52bb600` is described by its flag manipulation, not an
  independently decoded symbol name.
- The profile strings are observed in the binary; the active/inactive mapping is
  from the decompiled branch order.
- The 12 named gates are not established to be all detection gates.

## 2. `meter_report` service probe (inconclusive)

A fresh seed-123 headless process was started. Using the generated
`airsim_ros/MeterReport` client, the service returned `success=true` for all
four requests:

| index | value | success |
|---:|---:|---|
| 0 | 0 | true |
| 1 | 0 | true |
| 0 | 25 | true |
| 1 | 25 | true |

No `OnMeterReport`, `FactoryA`, `FactoryB`, `EnterPort` or `LeftPort` line
appeared in the simulator log before the container was stopped. The service
acceptance response is therefore not a factory-entry or value-validation result.
The calls were made from the ground start without a race-state/position trigger;
event queueing or state gating remains unresolved.

Dataset: `foundation/datasets/g2_meter_report_probe_seed123_20261109/` (pool
manifest and simulator log; no business-value acceptance).

## 3. Status

- Gate collision profile: **resolved to NoCollision/BlockAll toggle** (static).
- Detection-gate aperture/clearance: still **unverified**.
- Factory entry via `meter_report`: still **unverified**; service success is not
  evidence.
- G2 remains **PARTIAL**.

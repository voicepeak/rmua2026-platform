# G2 official contact probes — 2026-11-09

Status: **PARTIAL**. These runs add official contact evidence for one road
segment and one building wall. They do not validate a detection-gate aperture
or a factory entrance, so G2 remains PARTIAL.

## Method

- Disposable headless-pool container with the official simulator binary
  SHA-256 `5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800`
  and settings SHA-256
  `0d72886f92b091cbc5ba2ae58d25b4b4bc589045d4e725f483603efdc26c6c17`.
- Read-only GDB breakpoint at `0x2630780`; the official collision callback
  supplies the drone/source component transforms and the candidate hit-result
  bytes. GDB all-stop changes timing.
- `foundation/probes/contact_waypoints.py` flew bounded velocity waypoints with
  `debug/pose_gt` feedback. This is a diagnostic pilot, not a policy or race
  result. Body commands use a positive-up `VelCmd.vz`, while `pose_gt` z is
  NED-down; this sign was verified with a separate short process.
- Every run used a new container, a motion/time envelope, and immediate
  container stop. Failed iterations are retained with the datasets.

## Results

| Target | Dataset | Grouped callbacks | Candidate | Source transform error | Candidate hit-point distance | Second nearest |
|---|---|---:|---|---:|---:|---:|
| Road side rail | `g2_contact_road_seed123_20261109` | 939 | `BP_Segment_Rot_C_242` (`Segment`) | 0.00022 cm | 1.7e-7 – 3.6e-4 cm | 1040.6 cm |
| Port interior wall | `g2_contact_portwall_seed123_20261109` | 115 | `1_1_Port/Building_Port` | 1.5e-5 cm | 3.5e-6 – 2.1e-4 cm | 102.7 cm |
| Central junction (route run) | `g2_contact_gate1_seed123_20261109_v2` | 234 | `BP_Building_Junction_7/...` | 4.4e-6 cm | 2.6e-5 – 0.022 cm | 4.4e-6 cm (duplicate) |

The startup sprite contact (`1_1_Port/R_logo_Sprite1`) reappeared in each run
(2, 2 and 1 callbacks respectively) with the same transform and hit-point
residual as the earlier `contact_surface_seed123/456` evidence.

Machine-readable summaries:

- `g2_contact_road_seed123_20261109.json`
- `g2_contact_portwall_seed123_20261109.json`
- `g2_contact_gate1_seed123_20261109_v2.json`

The road run aborted on its total-duration envelope while the drone was held
against the rail; the port-wall run completed. The callbacks during sustained
contact are repeated events, not independent contacts.

### Route and gate attempt

The `gate1_v2` pilot followed the candidate route from the start to about
`(589.5, -98.4, 134.7)` m and then reached about
`(617.0, -81.7, 138.7)` m, where it was stopped by the central junction
building. That is about 9.6 m from the `1_1_Gate` candidate centre. No contact
with a gate candidate was observed, so this run does not establish a gate
aperture, free space, or a trigger volume. The duplicate nearest/second
candidate transforms also mean the junction component attribution is ambiguous.

## Retained failed iterations

- `g2_contact_near_seed123_20261109_zsign_failure`: NED z command sign error;
  the drone stayed on the ground.
- `g2_contact_near_seed123_20261109_v2_roof_failure`: straight climb inside the
  port stopped at the port roof (~4.4 m).
- `g2_contact_near_seed123_20261109_v3_no_breakpoint_failure`: the pool used an
  external `crash_capture.gdb`, so no contact callbacks were logged.
- `g2_contact_wall_seed123_20261109_failed_climb`: 20 m climb over the road was
  blocked below the target altitude.
- `g2_contact_wall_seed123_20261109_v2_failed_crash`: exterior approach crashed
  before reaching the chosen wall.
- `g2_contact_gate1_seed123_20261109_failed_port_structures`: first route exit
  overshot into the port upper structure.

## Remaining G2 work

1. Identify and probe a detection-gate aperture, rather than only the 12 named
   hub/route gate actors. The 12 gate transforms are runtime-matched, but their
   collision or trigger semantics are still unresolved.
2. Locate a factory entrance and obtain an official contact or clearance
   observation there.
3. Repeat the contact evidence with an independent control method or without
   the all-stop debugger if a low-perturbation channel becomes available.
4. Do not treat the candidate hit-point agreement as official free-space or
   complete collision coverage; the hit-result field at offset `0x18` is not
   independently typed.

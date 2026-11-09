# Vehicle and line training proxies

`foundation.obstacles` adds deterministic seeded generation and continuous
collision tests for a spherical drone: segment-to-segment capsule distance for
lines and a moving, fixed-yaw oriented box for vehicles. The vehicle box is
expanded by the drone radius, a conservative approximation at edges/corners;
rotation within a step is not modeled. Tests cover high-speed thin-line crossing,
moving vehicle relative motion, step splitting, deterministic generation and
launch clearance.

The user must supply the bounds, dimensions, line radius, speed and exclusion
distance. The generator checks the full line capsule and the initial oriented
vehicle box against the bounds and supports caller-specified line directions.
An optional feasibility callback can reject candidates; without an independently
verified road/geometry predicate, feasibility is still unknown. Moving vehicles
can leave the initial generation bounds, and mutually inconsistent obstacle
placements are not automatically ruled out. The reverse dossier reports
thin-line local convex bounds of about 0.052 m square by 7 m, and a vehicle
mesh with a Blueprint scale of 0.5, but instance transforms, collision channel
and vehicle path/velocity distribution remain unverified. Therefore those
numbers are not silently treated as world dimensions or defaults. This is a
training approximation under development, not evidence of official contact
behavior or completed C-level acceptance.

`VehiclePath` supports explicitly supplied piecewise linear NED waypoints,
time intervals, fixed yaw per segment, repeated waypoints for stops, and
relative swept collision split at waypoint times. The vehicle stays at its
last waypoint after the path ends. During an instantaneous yaw change a
circumscribed sphere conservatively bounds all intermediate orientations; it
is not a physical turn-rate model and may produce false positives. The path
does not imply road feasibility. `PhysicsSim` can consume these paths alongside
constant-velocity vehicles. Tests cover bounds, configurable directions,
feasibility callback, stop/turn, path boundaries and simulator integration.
The existing vehicle approximation expands an oriented box by the drone radius
and remains conservative at edges; no official collision parity is implied.

## Candidate-map vehicle sphere cover (2026-11-09)

`CandidateMapObstacleFilter` no longer uses only one circumscribed sphere per
vehicle. The vehicle box is partitioned into N×N×N sub-boxes, each covered by
its own circumscribed sphere; the default is N=2 and callers may increase it.
This union still covers the complete box and remains conservative, but it
removes much of the single-sphere margin that rejected vehicles near a road.
Instantaneous yaw changes still use the full circumscribed sphere at the
waypoint because intermediate orientations are unspecified.

The reproducible audit `foundation.audit_vehicle_map_clearance` samples 64
top-facing OL1 road triangles at four vertical gaps with an explicitly chosen
probe vehicle half-extent of (0.8, 0.4, 0.3) m. At gaps 0/0.1/0.3/0.5 m, the
single-sphere filter accepted 0/0/0/0 samples; N=2 accepted 0/0/1/45; N=4
accepted 0/0/45/46; N=10 accepted 0/29/45/46. The report and source hashes are
in `v2_vehicle_map_clearance_audit.json`. Exact road contact still rejects, the
probe dimensions are not an official vehicle asset, and accepted routes are
not proof of official passability.

`generate_vehicle_paths` assigns reproducible speeds and intermediate stops to
caller-supplied horizontal routes. It conservatively checks the complete route
against scene bounds and launch exclusion using the vehicle's circumscribed
sphere, and requires a caller-provided whole-path feasibility predicate. It
rejects infeasible inputs instead of substituting an unchecked route. Whether
that predicate proves clearance from real road geometry, and whether different
vehicles avoid each other, remain the caller's responsibility; a partially
reconstructed map is insufficient evidence of unobstructed space.

## Asset-derived local proxy bounds

`obstacle_asset_geometry.py` applies each convex element transform and its
Blueprint mesh-component transform to the extracted obstacle convex vertices.
The hash-traced result is `obstacle_geometry_candidates.json`. In the component
parent frame, the thin and thick line convexes fit conservative Z-axis capsules
with radii 0.027948 m and 0.055347 m respectively and approximately 7.002 m
between endpoint centers. These radii bound the vertices, including their
corner offsets; the half-widths of 0.026/0.051 m alone would not enclose every
vertex. The hover mesh's transformed axis-aligned box has half-extents
approximately (2.214, 0.879, 0.738) m and a nonzero center offset, retained in
the asset record. These are conservative local proxies, not measured world
collision dimensions.

`obstacle_asset_instances` can transform these bounds into NED proxy `Line`
and `Vehicle` objects when the caller explicitly supplies parent transform,
scale and UE/NED origin. Unit tests verify every transformed source vertex is
contained after a rotated, scaled instance transform. This still assumes the
provided parent transform is correct and does not identify actor instance
scale, runtime profile, active collision shape, or official contact response.

## Instrumented line-instance submissions

`analyze_line_instances.py` independently parses the saved 48-byte submission
records and their paired return indices from stage-injected OL1 experiments.
For stages 4, 5 and 6, with seeds 123, 456 and 2026 (one run each), all 8,775
observed submissions had nonnegative return indices. Per seed, stage 4 had
272 thick and 101 thin, stage 5 had 594 thick and 524 thin, and stage 6 had
1,434 thin. The candidate FTransform layout yields scale (1, 1, 1.2) in all
nine runs. If that scale is applied to the extracted line bounds, the capsule
endpoint separation is approximately 8.403 m; this is not a measured world-space
length. Under the candidate quaternion interpretation, axis tilt ranges from
approximately 0.83 to 60.19 degrees from vertical across those runs, so an
always-vertical training generator would not cover the recorded candidates.

Per-instance raw bytes, component selection, candidate transforms, return
indices, source hashes and limitations are in `line_submissions_stage*.json`.
`line_from_submission` can construct conservative NED proxies when the caller
explicitly chooses world or component-local interpretation and supplies the
required origin and, for local coordinates, the component transform. It
rejects failed instance returns and nonuniform component scale. Stage injection,
unverified transform coordinate space and collision-channel ambiguity prevent
these records from proving official obstacle geometry or contact behavior.

## Controlled fixture surface correspondence

In the existing stage-4 injected fixture matrices, a known identity-rotation,
unit-scale line was placed at UE (0, 0, 350) cm. Across three seeds and two
repetitions per line type, `analyze_line_fixture_surface.py` matches each hit's
source component and candidate instance index to the injected fixture, then
compares the candidate three-float hit vector at offset `0x18` with the
transformed extracted convex triangles. The 123 thin-line hits have a maximum
surface distance of 0.00739 cm; the 251 thick-line hits have a maximum distance
of 0.00872 cm. Individual values and log, manifest and asset hashes are in
`line_fixture_surface_thin.json` and `line_fixture_surface_thick.json`.

All candidate hit vectors lie on or very near the line's bottom end cap in
this fixture; these measurements do not cover lateral surfaces or arbitrary
instance rotations. The `0x18` vector and `0x58` index meanings are inferred
from the controlled match, not independently typed from engine metadata. The
result supports the extracted end-cap geometry and fixture transform under
these injected conditions; it does not establish the complete drone shape,
contact force, natural-course collision channels or general FTransform space.

## V2 protected route and synthetic sweep checks

`generate_constrained` adds explicit protected route tubes and a finite motion
horizon to the existing seeded generator. It rejects line capsules that touch a
protected tube. For moving vehicles it conservatively bounds the entire
constant-velocity trajectory by a circumscribed sphere for route and launch
clearance, and keeps the oriented box within configured bounds over the whole
horizon. A caller can additionally supply a trusted-map feasibility predicate;
the route itself and the predicate still need trustworthy map evidence. A
route tube alone does not establish official passability, and vehicles may
still overlap one another. The checks do not cover motion after the horizon.

`python -m foundation.collision_sweep_benchmark` runs 90 analytical synthetic
crossing/near-miss cases at relative speeds 10–150 m/s and timesteps
0.001–0.1 s. The saved [report](v2_collision_sweep_benchmark.json) records zero
misses and zero false hits in these axis-aligned examples, including moving
vehicles. These are controlled calculations, not a measured probability of
misses in real 3D courses; turning, mesh shapes, contact forces and official
collision channels remain outside this benchmark.

# OL1 road collision extraction — partial, not world-aligned

The 12.0.0.5 pak used here hashes to
`2407662b224ed28c64412402af044da85aced07d78ce6fc0948e6be11ea2a0c2`.
`repak unpack` extracted `Resources/Buildings/Road`; UAssetAPI 1.1.0 parsed
the packaged UE 4.27 assets. The original local package bytes remain under
`foundation/raw_assets/` and are intentionally ignored by Git.

| File | SHA-256 |
| --- | --- |
| Segment.uasset | 049a81eb79b37edd2e38f66e0b0b65d6db50d2d6fb2d56fb9c2884c6925960a5 |
| Segment.uexp | 3d9ff8c1ccf08532232e490d33b534cd41265d91836c6041e7a604c77cbd8bf3 |
| Segment_Half.uasset | 3757f4095a27a5d62c45b0f9eedf84c68a42633803bf9f85c8464c13a32a3862 |
| Segment_Half.uexp | ab7640971befe4d159c2ed72d7bc36d7782c03f6d6d05a610af5a3ad1a76d5f3 |

Both `BodySetup_0` exports specify `CTF_UseComplexAsSimple`. Their cooked
`Extras` contain one little-endian PhysX MESH v14 prefix each. Following the
public [PhysX 3.4 writer](https://github.com/NVIDIAGameWorks/PhysX-3.4/blob/master/PhysX_3.4/Source/PhysXCooking/src/mesh/TriangleMeshBuilder.cpp),
`extract_physx_mesh.py` extracted vertex and triangle arrays: `Segment` has
416 vertices/800 triangles, and `Segment_Half` has 248 vertices/476 triangles.
Indices are in range and faces have distinct indices; the resulting local
bounds nearly match the separately exported convex bounds. Source hashes and
remaining uninterpreted bytes are in the adjacent `.json` manifests. This is
strong evidence for the local cooked geometry, not a validated runtime contact.

`road_instances.py` records 471 `BP_Segment_Rot_C` and 12 `BP_Segment_Half_C`
actors in the OL1 UAssetAPI dump and preserves each component's uncomposed
attachment chain. `BP_Segment_Rot`'s template contains an additional relative
translation and rotation; some actor roots attach to a parent. Consequently,
the actor locations alone cannot be used as mesh world coordinates. Blueprint
defaults, UE rotations, absolute flags, collision channels and runtime
placement must be resolved and checked against representative official
contacts before these arrays can be called an OL1 collision map. Buildings,
factory interiors and other road types remain outside this extraction.

## Candidate placement and static sanity checks

`road_scene.py` composes the 483 saved chains using Blueprint mesh-component
defaults and a candidate UE rotator/FTransform convention; it can query finite
segments against the two local triangle meshes in UE centimetres. The
placements are explicitly status U, and exclude the Blueprint's four `Cube`
components and other map geometry. No AirSim-NED origin has been applied.

Under the assumed spline and placement conventions, 61 exported spline knots
have a median nearest road-vertex distance of 4.08 m (p95 5.49 m). Only 1 of
61 vertical 20 m zero-radius segments through knots intersects these two road
meshes. A separate local-grid probe finds vertical hits on some, but not all,
parts of the Segment mesh. These observations may reflect gaps in the chosen
geometry, spline/placement assumptions, or the shape itself; they do not prove
the placement correct or free space. `road_spline_sanity.json` retains each
query. Verify actual component transforms and representative runtime contacts.

Two further factory local meshes were extracted using the same decoder:
`Building_Factory_Factory` contains 253 vertices/470 triangles and explicitly
uses complex-as-simple; `Building_Factory_Ceiling` contains 8 vertices/12
triangles but has no serialized collision trace flag, so its effective flag is
unknown. These remain unplaced and their runtime collision behavior unverified.

## Building asset inventory

The OL1 map imports 16 building Blueprints. Their parsed imports identify 37
mesh packages, all extracted and parsed with readable local cooked triangle
prefixes. The junction meshes and Engine cubes use different BodySetup names,
which are now resolved by class. The serialized simple shapes include 35
convex elements and one box; the AirSim plane has no serialized simple shape
in this inspection. See
`OL1_blueprint_mesh_refs.json`, `mesh_collision/inventory.json` and
`OL1_simple_convex_candidates.json`. A decoded triangle prefix does not prove
it is the active collision representation: only some assets have an explicit
complex-as-simple flag, and omitted flags remain unknown.

The map inventory associates 8,682 static-mesh components across 485 building
actors with Blueprint mesh imports. Candidate transforms compose template
defaults and serialized parent chains; all 8,682 component placements reference
decoded triangle prefixes. These counts are static
references, not verified runtime collision coverage; the collision trace mode,
construction overrides, transforms and channels still need validation.

`map_scene.CandidateMap` combines the road candidates with 25 building
components whose source mesh explicitly uses complex-as-simple, for 508
queryable triangle instances. It excludes 8,657 building components with
unknown mode as well as road auxiliary cubes and other map geometry. Its
zero-radius finite-ray query supports UE coordinates or an explicitly supplied,
axis-aligned NED origin; the origin itself is not verified. The manifest and
synthetic self-hit check are in `partial_map_manifest.json`. This check exercises
the query against its own input; it is not independent validation of official
placement or contact, and a miss does not establish free space.

## Exploratory LiDAR correspondence

`lidar_map_probe.py` compared a deterministic sample of 128 returns from each
of the 23 historical `hub_lidar_probe_20261104` scans against this partial map,
using the separately observed start-area candidate origin (0.0078, 0.001, -7)
UE cm and the historical observer's assumed sensor offset. Of 2,944 rays,
2,894 intersected the candidate mesh within the measured return range plus 2 m;
2,876 had a nearest candidate intersection within 1 m of the measured range.
Among the 2,894 hits only, median/p95 absolute range difference was 0.060/0.398 m;
the maximum was 10.37 m. Most hits were on one candidate junction actor, and
the scans cover only the approach/hub, not the complete route.

GPS/IMU were latest samples rather than interpolated to the scan; the largest
GPS/IMU stamp offsets from the cloud in these scans were about 99/9 ms. LiDAR
visible surfaces, physics collision geometry, unverified origin, unknown object
channels and omitted building components are distinct sources of disagreement.
The result supports further investigation of the local placement but neither
establishes 1:1 collision geometry nor explains missing/outlier rays. Individual
rays, random seeds and source hashes are in `lidar_candidate_all23.json`.

## Serialized collision-profile evidence

`collision_evidence.py` retains the mesh-default, Blueprint-template and map
instance collision fields separately for all 8,682 inventoried building mesh
components. Of these, 8,646 templates explicitly name the `BlockAll` profile;
the remaining 36 light components do not serialize a profile in the inspected
template. No profile, collision-enabled or object-type field was found in the
inspected map-instance data; this does not rule out overrides elsewhere.

The extracted packaged engine and project configs both contain an explicit
`+Profiles` entry for `BlockAll` with `QueryAndPhysics` and `WorldStatic`; the
project also defines a custom trace channel whose default response is ignore.
`collision_config_evidence.json` records the exact config lines and hashes. No
`DefaultShapeComplexity` setting appears in the three inspected config files,
but absence is not proof of an engine default. These findings support further
inspection, not promotion of the 8,657 unknown components into the validated
map: effective shape mode, drone/trace channel, config merge, construction and
runtime overrides still require direct verification.

## Opt-in serialized simple-shape candidates

`simple_collision_mesh.py` turns serialized convex elements and box elements
into candidate local triangles, applying each element's serialized transform.
For elements with vertices but no index buffer, it reconstructs a convex hull;
that reconstruction is not a decoded cooked mesh. The map loader exposes this
only through `include_unverified_simple=True`; the default map is unchanged.
Explicit complex-as-simple assets continue to use their complex candidates.

On the current asset inventory this opt-in adds 8,645 building component
instances, for 9,153 total candidate instances including roads and explicitly
complex buildings; 12 building components remain excluded, as do other map
objects. These are serialized shape candidates, not established effective
collision: mode defaults, component transforms, profiles, channels, and
runtime overrides remain unverified. A ray miss still does not prove free
space, and zero-radius rays do not implement drone contact.

The same 2,944 historical hub LiDAR rays were reevaluated with the extra
shapes. Their nearest hits and errors were unchanged: 2,894 hits, 2,876 within
1 m and hit-only median/p95 absolute range error 0.060/0.398 m. Thus these
hub scans do not validate the newly added simple shapes; the data and hashes
are in `lidar_simple_candidates_all23.json`. Additional observations near
the newly included buildings and independent collision contacts are required.

## Candidate swept-sphere surface checks

`CandidateMap.sweep_sphere_ue` and `sweep_sphere_ned` test a finite-radius
sphere moving continuously against candidate triangle surfaces, including
face, edge and endpoint contacts. `PhysicsSim` accepts this map only when an
explicit map origin is provided, and labels every resulting contact
`static_candidate`. Without an explicitly supplied map, behavior remains
unchanged. A synthetic sweep through an extracted `Segment_Half` candidate
finds that actor, but is a self-test of the map against its own geometry, not
an official comparison.

This is a sphere approximation with no exact time of impact, contact response,
initial inside-solid detection, or chronological ordering against simultaneous
line and vehicle hits. The terminal state is the end of the substep. Excluded
objects, unverified transforms, channel settings, and the official drone's
actual collision geometry prevent this feature from closing G01 or justifying
free-space guarantees. Unit tests exercise face, edge, stationary, degenerate,
and high-speed simulator cases.

## Runtime contact-origin investigation

A fresh seed-123 process with no stage-state or fixture injection used the
bounded collective waveform and a read-only `0x2630780` callback breakpoint.
Four callbacks in the saved log identify the same source actor/component
pointer. The source component fields at offsets `0x1d0` and `0x1c0` read as
approximately (-29.992, 0.001, -40.000) cm and quaternion (0.5, 0.5, 0.5,
0.5). Among the current 9,153 candidate instances the nearest saved component
transform is `1_1_Port/Building_Port`, 237.95 cm away and 90 degrees different.
The transform comparison, raw log and hashes are in
`contact_transform_seed123.json` and `datasets/contact_probe_seed123_20261107`.

This is not an identified match: the source could be an omitted actor or
component, or the candidate transform could be wrong, and these memory offsets
have not been independently confirmed for every component type. The finding
is a concrete gap in object attribution, not proof that the nearest component
is misplaced. The process was stopped after its bounded player completed;
all-stop GDB affects timing.

Separately, a historical stage-injected/forced-vehicle pilot recorded 20
`HIT_SOURCE` events at seven distinct drone component origins. Candidate
surfaces were 2.7–14.1 cm away by a 0.25 cm resolution search in that sample;
the event's source pointer is not mapped to an asset, and a drone component
origin is not the physical contact point. Dynamic fixture contact is possible.
Consequently this proximity is not G01 validation; see
`contact_map_proximity_pilot.json` for each event and source hashes.

## Sprite contact candidate

In a separate fresh seed-456 process, the contact source actor's runtime root
was (-249.992188, 0.000977, -50) cm, matching the serialized root of
`1_1_Port`. Its source component transform matches the `1_1_Port` blueprint's
`R_logo_Sprite1` candidate within 0.000002 cm and 0.000004 degrees. This
explains why the earlier search, which contained only road and static-mesh
components, had no nearby transform. The root and transform match strongly
support this attribution but do not independently decode runtime object names;
the run used all-stop GDB and four callbacks from one component.

The packaged `Rlogo_Sprite` BodySetup explicitly specifies simple-as-complex
and contains one local box centered at (1, 0, 2) cm with full size
(854, 10, 846) cm. The blueprint component scale is (0.2, 0.2, 0.2). Five
other building sprite assets each contain a simple-as-complex box. Their parsed
data and original package hashes are recorded in
`sprite_collision_candidates.json`. The inventory now also composes the 12
standalone sprite actors directly from their serialized component transforms;
their 12 packaged BodySetups each specify simple-as-complex and one box. The
result includes 120 sprite candidates, with no unresolved sprite placements
in this inventory. Explicit `include_unverified_sprites=True` adds these
boxes; default map queries do not change. The candidate sprite box lies within
2 cm of the captured drone component origin in the seed-456 sample, but that
origin is not the hit point, and runtime channels and full shape alignment are
not established. Evidence: `contact_sprite_match_seed456.json` and
`datasets/contact_source_seed456_20261107`. G01 remains open.

With both simple-building and sprite candidates enabled, 9,273 instances are
queryable. This count is a static asset inventory, not verified collision
coverage. Standalone sprites' effective runtime profile and channel remain
unknown, as do unlisted non-sprite objects and construction effects.

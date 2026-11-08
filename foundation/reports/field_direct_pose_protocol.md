# Predicate-synchronous UE component pose protocol

Before collecting data, repeat the raised connector route in a fresh
12.0.0.5 seed-123 process with 145 s controller limit and 6 m/s diagnostic
VEL. Use read-only breakpoints: the existing predicate transition probe and the
previously explored actor-root access at `0x26302df`. Cache its actor/component
pointers in GDB convenience variables only. At each predicate transition,
read the component's candidate world position at `+0x1d0` directly, without
writing any simulator memory. The scripts are copied to a private temporary
directory; the existing repository and simulator binaries are not modified.

Compare each readable UE position to the existing road/port/factory candidate
union without a GPS time pairing or coordinate origin, retain every transition
and report mismatches. Preserve the concurrent GPS samples as a separate
cross-check, not a substitute for unavailable component reads. No geometry or
threshold tuning is permitted before reporting. The pointer's identity,
component transform update cadence, overlap volume and GDB timing remain
unverified, so this is not automatic F01 acceptance.

## First result and fixed exploratory repeat

The first process produced 14 predicate transitions with a readable component
position at each transition. Seven flag-one samples were inside the candidate
union, four flag-zero samples were outside, and three flag-zero samples were
inside by approximately 3.09, 3.14 and 3.50 cm. Ten samples had an accepted
GPS arrival within 0.1 s; GPS-to-component discrepancies ranged up to 0.34 m.
The same actor pointer was observed for all transitions. The dedicated
container was removed. Raw data, probe hashes and per-event comparisons are in
`field_direct_pose_comparison.json` and `datasets/field_direct_pose_seed123_20261108`.

Before inspecting the next process, freeze an exploratory 5 cm inward margin
for the union and repeat the same route with a 5.5 m/s VEL command and 160 s
limit. Record comparisons both with zero margin and this preselected margin,
including unavailable reads and all transitions. The 5 cm value was chosen
after observing the first run and is a hypothesis, not an identified drone
radius or an official rule. The repeat uses the same map, seed and route and
will not independently verify the pointer type or other field branches.

## Independent repeat result

The 5.5 m/s process reached the 160 s controller timeout (exit 124); its owned
container was confirmed removed. All 14 predicate transitions had readable
component positions. With no margin the union agreed on 12/14: two flag-zero
positions were just inside a candidate box. With the previously selected 5 cm
inward margin the union agreed on 14/14. The detailed samples, script hashes and
source hashes are in `field_direct_pose_repeat_comparison.json` and
`datasets/field_direct_pose_repeat_seed123_20261108`.

These are transition observations from the same route and seed, with all-stop
debugging; they do not cover continuous positions or independently identify the
cached component and official overlap primitive. The margin was chosen from
the first run, and the repeat is a limited test of that choice, not proof that
5 cm is a universal physical parameter. Neither the default simulator nor F01
acceptance has been changed on this basis.

## Cross-seed protocol

Before collecting the next data, use the same raised route, 5.5 m/s command,
160 s limit, read-only probes, unchanged box geometry and the already selected
5 cm comparison margin, but set seed 456 in a private copy of the runner.
Report every observed transition and both zero-margin and 5 cm classifications.
The simulator remains in a dedicated identity-checked container. This changes
the seed only and remains a narrow same-route test, not F01 acceptance.

The seed-456 process reached its 160 s limit and the owned container was
confirmed removed. All 16 transition poses were readable. The zero-margin
union matched 9/16 transitions: seven observed-zero samples were inside. The
preselected 5 cm margin matched 14/16: two observed-one samples were only
4.19 and 4.46 cm inside and therefore classified outside by that margin.
Detailed results and hashes are in `field_direct_pose_seed456_comparison.json`.
Thus the earlier 14/14 same-seed repeat does not generalize to this seed;
5 cm must remain an exploratory threshold and cannot be promoted as the
official trigger rule. Component identity, collision footprint, update timing
and other branches remain unresolved.

## Third-seed sensitivity protocol

After inspecting all three preceding direct-pose traces, an exploratory
inward margin of 3.8 cm separates their observed transition samples: the
deepest observed zero is approximately 3.59 cm inside, while the shallowest
observed one is approximately 4.19 cm inside. This threshold was selected
after observing those data, so it is not a physical measurement or acceptance
threshold. Before inspecting another run, freeze both 0 cm and 3.8 cm
comparisons and run the same raised route at 5.5 m/s with seed 2026 and a
160-second limit. Report every transition and any missing pose. The new seed
is a held-out check of this threshold, but route, geometry, timing assumptions
and all-stop probes remain shared; even a match cannot establish F01.

The seed-2026 process reached its 160-second limit and its container was
confirmed removed. All 16 transition positions were readable. The zero-margin
union matched 13/16, whereas the preselected 3.8 cm margin matched 16/16.
The per-event measurements and hashes are in
`field_direct_pose_seed2026_comparison.json`. This supports further testing of
the fitted margin on other routes and contact geometries; it does not establish
the official collision footprint, overlap timing, other predicate branches or
general force-field equivalence. No simulator defaults were changed.

## Periodic sample protocol

Before this run, retain the previously fixed zero and 3.8 cm margins and
capture both all predicate transitions and additional samples no more often
than 5 Hz from the same read-only predicate/pose probes. Use fresh seed 98765,
the same raised route, 5.5 m/s and a 160 s controller limit. Report transition
and non-transition samples separately, preserving missing component reads and
the raw stream. No margin adjustment is allowed after observing this run.
These samples are correlated in time and use the same route; neither their raw
count nor a perfect result would establish coverage of all natural zones.

The seed-98765 process reached its 160 s limit and its container was confirmed
removed. The probe recorded 637 readable observations: 16 transitions and 621
periodic samples. The zero-margin comparison matched 632/637; the fixed 3.8 cm
margin matched all 637, including all 16 transitions. Per-sample records and
hashes are in `field_periodic_seed98765_comparison.json` and the associated
dataset. These samples are strongly correlated and cover the same one route;
the cached component identity, update timing, other regions, and exact
collision geometry remain unverified. This evidence does not close F01 or
justify promoting the margin to an official parameter.

## Different-height route protocol

Before the next observation, use a separately generated candidate connector
with a 6 m lift rather than the prior 8 m, seed 24680, 5.5 m/s, 160 s limit,
and the same read-only periodic probe. Retain the pre-existing 0 and 3.8 cm
comparisons, all transition and periodic observations, missing poses and any
failed or early-terminated trial. Route generation inputs and hashes are
preserved beside the route. Because the straight connector and clearance are
not independently verified, LiDAR safeguards remain enabled. This varies
height on a similar route; it is not full course or F01 acceptance.

## Direct candidate sphere protocol

Before the next run, freeze the Blueprint-derived 2 cm sphere-overlap
hypothesis. In a fresh seed-86420 process, use the existing 6 m-lift route,
5.5 m/s and 160 s limit. In addition to existing read-only periodic predicate
and drone-root observations, read the actor's candidate `+0x348` component
pointer, verify its UObject outer pointer equals the actor, and read its world
transform at each logged predicate observation. Compare its directly observed
center to candidate road/port/factory boxes using 2 cm as the radius; record
all missing pointers, mismatches and transition/periodic samples separately.
No simulated memory is changed. The component type, radius and relationship to
the overlap event remain hypotheses until independently verified. This is an
independent process/seed on a related route, not complete F01 validation.

The seed-86420 process reached its 160 s limit and the owned container was
confirmed removed. The candidate `+0x348` component passed the actor-outer
check and its world position was readable in all 641 observations. Its
distance from the root remained 5.99935–6.00066 cm, supporting the expected
attachment during this flight. With the predeclared 2 cm radius, the candidate
box-overlap calculation matched all 16 transition observations and all 625
periodic observations. Full results and hashes are in
`field_sphere_direct_seed86420_comparison.json`.

This removes the earlier need to infer the sphere center from asynchronous ROS
orientation for this run. It still does not independently identify the
component's class or runtime radius, prove it generated the overlap events,
or validate other routes and predicate branches. Periodic samples are strongly
correlated; the result does not close F01 or establish an official rule.

The seed-24680 process reached its 160 s limit and the owned container was
confirmed removed. It recorded 647 readable observations, including 16
transitions. Zero-margin point containment matched 642/647; the preselected
3.8 cm margin matched all 647, including all transitions. During observed
zero-flag samples the candidate UE component height ranged 134.46–136.16 m,
compared with 133.07–135.25 m in the previous 8 m-lift periodic run. Full
records and hashes are in `field_lift6_seed24680_comparison.json`. The two
routes still share most geometry, samples are temporally correlated, and the
component identity, collision shape and other field branches are unverified;
this does not close F01 or establish an official 3.8 cm parameter.

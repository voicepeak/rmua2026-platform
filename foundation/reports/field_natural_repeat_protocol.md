# Natural predicate repeat protocol

Before inspecting the new data, repeat the historical raised connector route
in a fresh official 12.0.0.5 process with read-only predicate tracing, a 6 m/s
VEL diagnostic and a 145-second controller limit. Use the same recorded route,
candidate UE/NED origin (0.0078, 0.001, -7) cm and nearest accepted GPS arrival
within 0.1 s. Compare each observed predicate transition to point containment
in the existing candidate road/port/factory union; retain all late or missing
pairs and all mismatches. No geometry, tolerance or flag scaling is fitted to
the new data. The dedicated simulator must be stopped and its identity-checked
container removed, including on timeout or failure.

This repeat uses the same map, seed and nominal route as the earlier run, with
different execution timing. It can test only the observed transitions, not
continuous occupancy, all branches, exact spatial overlap or F01 acceptance.

## Result

The process reached its 145-second controller timeout (exit 124) and its
dedicated container was confirmed removed. Fourteen predicate transitions
were logged. Eleven had a nearest accepted GPS callback; one of those was
outside the predeclared 0.1 s arrival-gap limit, leaving ten for the fixed
comparison. The raw summary retains all 14, including the three without an
accepted nearest GPS sample.

Of the ten selected transitions, the candidate union agreed with the observed
predicate in five: two zero/outside and three one/inside. Five were observed
zero while the candidate point was inside the union, at signed road distances
from approximately -0.01 to -0.26 m. Three of those five GPS arrivals preceded
the event by about 74–76 ms; the other two were about 15 ms before and 36 ms
after. The finite drone shape, actual event/position time difference and other
predicate branches have not been isolated. The mismatches show that this
point-and-box union is insufficient as an exact instantaneous predictor under
the tested pairing; they do not by themselves identify the cause.

Full per-event observations and provenance are in
`field_natural_repeat_region_comparison.json` and raw records are in
`datasets/field_natural_repeat_seed123_20261108`. The prior seven-transition
pilot's 6/7 agreement must not be generalized to this repeat. The candidate
spatial field remains opt-in and unverified; F01 remains open.

As a posthoc timing sensitivity check, linear interpolation between accepted
GPS arrival samples with a maximum 0.2 s bracket gap classified 9 of the 14
events; 8 of those 9 matched the union and one did not. Five events lacked
close accepted bracketing samples and remain unclassified, including events
that contributed to the nearest-sample mismatches. The method and gap were
chosen after seeing the original results, and arrival time still differs from
physical measurement time. It therefore does not supersede the predeclared
5/10 result or establish improved accuracy. All 14 eligibility decisions are
in `field_natural_repeat_interpolated_exploratory.json`.

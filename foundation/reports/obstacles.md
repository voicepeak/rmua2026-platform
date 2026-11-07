# Vehicle and line training proxies

`foundation.obstacles` adds deterministic seeded generation and continuous
collision tests for a spherical drone: segment-to-segment capsule distance for
lines and a moving, fixed-yaw oriented box for vehicles. The vehicle box is
expanded by the drone radius, a conservative approximation at edges/corners;
rotation within a step is not modeled. Tests cover high-speed thin-line crossing,
moving vehicle relative motion, step splitting, deterministic generation and
launch clearance.

The user must supply the bounds, dimensions, line radius, speed and exclusion
distance; bounds currently constrain obstacle centers, not full geometry, and
the generator does not test road feasibility. The reverse dossier reports
thin-line local convex bounds of about 0.052 m square by 7 m, and a vehicle
mesh with a Blueprint scale of 0.5, but instance transforms, collision channel
and vehicle path/velocity distribution remain unverified. Therefore those
numbers are not silently treated as world dimensions or defaults. This is a
training approximation under development, not evidence of official contact
behavior or completed C-level acceptance.

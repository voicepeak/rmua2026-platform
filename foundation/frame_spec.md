# Coordinate baseline (provisional)

Map exports use UE coordinates in centimetres with Z up. The intended world
comparison frame is NED in metres, Z down. For an axis-aligned map, position
conversion is (UE position - UE origin) * (0.01, 0.01, -0.01). Vectors do not
use the translation. The actual UE origin, alignment, quaternion conventions,
body frame and rotation signs still require runtime verification. A position
round trip alone cannot validate those conventions.

The explicit candidate attitude conversion treats the source quaternion as a
body-to-world rotation in wxyz order and applies the Z reflection to both
world and body: `R_ned_frd = S R_ue S`, `S = diag(1, 1, -1)`. The equivalent
normalized quaternion transform is `(w, x, y, z) -> (w, -x, -y, z)` and is its
own inverse. Tests verify matrix action, known roll sign, batch roundtrip and
invalid input. This convention assumes axis-aligned world axes and UE body Z-up;
the algebraic tests do not establish those assumptions in the official build,
globally validate map origin, or independently establish angular-rate signs.

The simulator README publishes PWM order as right-front, left-rear, left-front,
right-rear. A historical source snapshot contains a four-float RotorPWM message
and a motor-pulse summary, but these do not independently establish accepted
range, saturation, or fully reproduce motor directions. Range and direction
remain U pending runtime checks; see evidence/README.md.

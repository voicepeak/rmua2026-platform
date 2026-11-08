# Independent segment-overlap repeat protocol

The association and metrics below were fixed before inspecting the repeat data:
use the existing `1_1` runtime-FName/root correspondence and map segment index
32 down to 0 against ascending serialized actor suffixes 242 through 274.
Run the existing sensor-only VEL diagnostic on the same OL1 route in a fresh
dedicated container, at 4 m/s for at most 65 seconds, with the existing
read-only segment trace. Compare the two counter-changing event types to their
assigned `ChargeArea` signed-distance boundary using the same candidate origin
(0.0078, 0.001, -7) UE cm and nearest accepted GPS arrival. Record all events,
including incomplete or failed runs. The route is shared with the earlier
diagnostic, but process, speed and GPS/event samples are independent.

This repeat tests the frozen ordering hypothesis only. It does not independently
decode object names, establish the drone collision footprint, synchronize the
physical clock or validate other field branches. It cannot by itself close F01.

## Result

The fresh seed-123 process ran at requested 4 m/s until its 65-second timeout
(controller exit 124), and the owned container was confirmed removed. It
recorded 33 events on indices 32 through 21: 12 enter-type, 11 other-type and
10 exit-type events. For adjacent counter-changing observations, 11 enter
events were followed by +1 in the raw counter difference and 10 exits by -1.

With the frozen reverse-index hypothesis, the 12 enter-type GPS positions had
median signed distance -0.115 m to their associated `ChargeArea` boundary and
95th-percentile absolute distance 0.261 m. The 10 exit-type positions had
median signed distance +0.248 m and 95th-percentile absolute distance 0.354 m;
one of those positions was still inside. The maximum absolute GPS-arrival
offset from an event was 37.3 ms. Full per-event data, source hashes and
limitations are in `road_box_events_segment_repeat.json`; raw data and the
verified binary manifest are in `datasets/segment_repeat_seed123_20261108`.

This is additional evidence for the frozen mapping and `ChargeArea` boundary
hypothesis over only the observed first 12 segments of the same route. A
common map and seed, unsynchronized sampled GPS, undetermined drone geometry,
cross-process FName association and other zone branches limit the inference.
Natural field timing, full route coverage and F01 remain unverified.

# Rotor pointer-to-command exploratory protocol

The seed-2026 mixed-waveform capture was used to choose a provisional mapping:
four ascending rotor addresses correspond to PWM channels 0–3. A posthoc
arrival-time grid gives the same best assignment for delays 2–5 ms; these are
not certified physical delays. Before collecting the next data, freeze the
ascending-address mapping, the 0.30–0.85 s window and that delay interval.
Collect the same bounded waveform in a fresh seed-456 process, report the
fixed mapping's raw input error over the full delay interval and compare the
best alternative permutation. Preserve aborts, missing samples and logs.

The repeat shares a waveform and software build, so it is a limited check of
object ordering, not a held-out dynamic acceptance or proof that address order
is stable across arbitrary versions and instances. Breakpoints are all-stop,
the logged times are host arrivals, and motor target/filter timing remains
separate from this raw input matching.

The seed-456 process completed with 26 samples per rotor in the fixed window.
For assumed 2–5 ms arrival shifts, the preselected ascending-address mapping
had raw-input RMSE between approximately 0.000000006 and 0.0000327; the best
alternative permutation had RMSE at least 0.001109. The seed-2026 selection
run had fixed-mapping RMSE about 0.000289 over the same interval. Per-delay
values, pointers, sample counts and hashes are recorded in
`kinematic_mapping_seed456.json` and `kinematic_mapping_seed2026.json`.

This supports the provisional mapping in these two processes, under this one
waveform. Neither the 2–5 ms interval nor the matching errors isolate physical
actuator latency, and address order is not guaranteed by a general API. The
mapping has not been promoted as a universal official property; Gate B remains
open.

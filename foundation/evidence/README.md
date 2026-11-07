# PWM evidence provenance

The three files in this directory were extracted from the historical source
snapshot `experiments/pwm_20261104/seed123_closed_loop/source_snapshot.tar.gz`
in the reverse repository. The archive SHA-256 is
`cd6e22e807aee68e6574ce6d1ca0fdf4e61f40fc08440487d7857dd545090631`.
Text was copied through PowerShell, so compare content, not extracted-file byte
hashes, with the archive. The message definition is a snapshot from the former
controller workspace; it is not by itself proof of the running simulator's
accepted range or saturation behavior.

`seed123_simulator_settings.json` was extracted byte-for-byte and hashes to
`0d72886f92b091cbc5ba2ae58d25b4b4bc589045d4e725f483603efdc26c6c17`,
matching the run's simulator preflight manifest. The manifest declares seed
123 as operator-specified; it does not independently establish the runtime seed.

The simulator README publishes motor order 0 right-front, 1 left-rear, 2
left-front, 3 right-rear. The seed-1003 summary records small individual motor
pulses and corresponding angular acceleration signs, but its underlying raw
run is not present in this directory; this is supporting historical evidence,
not an independently reproducible direction test. Seed 1002 hit a position
guard, so its collective observations cannot establish steady-state response.
The later seed-456 open-loop attempt was invalid after POSITION_LIMIT. The
seed-123 closed-loop run recorded 969 matching sent/observed messages, but did
not establish actuator consumption or a calibrated transfer function.

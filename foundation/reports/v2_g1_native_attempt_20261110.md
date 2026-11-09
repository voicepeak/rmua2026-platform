# G1 non-stopping native motor-state attempt (2026-11-10)

Status: **BLOCKED**, unchanged.  Two bounded attempts were made; neither produced
an independent rotor-state or physical command-time measurement.  No parameter
grid was reopened and the sealed `v2_holdout_axis` was not read.

## Attempt 1 — uprobe/perf availability

- Probe image: `rmua/re-probe:20261104`, id
  `sha256:a1bfe1df3d0f0b57c406fc7873803c07a812f291e42c1a7eba13d2f9328fc106`.
- `command -v perf` returned nothing; `perf --version` printed
  `bash: perf: command not found`.  `gcc` 9.4.0 and `gdb` are present.
- Consequence: a non-stopping `perf probe`/uprobe capture at the native rotor
  update (`0x269ab90`) could not be started.  Installing perf was not attempted
  because it would change the probe image and is outside the bounded attempt.

## Attempt 2 — parent-side `process_vm_readv` prototype

- A disposable Python parent/child test in the same probe image used
  `process_vm_readv` on a child heap address and read 8 bytes successfully
  (`n=8`, `errno=0`).  This establishes that a parent process can sample child
  memory without a debugger stop, if it owns the child.
- The existing native interposer source
  (`D:/RMUA2026-re/scripts/ros_publish_interpose.c`) hooks
  `RotorPWM` publish and `ROTOR_PWM_Callback`, but only records ROS header
  timestamps and message headers.  It does not expose rotor state.
- No official rotor object address was decoded from a live simulator in this
  attempt, so no independent motor state, physical command-application time or
  initial velocity was obtained.

## Decision

- G1 remains **PARTIAL; parameter identification BLOCKED**.
- Reopen condition remains: an independent, low-perturbation native rotor-state
  and command-reception measurement with an independent initial velocity and
  contact/wind exclusion.
- Do not convert the ROS callback header timing into physical command time; prior
  evidence shows the header comes from a separate IMU request.

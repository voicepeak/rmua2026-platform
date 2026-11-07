"""Diagnostic open-loop replay of an existing official closed-loop recording.

This is NOT an official calibration experiment. Pose truth is posthoc only;
velocity is estimated by a local polynomial fit and motor state is guessed.
Command ROS header time is assumed comparable to truth/IMU stamp and no
transport/actuation delay is corrected. No extrapolation beyond recorded
commands is allowed. Reports retain these limitations and all source hashes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from foundation.quad import Params, Quad
from foundation.compare import compare


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def deduplicate_stamp(rows):
    """Keep last sample at repeated stamp; reject out-of-order records."""
    out = []
    duplicates = 0
    for row in rows:
        if out and row["stamp"] < out[-1]["stamp"]:
            raise ValueError("timestamps went backwards")
        if out and row["stamp"] == out[-1]["stamp"]:
            out[-1] = row
            duplicates += 1
        else:
            out.append(row)
    return out, duplicates


def series(rows, t_key, value_key, width):
    t = np.array([r[t_key] for r in rows], dtype=float)
    v = np.array([r[value_key] for r in rows], dtype=float)
    if len(t) < 2 or v.shape != (len(t), width) or not np.isfinite(v).all() or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError(f"invalid or nonmonotonic {value_key}")
    return t, v


def position_velocity(t, positions, at, half_window=.12):
    mask = np.abs(t - at) <= half_window
    offsets = t[mask] - at
    if len(offsets) < 5 or not (offsets[0] < 0 < offsets[-1]):
        raise ValueError("insufficient two-sided samples for velocity estimate")
    # Quadratic local fit estimates position and velocity without pretending
    # the differentiated, noisy pose is a measured velocity sensor.
    design = np.column_stack((np.ones(len(offsets)), offsets, offsets ** 2))
    if np.linalg.matrix_rank(design) < 3:
        raise ValueError("singular velocity fit")
    coef = np.linalg.lstsq(design, positions[mask], rcond=None)[0]
    return coef[0], coef[1]


def run(directory, offset_s, duration_s, step_s=.002, experiment="exploratory replay; not acceptance",
        command_delay_s=0., motor_tau_s=.005):
    files = {"truth": directory / "evaluation_truth.jsonl", "imu": directory / "evaluation_imu.jsonl",
             "command": directory / "evaluation_pwm_commands.jsonl"}
    truth, imu, commands = (read_jsonl(files[k]) for k in ("truth", "imu", "command"))
    truth, truth_duplicates = deduplicate_stamp(truth)
    imu, imu_duplicates = deduplicate_stamp(imu)
    tt, pos = series(truth, "stamp", "position", 3)
    _, quat = series(truth, "stamp", "orientation", 4)
    it, omega = series(imu, "stamp", "omega_body", 3)
    # Shift epoch timestamps before arithmetic to preserve numerical precision.
    epoch = tt[0]
    tt, it = tt - epoch, it - epoch
    ct = np.array([r["data"]["header"]["stamp"]["secs"] +
                   r["data"]["header"]["stamp"]["nsecs"] * 1e-9 - epoch for r in commands])
    pwm = np.array([[r["data"][f"rotorPWM{i}"] for i in range(4)] for r in commands])
    if not np.isfinite(ct).all() or np.any(np.diff(ct) <= 0) or not np.isfinite(pwm).all() or np.any((pwm < 0) | (pwm > 1)):
        raise ValueError("invalid command stream")
    if not np.isfinite(command_delay_s) or command_delay_s < 0 or not np.isfinite(motor_tau_s) or motor_tau_s <= 0:
        raise ValueError("invalid candidate delay or motor time constant")
    start = ct[0] + offset_s
    end = start + duration_s
    ct = ct + command_delay_s
    if not (duration_s > 0 and step_s > 0 and ct[0] <= start < end <= ct[-1]
            and tt[0] + .12 <= start and end <= tt[-1] - .12
            and it[0] <= start and end <= it[-1]):
        raise ValueError("requested interval exceeds recorded data")
    if np.max(np.diff(ct[(ct >= start) & (ct <= end)])) > .1:
        raise ValueError("command gap exceeds 100 ms")
    selected = tt[(tt >= start) & (tt <= end)]
    if len(selected) < 3 or np.max(np.diff(selected)) > .05:
        raise ValueError("truth sampling is insufficient or has a gap")
    if np.max(np.diff(it[(it >= start) & (it <= end)])) > .05:
        raise ValueError("IMU gap exceeds 50 ms")
    start = float(selected[0])
    initial_position, initial_velocity = position_velocity(tt, pos, start)
    qinit = Slerp(tt, Rotation.from_quat(quat[:, [1, 2, 3, 0]]))([start]).as_quat()[0]
    winit = np.array([np.interp(start, it, omega[:, i]) for i in range(3)])
    command_index = np.searchsorted(ct, start, side="right") - 1
    rps2 = (11079.03 / 60) ** 2
    params = Params(.9, (.0046890742, .0069312, .010421166), .18,
                    .000367717 * rps2, 4.888486266072161e-06 * rps2, motor_tau_s)
    plant = Quad(params)
    plant.reset(initial_position, initial_velocity, qinit[[3, 0, 1, 2]], winit, pwm[command_index])
    predictions, references = [], []
    for stamp in selected:
        target = float(stamp - start)
        while plant.time < target - 1e-10:
            absolute = start + plant.time
            command_index = min(np.searchsorted(ct, absolute + 1e-10, side="right") - 1, len(ct)-1)
            next_event = ct[command_index + 1] - absolute if command_index + 1 < len(ct) else np.inf
            dt = min(step_s, target - plant.time, next_event)
            if dt <= 1e-12:
                raise ValueError("invalid command timing")
            plant.step(pwm[command_index], dt)
        s = plant.state()
        predictions.append({k: (float(v) if k == "t" else v.tolist()) for k, v in s.items() if k != "rotor_signal"})
        ix = np.searchsorted(tt, stamp)
        _, vref = position_velocity(tt, pos, stamp)
        references.append({"t": target, "position": pos[ix].tolist(), "velocity": vref.tolist(),
                           "quaternion_wxyz": quat[ix].tolist(),
                           "omega": [float(np.interp(stamp, it, omega[:, i])) for i in range(3)]})
    result = compare(references, predictions, max_gap=.05)
    result.update({"experiment": experiment,
                   "time_basis": "recorded ROS command header and truth/IMU stamps, assumed same clock",
                   "absolute_start_stamp": epoch + start,
                   "initial_velocity": "quadratic local fit to posthoc positions over +/- 0.12 s",
                   "reference_velocity": "same local fit; not directly measured",
                   "initial_rotor_signal": "command at start, unmeasured",
                   "duplicate_stamps_keep_last": {"truth": truth_duplicates, "imu": imu_duplicates},
                   "motor_tau_s": params.motor_tau, "command_delay_s": command_delay_s,
                   "force_model": "no wind, drag or collision; selected interval must be examined",
                   "source_sha256": {k: hashlib.sha256(v.read_bytes()).hexdigest() for k, v in files.items()}})
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--offset", type=float, default=5.)
    p.add_argument("--duration", type=float, default=1.)
    p.add_argument("--experiment", default="exploratory replay; not acceptance")
    p.add_argument("--command-delay", type=float, default=0.)
    p.add_argument("--motor-tau", type=float, default=.005)
    args = p.parse_args()
    result = run(args.directory, args.offset, args.duration, experiment=args.experiment,
                 command_delay_s=args.command_delay, motor_tau_s=args.motor_tau)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

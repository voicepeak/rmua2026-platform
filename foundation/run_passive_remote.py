"""Run one isolated, time-bounded official capture via SSH."""
import argparse
import hashlib
import json
from pathlib import Path
import uuid

from foundation.remote_preflight import connect


def main():
    p = argparse.ArgumentParser()
    p.add_argument("connection_script", type=Path)
    p.add_argument("output", type=Path, help="new local output directory")
    p.add_argument("--mode", choices=("passive", "collective", "aero_probe", "aero_force_probe", "aero_wind0", "aero_wind3", "aero_wind3_tilt", "differential", "yaw", "yaw_small", "roll_pitch", "roll_pitch_small", "braking", "small_steps", "rotor_probe", "contact_probe", "field_forced0", "field_forced1", "field_toggle", "field_toggle_high", "field_toggle_air", "field_trace", "field_recovery", "cadence20", "cadence50", "cadence100", "frame_probe", "frame_lateral"), default="passive")
    p.add_argument("--seed", type=int, default=123)
    args = p.parse_args()
    if not 0 <= args.seed <= 2147483647:
        p.error("seed must be a nonnegative 32-bit integer")
    if args.mode == "passive" and args.seed != 123:
        p.error("passive harness currently uses seed 123")
    args.output.mkdir(parents=True, exist_ok=False)
    remote = "/tmp/foundation-" + uuid.uuid4().hex
    here = Path(__file__).parent / "probes"
    c = connect(args.connection_script)
    try:
        sftp = c.open_sftp()
        sftp.mkdir(remote, mode=0o700)
        sources = (["collect_ros.py", "passive_remote.sh"] if args.mode == "passive" else
                   ["collect_ros.py", "collective_remote.sh", "pwm_collective.py"])
        if args.mode in ('frame_probe', 'frame_lateral'):
            sources.append('frame_readonly.gdb')
        if args.mode == 'rotor_probe':
            sources.append('rotor_readonly.gdb')
        if args.mode == 'aero_probe':
            sources.append('aero_readonly.gdb')
        if args.mode == 'aero_force_probe':
            sources.append('aero_force_readonly.gdb')
        if args.mode in ('aero_wind0', 'aero_wind3', 'aero_wind3_tilt'):
            sources.append('aero_wind_injected.gdb')
        if args.mode == 'contact_probe':
            sources.append('contact_readonly.gdb')
        if args.mode in ('field_forced0', 'field_forced1'):
            sources.append('field_forced.gdb')
        if args.mode in ('field_toggle', 'field_toggle_high', 'field_toggle_air'):
            sources.append('field_toggle.gdb')
        if args.mode == 'field_trace':
            sources.append('field_trace.gdb')
        if args.mode == 'field_recovery':
            sources.append('field_recovery.gdb')
        for name in sources:
            sftp.put(str(here / name), remote + "/" + name)
        script = "passive_remote.sh" if args.mode == "passive" else "collective_remote.sh"
        _, out, err = c.exec_command("timeout --signal=TERM --kill-after=15s 120s bash " + remote + "/" + script + " " + args.mode + " " + str(args.seed), timeout=145)
        stdout, stderr = out.read().decode("utf-8", "replace"), err.read().decode("utf-8", "replace")
        status = out.channel.recv_exit_status()
        for name in ("start.log", "stop.log", "binary_check.log", "topics.txt", "collector.log", "player.log", "capture.jsonl", "commands.jsonl", "cleanup.log", "pool/pool.json", "scripts/headless_pool.py"):
            try:
                dest = args.output / name
                dest.parent.mkdir(parents=True, exist_ok=True)
                sftp.get(remote + "/" + name, str(dest))
            except FileNotFoundError:
                pass
        try:
            for name in sftp.listdir(remote + '/pool'):
                if name.endswith('.log'):
                    sftp.get(remote + '/pool/' + name, str(args.output/'pool'/name))
        except FileNotFoundError:
            pass
        hashes = {name: hashlib.sha256((here / name).read_bytes()).hexdigest() for name in sources}
        for name in sources:
            (args.output / name).write_bytes((here / name).read_bytes())
        (args.output / "run.json").write_text(json.dumps({"remote_directory": remote, "exit_status": status,
            "mode": args.mode, "seed_requested": args.seed, "stdout": stdout, "stderr": stderr, "source_sha256": hashes}, indent=2) + "\n", encoding="utf-8")
        sftp.close()
        print("exit_status:", status)
        print(stdout, stderr)
        if status:
            raise SystemExit(status)
    finally:
        c.close()


if __name__ == "__main__":
    main()

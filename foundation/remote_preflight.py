"""Read-only remote preflight using existing local connection configuration.

No credentials are logged. SSH host key must already be in system/user
known_hosts; this program deliberately does not trust unknown host keys.
"""
import argparse
import ast
from pathlib import Path
import paramiko


def connect(connection_script):
    tree = ast.parse(connection_script.read_text(encoding="utf-8-sig"))
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Attribute) and node.func.attr == "connect"]
    if len(calls) != 1:
        raise ValueError("expected exactly one connection specification")
    call = calls[0]
    positional = [ast.literal_eval(value) for value in call.args]
    options = {item.arg: ast.literal_eval(item.value) for item in call.keywords}
    options.update(timeout=8, banner_timeout=8, auth_timeout=8)
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect(*positional, **options)
    return client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("connection_script", type=Path)
    args = parser.parse_args()
    client = connect(args.connection_script)
    try:
        command = ("hostname; docker ps --format 'container={{.Names}} image={{.Image}}'; "
                   "docker image inspect rmua/noetic-xal:driver-recovery-20261012 --format 'probe_image={{.Id}}' 2>/dev/null || true; "
                   "ps -eo pid,args | grep '[R]MUA-Linux-Shipping' || true; "
                   "test -d /home/tianbot/RUMA-by-helinjun/beta/rmua_ws && echo ros_workspace_available; "
                   "test -f /home/tianbot/桌面/RMUA2026-re/scripts/headless_pool.py && echo headless_pool_available; "
                   "test -f /home/tianbot/RUMA-by-helinjun/main/configs/simulator_settings.json && echo simulator_settings_available; "
                   "test -f /home/tianbot/RUMA-by-helinjun/main/simulator/simulator_12.0.0.5/Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping && echo simulator_available")
        _, stdout, stderr = client.exec_command(command, timeout=15)
        print(stdout.read().decode("utf-8", "replace"))
        print(stderr.read().decode("utf-8", "replace"))
    finally:
        client.close()


if __name__ == "__main__":
    main()

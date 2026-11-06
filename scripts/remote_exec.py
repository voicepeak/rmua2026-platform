"""Generic SSH command runner for the RMUA remote host.

Reads credentials from environment variables; never hardcode them:

    $env:RMUA_SSH_HOST="100.120.200.58"
    $env:RMUA_SSH_USER="tianbot"
    $env:RMUA_SSH_PASSWORD="..."   # from HANDOFF.local.md, do not commit
    python scripts/remote_exec.py "uname -a"

Requires paramiko.
"""

from __future__ import annotations

import os
import sys

import paramiko


def main(argv=None) -> int:
    host = os.environ.get("RMUA_SSH_HOST", "100.120.200.58")
    user = os.environ.get("RMUA_SSH_USER", "tianbot")
    password = os.environ.get("RMUA_SSH_PASSWORD")
    if not password:
        raise SystemExit("set RMUA_SSH_PASSWORD (see HANDOFF.local.md)")
    command = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "uname -a"
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        host,
        22,
        username=user,
        password=password,
        timeout=15,
        banner_timeout=15,
        auth_timeout=15,
    )
    _, stdout, stderr = client.exec_command(command, timeout=300)
    print(stdout.read().decode("utf-8", errors="replace"))
    err = stderr.read().decode("utf-8", errors="replace")
    if err:
        print("--- stderr ---")
        print(err)
    code = stdout.channel.recv_exit_status()
    client.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())

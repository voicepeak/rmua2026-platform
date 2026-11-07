#!/usr/bin/env bash
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
POOL=/home/tianbot/桌面/RMUA2026-re/scripts/headless_pool.py
SIM=/home/tianbot/RUMA-by-helinjun/main/simulator/simulator_12.0.0.5
SETTINGS=/home/tianbot/RUMA-by-helinjun/main/configs/simulator_settings.json
cleanup() {
  if [ -f "$HERE/pool/pool.json" ]; then
    python3 "$POOL" stop "$HERE/pool/pool.json" >> "$HERE/cleanup.log" 2>&1
  fi
}
trap 'cleanup || true' EXIT TERM INT
SEED=${2:-123}
case "$SEED" in *[!0-9]*|'') exit 3 ;; esac
MODE=${1:-collective}
if [ "$MODE" = frame_probe ]; then
  printf '%s  %s\n' 5231addfa25ddaa9e4473ea9ac40c9c0bc685174e69cd14bb49159f443b97800 "$SIM/Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping" | sha256sum -c > "$HERE/binary_check.log"
  mkdir "$HERE/scripts"
  cp "$POOL" "$HERE/scripts/headless_pool.py"
  cp "$HERE/frame_readonly.gdb" "$HERE/scripts/crash_capture.gdb"
  POOL="$HERE/scripts/headless_pool.py"
  python3 "$POOL" start --simulator "$SIM" --settings "$SETTINGS" --seeds "$SEED" --port 11821 --output "$HERE/pool" --timeout 75 --gdb-crash --image rmua/re-probe:20261104 > "$HERE/start.log" 2>&1
else
  python3 "$POOL" start --simulator "$SIM" --settings "$SETTINGS" --seeds "$SEED" --port 11821 --output "$HERE/pool" --timeout 75 > "$HERE/start.log" 2>&1
fi
CONTAINER=$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); assert d["ready"] and len(d["instances"]) == 1; print(d["instances"][0]["container"])' "$HERE/pool/pool.json")
docker exec -i "$CONTAINER" bash -c 'source /opt/ros/noetic/setup.bash; python3 - /tmp/foundation_capture.jsonl 4' < "$HERE/collect_ros.py" > "$HERE/collector.log" 2>&1 &
COLLECTOR=$!
sleep .5
set +e
case "$MODE" in collective|differential|yaw|yaw_small|roll_pitch|roll_pitch_small|cadence20|cadence50|cadence100|frame_probe) ;; *) exit 3 ;; esac
docker exec -i "$CONTAINER" bash -c 'source /opt/ros/noetic/setup.bash; python3 - /tmp/foundation_commands.jsonl "$1"' _ "$MODE" < "$HERE/pwm_collective.py" > "$HERE/player.log" 2>&1
PLAYER_STATUS=$?
# Stop the dedicated simulator as soon as the guarded player exits, including
# on abort. Files remain available to docker cp from the stopped container.
docker stop --time 1 "$CONTAINER" > "$HERE/stop.log" 2>&1
STOP_STATUS=$?
wait "$COLLECTOR"
COLLECTOR_STATUS=$?
set -e
docker cp "$CONTAINER:/tmp/foundation_capture.jsonl" "$HERE/capture.jsonl" || true
docker cp "$CONTAINER:/tmp/foundation_commands.jsonl" "$HERE/commands.jsonl" || true
cleanup
trap - EXIT TERM INT
printf 'player=%s stop=%s collector=%s (collector may be interrupted by stop)\n' "$PLAYER_STATUS" "$STOP_STATUS" "$COLLECTOR_STATUS"
[ "$PLAYER_STATUS" -eq 0 ] && [ "$STOP_STATUS" -eq 0 ] && [ -s "$HERE/capture.jsonl" ] && [ -s "$HERE/commands.jsonl" ]

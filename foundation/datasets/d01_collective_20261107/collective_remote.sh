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
python3 "$POOL" start --simulator "$SIM" --settings "$SETTINGS" --seeds 123 --port 11821 --output "$HERE/pool" --timeout 75 > "$HERE/start.log" 2>&1
CONTAINER=$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); assert d["ready"] and len(d["instances"]) == 1; print(d["instances"][0]["container"])' "$HERE/pool/pool.json")
docker exec -i "$CONTAINER" bash -c 'source /opt/ros/noetic/setup.bash; python3 - /tmp/foundation_capture.jsonl 4' < "$HERE/collect_ros.py" > "$HERE/collector.log" 2>&1 &
COLLECTOR=$!
sleep .5
set +e
docker exec -i "$CONTAINER" bash -c 'source /opt/ros/noetic/setup.bash; python3 - /tmp/foundation_commands.jsonl' < "$HERE/pwm_collective.py" > "$HERE/player.log" 2>&1
PLAYER_STATUS=$?
if [ "$PLAYER_STATUS" -ne 0 ]; then
  docker cp "$CONTAINER:/tmp/foundation_capture.jsonl" "$HERE/capture.jsonl" || true
  docker cp "$CONTAINER:/tmp/foundation_commands.jsonl" "$HERE/commands.jsonl" || true
  cleanup
  trap - EXIT TERM INT
  wait "$COLLECTOR" || true
  exit "$PLAYER_STATUS"
fi
wait "$COLLECTOR"
COLLECTOR_STATUS=$?
set -e
docker cp "$CONTAINER:/tmp/foundation_capture.jsonl" "$HERE/capture.jsonl" || true
docker cp "$CONTAINER:/tmp/foundation_commands.jsonl" "$HERE/commands.jsonl" || true
cleanup
trap - EXIT TERM INT
printf 'player=%s collector=%s\n' "$PLAYER_STATUS" "$COLLECTOR_STATUS"
[ "$PLAYER_STATUS" -eq 0 ] && [ "$COLLECTOR_STATUS" -eq 0 ]

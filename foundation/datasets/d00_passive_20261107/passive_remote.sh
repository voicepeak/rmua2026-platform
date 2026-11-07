#!/usr/bin/env bash
# Runs in a fresh, private remote experiment directory. Never touches an
# existing container: headless_pool creates and identity-checks its own pool.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
POOL=/home/tianbot/桌面/RMUA2026-re/scripts/headless_pool.py
SIM=/home/tianbot/RUMA-by-helinjun/main/simulator/simulator_12.0.0.5
SETTINGS=/home/tianbot/RUMA-by-helinjun/main/configs/simulator_settings.json
cleanup() {
  if [ -f "$HERE/pool/pool.json" ]; then
    python3 "$POOL" stop "$HERE/pool/pool.json" >> "$HERE/cleanup.log" 2>&1 || true
  fi
}
trap cleanup EXIT TERM INT
python3 "$POOL" start --simulator "$SIM" --settings "$SETTINGS" --seeds 123 --port 11821 --output "$HERE/pool" --timeout 75 > "$HERE/start.log" 2>&1
CONTAINER=$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); assert d["ready"] and len(d["instances"]) == 1; print(d["instances"][0]["container"])' "$HERE/pool/pool.json")
docker exec "$CONTAINER" bash -c 'source /opt/ros/noetic/setup.bash; rostopic list' > "$HERE/topics.txt" 2>&1
docker exec -i "$CONTAINER" bash -c 'source /opt/ros/noetic/setup.bash; python3 - /tmp/foundation_capture.jsonl 5' < "$HERE/collect_ros.py" > "$HERE/collector.log" 2>&1
docker cp "$CONTAINER:/tmp/foundation_capture.jsonl" "$HERE/capture.jsonl"
cleanup
trap - EXIT TERM INT
echo completed

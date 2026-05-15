#!/usr/bin/env bash
# SLAM bench orchestrator: A/B compare arise_slam_mid360 vs lightning-lm
# under identical input from a recorded rosbag.
#
# For each backend, this script:
#   1) launches just the SLAM package (no autonomy, no rviz)
#   2) waits for /state_estimation to start publishing
#   3) starts trajectory / latency / compute probes
#   4) plays the bag (blocking)
#   5) sleeps to settle, then SIGINTs everything in reverse
# Then it writes a markdown summary comparing both runs.
#
# The bag is expected to contain /livox/imu plus BOTH lidar streams:
#   /livox/lidar         (PointCloud2)  — consumed by Lightning
#   /livox/lidar_custom  (typego CustomMsg) — consumed by ARISE
# Topic checks below will warn if any is missing.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_DIR="$REPO_ROOT/scripts/slam_bench"

BAG=""
DURATION=""             # optional max wall-clock cap (s); 0/empty = bag length
BAG_RATE="1.0"
BACKENDS="arise,lightning"
REPEATS=1
OUT_BASE="$REPO_ROOT/logs/slam_bench"

usage() {
  cat <<EOF
Usage: $0 --bag PATH [options]

Required:
  --bag PATH              path to ros2 bag directory (with metadata.yaml inside)

Options:
  --duration SEC          hard wall-clock cap per run (default: play full bag)
  --bag-rate FLOAT        ros2 bag play --rate (default: 1.0)
  --backends a,b          comma-separated list (default: arise,lightning)
  --repeats N             repeat each backend N times (default: 1)
  --out DIR               output base dir (default: $OUT_BASE)
  -h, --help              this message

Output: <OUT>/<timestamp>/<backend>[/run_i]/ + summary.md
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)       BAG="$2"; shift 2 ;;
    --duration)  DURATION="$2"; shift 2 ;;
    --bag-rate)  BAG_RATE="$2"; shift 2 ;;
    --backends)  BACKENDS="$2"; shift 2 ;;
    --repeats)   REPEATS="$2"; shift 2 ;;
    --out)       OUT_BASE="$2"; shift 2 ;;
    -h|--help)   usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

[[ -z "$BAG" ]] && { echo "error: --bag is required" >&2; usage; exit 2; }
[[ ! -d "$BAG" ]] && { echo "error: bag dir not found: $BAG" >&2; exit 2; }

# --- environment ---
# colcon's setup.bash references unbound vars (COLCON_TRACE); relax `set -u`
# just for the source.
if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "$REPO_ROOT/install/setup.bash"
  set -u
else
  echo "warning: $REPO_ROOT/install/setup.bash not found — run 'colcon build' first?" >&2
fi

command -v ros2 >/dev/null || { echo "error: ros2 not on PATH (source /opt/ros/humble/setup.bash)" >&2; exit 3; }

# Clear the ros2 daemon so topic discovery starts from a clean slate — a stale
# daemon can cache a "no publishers" view and never refresh it.
ros2 daemon stop >/dev/null 2>&1 || true
command -v python3 >/dev/null || { echo "error: python3 not on PATH" >&2; exit 3; }
python3 -c "import psutil" 2>/dev/null || {
  echo "error: python3 'psutil' missing — install with: pip install --user psutil" >&2
  exit 3
}

# --- output dir + bag info ---
TS="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$OUT_BASE/$TS"
mkdir -p "$RUN_DIR"
echo "==> bench output dir: $RUN_DIR"

BAG_INFO="$RUN_DIR/bag_info.txt"
echo "==> recording bag info to $BAG_INFO"
ros2 bag info "$BAG" > "$BAG_INFO" 2>&1 || true

# Warn on missing topics.
for t in "/livox/lidar" "/livox/lidar_custom" "/livox/imu"; do
  if ! grep -q " $t " "$BAG_INFO"; then
    echo "WARNING: bag does not appear to contain topic '$t' — at least one backend will starve." >&2
  fi
done

# --- cleanup machinery ---
SLAM_PGID=""
PROBE_PIDS=()
BAG_PID=""

stop_pid_group() {
  # Send SIGINT to a process group, escalate to SIGTERM then SIGKILL on timeout.
  local pgid="$1" sig="${2:-INT}" wait_s="${3:-5}"
  [[ -z "$pgid" ]] && return 0
  if kill -0 "-$pgid" 2>/dev/null; then
    kill -"$sig" "-$pgid" 2>/dev/null || true
    local waited=0
    while kill -0 "-$pgid" 2>/dev/null && (( waited < wait_s * 10 )); do
      sleep 0.1; waited=$((waited+1))
    done
    if kill -0 "-$pgid" 2>/dev/null; then
      echo "  pgid $pgid still alive after SIG$sig; SIGTERM"
      kill -TERM "-$pgid" 2>/dev/null || true; sleep 1
    fi
    if kill -0 "-$pgid" 2>/dev/null; then
      echo "  pgid $pgid still alive; SIGKILL"
      kill -KILL "-$pgid" 2>/dev/null || true
    fi
  fi
}

cleanup() {
  local rc=$?
  echo "==> cleanup (rc=$rc)"
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  for pid in "${PROBE_PIDS[@]:-}"; do
    [[ -n "${pid:-}" ]] && kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "${PROBE_PIDS[@]:-}"; do
    [[ -n "${pid:-}" ]] && wait "$pid" 2>/dev/null || true
  done
  stop_pid_group "$SLAM_PGID"
  # Belt-and-suspenders: kill any lingering SLAM exe by name.
  pkill -INT -f "run_slam_online" 2>/dev/null || true
  pkill -INT -f "feature_extraction_node" 2>/dev/null || true
  pkill -INT -f "laser_mapping_node" 2>/dev/null || true
  pkill -INT -f "imu_preintegration_node" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  # Poll for a live publisher on $topic. --no-daemon forces a fresh DDS
  # discovery on every check, so a stale daemon cache can't mask the topic.
  local topic="$1" timeout_s="${2:-40}"
  local deadline=$(( SECONDS + timeout_s ))
  while (( SECONDS < deadline )); do
    if ros2 topic info "$topic" --no-daemon 2>/dev/null \
         | grep -qE 'Publisher count:[[:space:]]*[1-9]'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

run_one() {
  local backend="$1"
  local out_dir="$2"
  mkdir -p "$out_dir"

  echo
  echo "============================================================"
  echo "  backend=$backend → $out_dir"
  echo "============================================================"

  local launch_pkg launch_file
  case "$backend" in
    arise)     launch_pkg="arise_slam_mid360"; launch_file="arise_slam.launch.py" ;;
    lightning) launch_pkg="lightning";         launch_file="lightning_lm.launch.py" ;;
    *) echo "error: unknown backend '$backend'" >&2; return 2 ;;
  esac

  # Launch SLAM in its own process group via setsid so we can kill the whole tree.
  echo "==> launching $backend SLAM..."
  setsid ros2 launch "$launch_pkg" "$launch_file" \
    >"$out_dir/slam.log" 2>&1 &
  local slam_pid=$!
  SLAM_PGID="$slam_pid"   # setsid → child becomes group leader, pgid == pid
  echo "    slam_pid=$slam_pid (pgid=$SLAM_PGID); log=$out_dir/slam.log"

  echo "==> waiting for /state_estimation publisher..."
  if ! wait_for_topic "/state_estimation" 30; then
    echo "ERROR: /state_estimation never appeared. SLAM may have failed; see $out_dir/slam.log" >&2
    stop_pid_group "$SLAM_PGID"
    SLAM_PGID=""
    return 4
  fi
  # Give the SLAM a beat to fully initialize subscribers.
  sleep 2

  echo "==> starting probes..."
  PROBE_PIDS=()
  # The rclpy probes run with use_sim_time:=true so their clock tracks the
  # bag's /clock. Without it, lag = (wall-clock now) - (bag's recorded lidar
  # stamp) ≈ "time since the bag was recorded", not the SLAM pipeline latency.
  python3 "$SCRIPT_DIR/trajectory_logger.py" \
    --topic /state_estimation \
    --out "$out_dir/trajectory.tum" \
    --drift-out "$out_dir/drift.json" \
    --ros-args -p use_sim_time:=true \
    > "$out_dir/trajectory.log" 2>&1 &
  PROBE_PIDS+=($!)
  python3 "$SCRIPT_DIR/latency_logger.py" \
    --topic /state_estimation \
    --out "$out_dir/latency.csv" \
    --ros-args -p use_sim_time:=true \
    > "$out_dir/latency.log" 2>&1 &
  PROBE_PIDS+=($!)
  python3 "$SCRIPT_DIR/compute_probe.py" \
    --backend "$backend" \
    --out "$out_dir/compute.csv" \
    --interval 0.5 \
    > "$out_dir/compute.log" 2>&1 &
  PROBE_PIDS+=($!)
  echo "    probe pids: ${PROBE_PIDS[*]}"
  # Give probes a moment to subscribe / prime cpu_percent.
  sleep 1

  echo "==> playing bag (rate=$BAG_RATE)..."
  if [[ -n "$DURATION" && "$DURATION" != "0" ]]; then
    timeout --signal=INT "$DURATION" \
      ros2 bag play "$BAG" --rate "$BAG_RATE" --clock \
      > "$out_dir/bag_play.log" 2>&1 &
  else
    ros2 bag play "$BAG" --rate "$BAG_RATE" --clock \
      > "$out_dir/bag_play.log" 2>&1 &
  fi
  BAG_PID=$!
  wait "$BAG_PID" || true
  BAG_PID=""
  echo "==> bag finished. settling for 3s..."
  sleep 3

  echo "==> stopping probes..."
  for pid in "${PROBE_PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "${PROBE_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  PROBE_PIDS=()

  echo "==> stopping SLAM..."
  stop_pid_group "$SLAM_PGID"
  SLAM_PGID=""

  echo "==> cool-down 5s..."
  sleep 5
}

IFS=',' read -r -a BACKEND_ARR <<< "$BACKENDS"
for be in "${BACKEND_ARR[@]}"; do
  be="$(echo "$be" | xargs)"   # trim
  [[ -z "$be" ]] && continue
  for i in $(seq 1 "$REPEATS"); do
    if (( REPEATS > 1 )); then
      out="$RUN_DIR/$be/run_$i"
    else
      out="$RUN_DIR/$be"
    fi
    if ! run_one "$be" "$out"; then
      echo "WARNING: backend '$be' run failed — continuing with remaining backends." >&2
    fi
  done
done

echo
echo "==> writing summary"
python3 "$SCRIPT_DIR/summarize.py" "$RUN_DIR" || true
echo "==> done. results in: $RUN_DIR"

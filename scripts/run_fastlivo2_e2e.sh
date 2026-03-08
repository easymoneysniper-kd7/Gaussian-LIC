#!/bin/bash
# =============================================================================
# run_fastlivo2_e2e.sh
#
# One-shot script to run the full FAST-LIVO2 + Gaussian-LIC pipeline inside
# the gaussian_lic_dev Docker container.
#
# Usage (from the host machine, inside the Gaussian-LIC project root):
#   bash scripts/run_fastlivo2_e2e.sh [dataset_name] [bag_rate]
#
# Example:
#   bash scripts/run_fastlivo2_e2e.sh hku_campus_seq_00_fastlivo2_direct 1.0
#
# The script:
#   1. Starts roscore
#   2. Launches FAST-LIVO2 (from catkin_fastlivo2 workspace)
#   3. Launches bridge + Gaussian-LIC backend  (from catkin_gaussian workspace)
#   4. Plays the bag at the specified rate
#   5. Waits for gs_mapping to finish (it auto-exits when data stops)
#   6. Collects the result path
# =============================================================================

set -e

CONTAINER="gaussian_lic_dev"
BAG_PATH="/root/catkin_gaussian/src/Gaussian-LIC/datasets/hku_campus_seq_00.bag"
DATASET_NAME="${1:-hku_campus_seq_00_fastlivo2_direct}"
BAG_RATE="${2:-1.0}"

LOG_DIR="/tmp/gaussian_lic_logs"

# ── Helpers ──────────────────────────────────────────────────────────────────

dexec()  { docker exec    "$CONTAINER" bash -c "$*"; }
dexecd() { docker exec -d "$CONTAINER" bash -c "$*"; }

maybe_start_container() {
    if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
        echo "[runner] Starting container $CONTAINER ..."
        docker start "$CONTAINER"
        sleep 3
    fi
}

kill_all_ros() {
    echo "[runner] Killing all previous ROS processes ..."
    # Kill in reverse order: bag → gs_mapping → bridge → fastlivo2 → roscore
    dexec "pkill -f 'rosbag play' 2>/dev/null; true" || true
    dexec "pkill -f 'gs_mapping' 2>/dev/null; true" || true
    dexec "pkill -f 'fastlivo2_bridge' 2>/dev/null; true" || true
    dexec "pkill -f 'roslaunch' 2>/dev/null; true" || true
    dexec "pkill -f 'laserMapping' 2>/dev/null; true" || true
    dexec "pkill -f 'roscore' 2>/dev/null; true" || true
    dexec "pkill -f 'rosmaster' 2>/dev/null; true" || true
    sleep 3
    # Force kill anything stubborn
    dexec "pkill -9 -f 'gs_mapping' 2>/dev/null; true" || true
    dexec "pkill -9 -f 'laserMapping' 2>/dev/null; true" || true
    dexec "pkill -9 -f 'rosmaster' 2>/dev/null; true" || true
    sleep 2
    echo "[runner] ROS process cleanup done."
}

# ── Main ─────────────────────────────────────────────────────────────────────

maybe_start_container

kill_all_ros

echo "[runner] Cleaning up previous log dir ..."
dexec "rm -rf $LOG_DIR && mkdir -p $LOG_DIR"

# 1. roscore
echo "[runner] Starting roscore ..."
dexecd "source /opt/ros/noetic/setup.bash && roscore > $LOG_DIR/roscore.log 2>&1"
sleep 3

# 2. FAST-LIVO2 frontend (uses its own catkin_fastlivo2 workspace)
echo "[runner] Starting FAST-LIVO2 ..."
dexecd "
source /opt/ros/noetic/setup.bash
source /root/catkin_fastlivo2/devel/setup.bash
roslaunch fast_livo mapping_hku.launch rviz:=false > $LOG_DIR/fastlivo2.log 2>&1
"
echo "[runner] Waiting for FAST-LIVO2 to initialise (10 s) ..."
sleep 10

# 3. Bridge + Gaussian-LIC backend (uses catkin_gaussian workspace)
echo "[runner] Starting bridge + gs_mapping ..."
dexecd "
source /opt/ros/noetic/setup.bash
source /root/catkin_gaussian/devel/setup.bash
roslaunch gaussian_lic fastlivo.launch dataset_name:=$DATASET_NAME > $LOG_DIR/gaussian_lic.log 2>&1
"
echo "[runner] Waiting for backend to initialise (10 s) ..."
sleep 10

# 4. Play bag
echo "[runner] Playing bag at rate $BAG_RATE ..."
dexecd "
source /opt/ros/noetic/setup.bash
rosbag play $BAG_PATH -r $BAG_RATE \
  /camera/image_color/compressed:=/camera/image_color/compressed \
  > $LOG_DIR/rosbag.log 2>&1
"

# 5. Stream gs_mapping log in real time until it exits
echo "[runner] Streaming gs_mapping output (Ctrl-C safe – process keeps running) ..."
echo "──────────────────────────────────────────────────────────────────"
# tail -F follows the log as it grows; we stop once gs_mapping exits
docker exec gaussian_lic_dev bash -c "
  # Wait for the log file to appear
  for i in \$(seq 1 30); do
    [ -s $LOG_DIR/gaussian_lic.log ] && break
    sleep 1
  done
  tail -F $LOG_DIR/gaussian_lic.log &
  TAIL_PID=\$!
  # Poll until gs_mapping is gone
  while pgrep -x gs_mapping > /dev/null 2>&1; do
    sleep 2
  done
  sleep 3   # flush last lines
  kill \$TAIL_PID 2>/dev/null
  true
"
echo "──────────────────────────────────────────────────────────────────"
echo "[runner] gs_mapping finished."

# 6. Show results
echo ""
echo "========================================================"
echo " Results"
echo "========================================================"
dexec "ls -lt /root/catkin_gaussian/src/Gaussian-LIC/result/ | head -5"
LATEST_DIR=$(dexec "ls -td /root/catkin_gaussian/src/Gaussian-LIC/result/*fastlivo2*/ 2>/dev/null | head -1 | tr -d '\\n'" || true)
if [ -n "$LATEST_DIR" ]; then
    echo ""
    echo "Latest result: $LATEST_DIR"
    dexec "cat ${LATEST_DIR}metrics.txt 2>/dev/null" || echo "(metrics.txt not yet available)"
fi

echo ""
echo "[runner] Logs saved in container at $LOG_DIR"
echo "[runner] Done."

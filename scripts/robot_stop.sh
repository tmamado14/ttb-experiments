#!/usr/bin/env bash
# Stop the robot-side ROS 2 nodes (does NOT power off the robot).
#   Usage:  ./scripts/robot_stop.sh
#
# Exits non-zero if anything is still running afterwards, so "it printed
# something" and "it worked" stop being different claims.
set -euo pipefail
cd "$(dirname "$0")/.."
set -a; source .env; set +a

# Every node bringup starts. hlds_laser_publisher is the one that used to get
# missed: it is the LDS-01 driver and it does NOT match "turtlebot3", so /scan
# kept publishing long after everything else was dead. robot.launch.py is the
# launcher -- killing it alone leaves all of its children running, which is
# exactly what made the previous version look like it had worked.
NODE_PATTERN='turtlebot3_ros|turtlebot3_node|image_publisher|robot_state_publisher|hlds_laser_publisher|hls_lfcd_lds_driver|robot\.launch\.py'

# The remote command's own text contains NODE_PATTERN, so a naive `pkill -f`
# matches the very shell running it and the session dies mid-script -- silently,
# because the old `|| true` swallowed it. Every process in our own SSH session
# carries this sentinel in its command line, so it can be told apart from a real
# ROS node by inspecting /proc rather than by guessing at PIDs ($$ misses the
# subshells that command substitution creates).
SENTINEL='__ttb_stop_sentinel__'

REMOTE_CMD='
PAT="'"${NODE_PATTERN}"'"
SENT="'"${SENTINEL}"'"

victims() {
  for p in $(pgrep -f "$PAT" 2>/dev/null); do
    [ -r "/proc/$p/cmdline" ] || continue
    if tr "\0" " " < "/proc/$p/cmdline" | grep -q "$SENT"; then
      continue                      # one of ours, not a robot node
    fi
    echo "$p"
  done
}

list=$(victims)
if [ -z "$list" ]; then
  echo "ALL ROS NODES STOPPED (nothing was running)"
  exit 0
fi

echo "stopping:"
for p in $list; do
  echo "  $(tr "\0" " " < /proc/$p/cmdline 2>/dev/null | cut -c1-90)"
done

# TERM first so nodes can shut their hardware down tidily, KILL only if needed.
kill -TERM $list 2>/dev/null || true
sleep 3
list=$(victims)
if [ -n "$list" ]; then
  kill -KILL $list 2>/dev/null || true
  sleep 2
  list=$(victims)
fi

if [ -n "$list" ]; then
  echo "STILL RUNNING after TERM and KILL:"
  for p in $list; do
    echo "  $p $(tr "\0" " " < /proc/$p/cmdline 2>/dev/null | cut -c1-80)"
  done
  exit 1
fi
echo "ALL ROS NODES STOPPED"
'

# Retried, because DDS traffic from a running bringup starves the SSH handshake
# and a first attempt times out often enough to matter -- observed repeatedly.
# Once the nodes are dead the link frees up, so a retry that gets through tends
# to be the one that finishes the job.
run_remote() {
  local attempt
  for attempt in 1 2 3; do
    if command -v sshpass >/dev/null 2>&1; then
      sshpass -p "${ROBOT_PASSWORD}" ssh -o StrictHostKeyChecking=accept-new \
        -o ConnectTimeout=30 -o ServerAliveInterval=5 \
        "${ROBOT_USER}@${ROBOT_IP}" "${REMOTE_CMD} : ${SENTINEL}" && return 0
    else
      ssh -o StrictHostKeyChecking=accept-new \
        -o ConnectTimeout=30 -o ServerAliveInterval=5 \
        "${ROBOT_USER}@${ROBOT_IP}" "${REMOTE_CMD} : ${SENTINEL}" && return 0
    fi
    echo "  (attempt ${attempt} failed - retrying)" >&2
    sleep 3
  done
  return 1
}

if ! run_remote; then
  echo "ERROR: could not stop the robot's nodes over SSH." >&2
  echo "  The robot may still be running them. Check with:" >&2
  echo "    ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ros2 topic list --no-daemon" >&2
  exit 1
fi

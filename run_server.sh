#!/usr/bin/env bash
# Launch the TurtleBot3 motion server on this laptop using values from .env.
#   Usage:  ./run_server.sh
set -euo pipefail
cd "$(dirname "$0")"

# --- load .env ---------------------------------------------------------------
if [[ ! -f .env ]]; then
  echo "ERROR: .env not found. Copy the template first:  cp .env.example .env" >&2
  exit 1
fi
set -a; source .env; set +a

# --- source ROS 2 ------------------------------------------------------------
# ROS's setup scripts reference unset vars, so relax `set -u` while sourcing.
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID
unset FASTRTPS_DEFAULT_PROFILES_FILE   # default multicast discovery works here

echo "Starting motion server on http://localhost:${SERVER_PORT}"
echo "  robot=${ROBOT_IP}  domain=${ROS_DOMAIN_ID}  image=${IMAGE_TOPIC}"

# --- optional natural-language control ---------------------------------------
NL_ARGS=()
if [[ "${ENABLE_NL:-0}" == "1" ]]; then
  NL_ARGS=(--enable-nl
           --llm-url "${LLM_URL:-http://localhost:11434}"
           --llm-model "${LLM_MODEL:-qwen2.5:3b}"
           --nl-max-duration "${NL_MAX_DURATION:-10}"
           --nl-max-distance "${NL_MAX_DISTANCE:-2.0}"
           --nl-max-angle "${NL_MAX_ANGLE:-360}"
           --nl-max-steps "${NL_MAX_STEPS:-5}"
           --nl-max-chain-distance "${NL_MAX_CHAIN_DISTANCE:-3.0}"
           --nl-max-chain-angle "${NL_MAX_CHAIN_ANGLE:-720}"
           --nl-max-chain-seconds "${NL_MAX_CHAIN_SECONDS:-120}")
fi

# --- optional visual seek ("go to the bottle") --------------------------------
# Needs the lidar as well as the camera: vision supplies the bearing, /scan
# supplies the range it stops on.
SEEK_ARGS=()
if [[ "${ENABLE_SEEK:-0}" == "1" ]]; then
  SEEK_ARGS=(--enable-seek
             --scan-topic "${SCAN_TOPIC:-/scan}"
             --seek-weights "${SEEK_WEIGHTS:-yolov8m-world.pt}"
             --seek-conf "${SEEK_CONF:-0.15}"
             --seek-stop-distance "${SEEK_STOP_DISTANCE:-0.35}"
             --seek-max-travel "${SEEK_MAX_TRAVEL:-2.5}"
             --seek-search-step "${SEEK_SEARCH_STEP:-25}"
             --seek-search-max "${SEEK_SEARCH_MAX:-400}"
             --seek-timeout "${SEEK_TIMEOUT:-60}")
  [[ -n "${SEEK_DEVICE:-}" ]] && SEEK_ARGS+=(--seek-device "${SEEK_DEVICE}")
fi

exec python3 motion_server.py \
  --image-topic  "${IMAGE_TOPIC}" \
  --image-type   "${IMAGE_TYPE}" \
  --cmd-vel-topic "${CMD_VEL_TOPIC}" \
  --odom-topic "${ODOM_TOPIC:-/odom}" \
  --max-lin "${MAX_LIN}" \
  --max-ang "${MAX_ANG}" \
  --robot-ip "${ROBOT_IP}" \
  --port "${SERVER_PORT}" \
  --goal-timeout-max "${GOAL_TIMEOUT_MAX:-60}" \
  ${NL_ARGS[@]+"${NL_ARGS[@]}"} \
  ${SEEK_ARGS[@]+"${SEEK_ARGS[@]}"}

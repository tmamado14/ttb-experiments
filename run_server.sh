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
           --nl-max-duration "${NL_MAX_DURATION:-10}")
fi

exec python3 motion_server.py \
  --image-topic  "${IMAGE_TOPIC}" \
  --image-type   "${IMAGE_TYPE}" \
  --cmd-vel-topic "${CMD_VEL_TOPIC}" \
  --max-lin "${MAX_LIN}" \
  --max-ang "${MAX_ANG}" \
  --robot-ip "${ROBOT_IP}" \
  --port "${SERVER_PORT}" \
  "${NL_ARGS[@]}"

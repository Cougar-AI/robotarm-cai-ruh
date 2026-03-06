#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-}"
AUTO_MOVE=1
CLEAN_STALE=0
WORLD_PATH=""
WAIT_TIMEOUT="${WAIT_TIMEOUT:-120}"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Starts the BCR arm Gazebo demo and optionally sends trajectory commands.

Options:
  --ros-distro <name>   ROS distro to source (default: auto-detect)
  --world <path>        Gazebo world path for bcr_arm.gazebo.launch.py
  --no-auto-move        Start demo but do not send trajectory command
  --clean-stale         Kill stale Gazebo/ROS launch processes before start
  --help                Show this help text
USAGE
}

source_safe() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

resolve_ros_distro() {
  local setup_file

  if [[ -n "${ROS_DISTRO_NAME}" ]]; then
    return
  fi

  for distro in humble jazzy; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      ROS_DISTRO_NAME="${distro}"
      return
    fi
  done

  shopt -s nullglob
  for setup_file in /opt/ros/*/setup.bash; do
    ROS_DISTRO_NAME="$(basename "$(dirname "${setup_file}")")"
    shopt -u nullglob
    return
  done
  shopt -u nullglob
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_SETUP="${REPO_ROOT}/software/ws_ros/install/setup.bash"
LAUNCH_PID=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro)
      ROS_DISTRO_NAME="${2:-}"
      shift 2
      ;;
    --world)
      WORLD_PATH="${2:-}"
      shift 2
      ;;
    --no-auto-move)
      AUTO_MOVE=0
      shift
      ;;
    --clean-stale)
      CLEAN_STALE=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

resolve_ros_distro

if [[ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  echo "Missing ROS setup file under /opt/ros."
  echo "Install ROS 2 first, or pass --ros-distro <name>."
  exit 1
fi

if [[ -z "${WORLD_PATH}" ]]; then
  WORLD_PATH="/opt/ros/${ROS_DISTRO_NAME}/share/bcr_arm_gazebo/worlds/empty.world"
fi

source_safe "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ -f "${WS_SETUP}" ]]; then
  source_safe "${WS_SETUP}"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI not found after sourcing environment."
  exit 1
fi

if [[ "${CLEAN_STALE}" -eq 1 ]]; then
  pkill -f "ign gazebo" >/dev/null 2>&1 || true
  pkill -f "ros2 launch bcr_arm" >/dev/null 2>&1 || true
  sleep 1
fi

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -TERM "-${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -KILL "-${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

ENV_CMD="set +u; source /opt/ros/${ROS_DISTRO_NAME}/setup.bash; set -u"
if [[ -f "${WS_SETUP}" ]]; then
  ENV_CMD+=" && set +u; source '${WS_SETUP}'; set -u"
fi

LOG_FILE="/tmp/ros2_bcr_arm_demo_$(date +%Y%m%d_%H%M%S).log"
setsid bash -lc "${ENV_CMD} && ros2 launch bcr_arm_gazebo bcr_arm.gazebo.launch.py use_camera:=false world_path:='${WORLD_PATH}'" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

echo "Launch started. Log: ${LOG_FILE}"

ready=0
for ((i=1; i<=WAIT_TIMEOUT; i++)); do
  controllers_raw="$(timeout 2 ros2 control list_controllers 2>/dev/null || true)"
  controllers="$(printf '%s\n' "${controllers_raw}" | sed -r 's/\x1B\[[0-9;]*[A-Za-z]//g')"

  if echo "${controllers}" | grep -Eq 'joint_state_broadcaster.*active' && \
     echo "${controllers}" | grep -Eq 'joint_trajectory_controller.*active'; then
    ready=1
    echo "Controllers ready after ${i}s."
    break
  fi

  if (( i % 10 == 0 )); then
    echo "Waiting for controllers... (${i}s)"
  fi
  sleep 1
done

if [[ "${ready}" -ne 1 ]]; then
  echo "Timed out waiting for active controllers."
  echo "Check log: ${LOG_FILE}"
  exit 1
fi

echo "Current controllers:"
ros2 control list_controllers || true

if [[ "${AUTO_MOVE}" -eq 1 ]]; then
  action_ready=0
  for ((i=1; i<=WAIT_TIMEOUT; i++)); do
    if ros2 action list 2>/dev/null | grep -q '^/joint_trajectory_controller/follow_joint_trajectory$'; then
      action_ready=1
      echo "Trajectory action server ready after ${i}s."
      break
    fi

    if (( i % 10 == 0 )); then
      echo "Waiting for trajectory action server... (${i}s)"
    fi
    sleep 1
  done

  if [[ "${action_ready}" -ne 1 ]]; then
    echo "Timed out waiting for trajectory action server."
    echo "Check log: ${LOG_FILE}"
    exit 1
  fi

  echo "Sending demo trajectory..."
  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    "{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6','joint7'], points: [{positions: [0.1,-0.2,0.25,-0.15,0.1,-0.1,0.05], time_from_start: {sec: 3}}]}}"

  echo "Sending return trajectory..."
  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    "{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6','joint7'], points: [{positions: [0.0,0.0,0.1,0.0,0.0,0.0,0.0], time_from_start: {sec: 3}}]}}"
fi

echo "Demo is running. Press Ctrl+C to stop Gazebo and all child processes."
wait "${LAUNCH_PID}"

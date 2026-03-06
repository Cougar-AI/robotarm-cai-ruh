#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-}"
UR_TYPE="ur3"
WORLD_FILE="empty.sdf"
WITH_MOVEIT=1
AUTO_TRAJECTORY=1
SPAWN_ASSETS=1
GAZEBO_GUI=1
CLEAN_STALE=0
WAIT_TIMEOUT="${WAIT_TIMEOUT:-120}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ASSET_DIR="${SCRIPT_DIR}/assets/models"
WS_SETUP="${REPO_ROOT}/software/ws_ros/install/setup.bash"

SIM_PGID=""
MOVEIT_PGID=""
SIM_LOG=""
MOVEIT_LOG=""
MOVEIT_READY=0

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Launch a UR robot (default UR3) in Gazebo, optionally launch MoveIt, spawn demo
scene assets (table/bins/cube), and run a pick/place-style trajectory sequence.

Options:
  --ur-type <type>      UR type (default: ur3)
  --world-file <path>   Gazebo world file (default: empty.sdf)
  --no-moveit           Skip MoveIt launch
  --no-auto-trajectory  Skip automatic trajectory sequence
  --no-assets           Do not spawn scene assets
  --headless            Start Gazebo server only (no GUI)
  --clean-stale         Kill stale Gazebo/ROS launch processes first
  --ros-distro <name>   ROS distro to source (default: auto-detect)
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

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ur-type)
      UR_TYPE="${2:-}"
      shift 2
      ;;
    --world-file)
      WORLD_FILE="${2:-}"
      shift 2
      ;;
    --no-moveit)
      WITH_MOVEIT=0
      shift
      ;;
    --no-auto-trajectory)
      AUTO_TRAJECTORY=0
      shift
      ;;
    --no-assets)
      SPAWN_ASSETS=0
      shift
      ;;
    --headless)
      GAZEBO_GUI=0
      shift
      ;;
    --clean-stale)
      CLEAN_STALE=1
      shift
      ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:-}"
      shift 2
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

source_safe "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ -f "${WS_SETUP}" ]]; then
  source_safe "${WS_SETUP}"
fi

if [[ "${CLEAN_STALE}" -eq 1 ]]; then
  pkill -f "ign gazebo" >/dev/null 2>&1 || true
  pkill -f "ros2 launch ur_simulation_gz" >/dev/null 2>&1 || true
  pkill -f "ros2 launch ur_moveit_config" >/dev/null 2>&1 || true
  sleep 1
fi

cleanup() {
  if [[ -n "${MOVEIT_PGID}" ]]; then
    kill -TERM "-${MOVEIT_PGID}" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL "-${MOVEIT_PGID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${SIM_PGID}" ]]; then
    kill -TERM "-${SIM_PGID}" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL "-${SIM_PGID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

startup_failed() {
  local log_file="$1"

  [[ -f "${log_file}" ]] || return 1

  if grep -Eq 'undefined symbol:|Failed to load system plugin|No plugins detected in library' "${log_file}"; then
    return 0
  fi

  return 1
}

SIM_LOG="/tmp/ur3_sim_$(date +%Y%m%d_%H%M%S).log"
MOVEIT_LOG="/tmp/ur3_moveit_$(date +%Y%m%d_%H%M%S).log"

ENV_CMD="set +u; source /opt/ros/${ROS_DISTRO_NAME}/setup.bash; set -u"
if [[ -f "${WS_SETUP}" ]]; then
  ENV_CMD+=" && set +u; source '${WS_SETUP}'; set -u"
fi

setsid bash -lc "${ENV_CMD} && ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=${UR_TYPE} launch_rviz:=false gazebo_gui:=$([[ ${GAZEBO_GUI} -eq 1 ]] && echo true || echo false) world_file:='${WORLD_FILE}'" >"${SIM_LOG}" 2>&1 &
SIM_PGID=$!

echo "UR simulation log: ${SIM_LOG}"

controllers_ready=0
for ((i=1; i<=WAIT_TIMEOUT; i++)); do
  controllers_raw="$(timeout 2 ros2 control list_controllers 2>/dev/null || true)"
  controllers="$(printf '%s\n' "${controllers_raw}" | sed -r 's/\x1B\[[0-9;]*[A-Za-z]//g')"

  if echo "${controllers}" | grep -Eq 'joint_state_broadcaster.*active' && \
     echo "${controllers}" | grep -Eq 'joint_trajectory_controller.*active'; then
    controllers_ready=1
    echo "Controllers ready after ${i}s."
    break
  fi

  if (( i % 10 == 0 )); then
    echo "Waiting for UR controllers... (${i}s)"
  fi

  if startup_failed "${SIM_LOG}"; then
    echo "UR simulation startup failed before controllers became ready."
    echo "A Gazebo or ros2_control plugin failed to load. Check ${SIM_LOG}"
    exit 1
  fi

  sleep 1
done

if [[ "${controllers_ready}" -ne 1 ]]; then
  echo "Timed out waiting for UR controllers."
  echo "Check ${SIM_LOG}"
  exit 1
fi

spawn_asset() {
  local file="$1"
  local name="$2"
  local x="$3"
  local y="$4"
  local z="$5"
  local yaw="$6"

  ros2 run ros_gz_sim create \
    -file "${file}" \
    -name "${name}" \
    -allow_renaming true \
    -x "${x}" -y "${y}" -z "${z}" -Y "${yaw}" >/dev/null 2>&1
}

if [[ "${SPAWN_ASSETS}" -eq 1 ]]; then
  if [[ -d "${ASSET_DIR}" ]]; then
    echo "Spawning demo scene assets..."
    spawn_asset "${ASSET_DIR}/table.sdf" "demo_table" 0.58 0.0 0.225 0.0 || true
    spawn_asset "${ASSET_DIR}/bin.sdf" "demo_bin_left" 0.42 0.23 0.50 0.0 || true
    spawn_asset "${ASSET_DIR}/bin.sdf" "demo_bin_right" 0.42 -0.23 0.50 0.0 || true
    spawn_asset "${ASSET_DIR}/pick_cube.sdf" "demo_pick_cube" 0.45 0.18 0.473 0.0 || true
    spawn_asset "${ASSET_DIR}/place_target.sdf" "demo_place_target" 0.45 -0.18 0.455 0.0 || true
  else
    echo "Asset directory not found: ${ASSET_DIR}"
  fi
fi

if [[ "${WITH_MOVEIT}" -eq 1 ]]; then
  setsid bash -lc "${ENV_CMD} && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=${UR_TYPE} use_sim_time:=true launch_rviz:=true" >"${MOVEIT_LOG}" 2>&1 &
  MOVEIT_PGID=$!
  echo "MoveIt log: ${MOVEIT_LOG}"

  for ((i=1; i<=WAIT_TIMEOUT; i++)); do
    if grep -q "You can start planning now" "${MOVEIT_LOG}" 2>/dev/null; then
      MOVEIT_READY=1
      echo "MoveIt ready after ${i}s."
      break
    fi
    if (( i % 10 == 0 )); then
      echo "Waiting for MoveIt... (${i}s)"
    fi
    sleep 1
  done

  if [[ "${MOVEIT_READY}" -ne 1 ]]; then
    echo "MoveIt did not report readiness before timeout."
    echo "Check ${MOVEIT_LOG}"
    AUTO_TRAJECTORY=0
  fi
fi

# If GUI was closed manually before trajectory execution, skip automatic motion.
if [[ "${GAZEBO_GUI}" -eq 1 ]] && ! pgrep -f "ign gazebo gui" >/dev/null 2>&1; then
  echo "Gazebo GUI is not running; skipping auto trajectory."
  AUTO_TRAJECTORY=0
fi

send_joint_goal() {
  local positions="$1"
  local duration="$2"

  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    "{trajectory: {joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'], points: [{positions: [${positions}], time_from_start: {sec: ${duration}}}]}}" >/dev/null
}

if [[ "${AUTO_TRAJECTORY}" -eq 1 ]]; then
  action_ready=0
  for ((i=1; i<=WAIT_TIMEOUT; i++)); do
    if ros2 action list 2>/dev/null | grep -q '^/joint_trajectory_controller/follow_joint_trajectory$'; then
      action_ready=1
      break
    fi
    sleep 1
  done

  if [[ "${action_ready}" -eq 1 ]]; then
    echo "Running pick/place-style joint sequence..."
    send_joint_goal "0.00,-1.57,1.57,-1.57,-1.57,0.00" 3
    send_joint_goal "0.35,-1.20,1.45,-1.80,-1.57,0.15" 3
    send_joint_goal "0.35,-1.05,1.20,-1.65,-1.57,0.15" 3
    send_joint_goal "-0.35,-1.10,1.40,-1.80,-1.57,-0.20" 4
    send_joint_goal "0.00,-1.57,1.57,-1.57,-1.57,0.00" 3
    echo "Trajectory sequence complete."
  else
    echo "Trajectory action server not ready; skipping auto trajectory."
  fi
fi

echo "UR demo is running. Press Ctrl+C to stop simulation and MoveIt."
wait "${SIM_PGID}"

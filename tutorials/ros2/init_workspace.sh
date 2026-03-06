#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-}"
BUILD_WS=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="${REPO_ROOT}/software/ws_ros"
SRC_DIR="${WS_DIR}/src"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Initialize a general-purpose ROS 2 workspace layout for a custom robot arm.

Options:
  --ros-distro <name>   ROS distro to source (default: auto-detect)
  --ws-dir <path>       Workspace directory (default: software/ws_ros)
  --build               Run colcon build after creating packages
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
    --ros-distro)
      ROS_DISTRO_NAME="${2:-}"
      shift 2
      ;;
    --ws-dir)
      WS_DIR="${2:-}"
      SRC_DIR="${WS_DIR}/src"
      shift 2
      ;;
    --build)
      BUILD_WS=1
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

source_safe "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

mkdir -p "${SRC_DIR}"

create_pkg_if_missing() {
  local pkg_name="$1"
  local pkg_path="${SRC_DIR}/${pkg_name}"

  if [[ -f "${pkg_path}/package.xml" ]]; then
    echo "Package exists: ${pkg_name}"
    return
  fi

  echo "Creating package: ${pkg_name}"
  (cd "${SRC_DIR}" && ros2 pkg create --build-type ament_cmake "${pkg_name}")
}

create_pkg_if_missing "custom_arm_description"
create_pkg_if_missing "custom_arm_gazebo"
create_pkg_if_missing "custom_arm_moveit_config"
create_pkg_if_missing "custom_arm_bringup"

mkdir -p "${SRC_DIR}/custom_arm_description/urdf" \
         "${SRC_DIR}/custom_arm_description/meshes" \
         "${SRC_DIR}/custom_arm_description/config" \
         "${SRC_DIR}/custom_arm_description/launch"

mkdir -p "${SRC_DIR}/custom_arm_gazebo/worlds" \
         "${SRC_DIR}/custom_arm_gazebo/models" \
         "${SRC_DIR}/custom_arm_gazebo/config" \
         "${SRC_DIR}/custom_arm_gazebo/launch"

mkdir -p "${SRC_DIR}/custom_arm_moveit_config/config" \
         "${SRC_DIR}/custom_arm_moveit_config/srdf" \
         "${SRC_DIR}/custom_arm_moveit_config/launch"

mkdir -p "${SRC_DIR}/custom_arm_bringup/launch" \
         "${SRC_DIR}/custom_arm_bringup/config" \
         "${SRC_DIR}/custom_arm_bringup/scripts"

if [[ ! -f "${SRC_DIR}/custom_arm_description/urdf/custom_arm.urdf.xacro" ]]; then
  cat > "${SRC_DIR}/custom_arm_description/urdf/custom_arm.urdf.xacro" <<'XEOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="custom_arm">
  <!-- Replace this placeholder with your full custom robot arm URDF/Xacro. -->
  <link name="base_link"/>
</robot>
XEOF
fi

if [[ ! -f "${SRC_DIR}/custom_arm_gazebo/worlds/custom_arm_demo.sdf" ]]; then
  cat > "${SRC_DIR}/custom_arm_gazebo/worlds/custom_arm_demo.sdf" <<'XEOF'
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="default">
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground%20Plane</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>
  </world>
</sdf>
XEOF
fi

cat > "${WS_DIR}/WORKSPACE_LAYOUT.md" <<'XEOF'
# Workspace Layout

This workspace is initialized for custom arm simulation and planning.

Packages:
- custom_arm_description: URDF/Xacro, meshes, robot description launch files
- custom_arm_gazebo: Gazebo worlds, model assets, sim launch files
- custom_arm_moveit_config: MoveIt SRDF and planning configs
- custom_arm_bringup: top-level launch files and runtime configuration

Recommended flow:
1) Drop your final URDF/Xacro into custom_arm_description/urdf
2) Add ros2_control + Gazebo plugins to your description
3) Build MoveIt config for your robot (MoveIt Setup Assistant)
4) Create integrated launch files in custom_arm_bringup/launch
XEOF

if [[ "${BUILD_WS}" -eq 1 ]]; then
  echo "Building workspace: ${WS_DIR}"
  (cd "${WS_DIR}" && colcon build --symlink-install)
fi

echo "Workspace initialized at: ${WS_DIR}"
echo "Next: edit custom_arm_description/urdf/custom_arm.urdf.xacro"
echo "To source workspace after build: source ${WS_DIR}/install/setup.bash"

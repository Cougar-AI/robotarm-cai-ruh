# ROS 2 + Gazebo on Jetson (tutorial)

This tutorial gives you a reusable ROS workspace pattern for robot-arm simulation and planning, then demonstrates it with a UR3 Gazebo + MoveIt scene.

Important distro note:
1) `Ubuntu 22.04 (jammy)` -> use `ROS 2 Humble`
2) `Ubuntu 24.04 (noble)` -> use `ROS 2 Jazzy`
3) If you are on WSL2 and `apt update` shows `noble`, do not try to install `ros-humble-*` packages.

##### References: [ROS 2](https://docs.ros.org/en/humble/index.html); [Gazebo Sim](https://gazebosim.org/docs); [MoveIt 2](https://moveit.picknik.ai/main/index.html); [UR ROS 2](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver); [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install)

> Platform status
> 1) Verified in this session: Linux Jetson (`Ubuntu 22.04`, `arm64`) on `February 18, 2026`
> 2) Planned for separate verification session: Windows + WSL2
> 3) macOS: not working in our experience for this path right now

---

## Why a ROS Workspace Exists

A ROS workspace (like `software/ws_ros`) is the place where your **project-specific packages** live and are built as one overlay.

Core purpose:
1) Keep your robot code separate from `/opt/ros` system packages.
2) Build your description, simulation, control, and planning packages together (`colcon build`).
3) Overlay custom packages on top of vendor packages (`source install/setup.bash`).
4) Version your package sources while ignoring build artifacts.

For a robot arm, the usual package split is:
1) `*_description` (URDF/Xacro + meshes)
2) `*_gazebo` (worlds + sim config + ros2_control config)
3) `*_moveit_config` (SRDF + planning config)
4) `*_bringup` (top-level launch orchestration)

---

## Outcome

After this guide, you will have:
1) ROS 2 Humble or Jazzy + Gazebo + MoveIt installed,
2) a general workspace initializer for your future custom URDF,
3) a UR3 demo launcher with scene assets and trajectory sequence,
4) a working path for planning + simulation workflows.

---

# A) Linux Jetson Guide (fully explicit)

## A.1 Verify OS and architecture

```bash
lsb_release -a
dpkg --print-architecture
uname -a
```

Expected for the Jetson path: Ubuntu `jammy` and `arm64`.

If you are on WSL2, it is common to see Ubuntu `noble` and `amd64`. In that case, use the `jazzy` package names in the next step.

## A.2 Install base dependencies + locale

```bash
sudo apt update
sudo apt install -y locales curl gnupg2 lsb-release software-properties-common ca-certificates
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

## A.3 Add ROS 2 apt key and source

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## A.4 Install ROS 2 + Gazebo + MoveIt + UR + BCR stacks

Ubuntu `22.04` / `jammy`:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-vcstool \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-sim-demos \
  ros-humble-gz-ros2-control \
  ros-humble-gz-ros2-control-demos \
  ros-humble-moveit \
  ros-humble-moveit-task-constructor-demo \
  ros-humble-ur \
  ros-humble-ur-simulation-gz \
  ros-humble-ur-moveit-config \
  ros-humble-bcr-arm
```

Ubuntu `24.04` / `noble` (typical for newer WSL2 installs):

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-vcstool \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-sim-demos \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-ros2-control-demos \
  ros-jazzy-moveit \
  ros-jazzy-moveit-task-constructor-demo \
  ros-jazzy-ur \
  ros-jazzy-ur-simulation-gz \
  ros-jazzy-ur-moveit-config \
  ros-jazzy-bcr-arm
```

## A.5 Source ROS and create workspace path

```bash
echo "source /opt/ros/<humble-or-jazzy>/setup.bash" >> ~/.bashrc
source ~/.bashrc
mkdir -p ~/robotarm-cai-ruh/software/ws_ros/src
```

## A.6 Validate install

```bash
echo "$ROS_DISTRO"
command -v ros2
command -v ign
ros2 pkg list | grep -E 'ur_simulation_gz|ur_moveit_config|bcr_arm_gazebo|gz_ros2_control'
```

---

# B) Initialize a General Workspace (for your custom URDF)

This creates a standard robot-arm package layout under `software/ws_ros/src`.

## B.1 Run workspace initializer

```bash
bash tutorials/ros2/init_workspace.sh
```

Optional build immediately:

```bash
bash tutorials/ros2/init_workspace.sh --build
```

If you want to force a specific distro, pass it explicitly:

```bash
bash tutorials/ros2/init_workspace.sh --ros-distro jazzy
```

What it creates:
1) `custom_arm_description`
2) `custom_arm_gazebo`
3) `custom_arm_moveit_config`
4) `custom_arm_bringup`

## B.2 Start integrating your custom robot

```bash
# 1) Edit your robot model
$EDITOR software/ws_ros/src/custom_arm_description/urdf/custom_arm.urdf.xacro

# 2) Build workspace
cd software/ws_ros
colcon build --symlink-install

# 3) Source overlay
source install/setup.bash
```

---

# C) UR3 Example: Simulation + Planning + Scene Assets

This is the “rich demo” path you asked for: UR3 in Gazebo, scene objects, and trajectory motion.

## C.1 Run UR3 demo script

```bash
bash tutorials/ros2/ur3_demo.sh --clean-stale
```

On Ubuntu `24.04`, you can also pass the distro explicitly:

```bash
bash tutorials/ros2/ur3_demo.sh --ros-distro jazzy --clean-stale
```

What this script does:
1) Launches `ur_simulation_gz` with `ur_type:=ur3`
2) Waits for active controllers
3) Spawns scene assets from `tutorials/ros2/assets/models/`
4) Launches MoveIt (`ur_moveit_config`)
5) Sends a pick/place-style joint trajectory sequence

## C.2 Useful flags

```bash
bash tutorials/ros2/ur3_demo.sh --help
bash tutorials/ros2/ur3_demo.sh --no-moveit
bash tutorials/ros2/ur3_demo.sh --no-auto-trajectory
bash tutorials/ros2/ur3_demo.sh --headless
bash tutorials/ros2/ur3_demo.sh --ur-type ur3e
```

## C.3 Notes about pick and place

Current script is a **pick/place-style trajectory demo** in a staged scene.
For true grasp/attach/place logic, add one of:
1) gripper model + grasp controller,
2) MoveIt Task Constructor pipeline,
3) contact/attach plugin workflow.

---

# D) Existing BCR Arm Demo

You still have the BCR-focused launcher:

```bash
bash tutorials/ros2/setup.sh --clean-stale
```

Or, explicitly on Ubuntu `24.04`:

```bash
bash tutorials/ros2/setup.sh --ros-distro jazzy --clean-stale
```

This remains useful for quick verification on this machine.

---

# E) Windows 11 + WSL2 (planned, not yet verified here)

PowerShell (Administrator):

```powershell
wsl --install -d Ubuntu
wsl --update
```

Then run the same Linux commands from sections `A` through `D` inside WSL Ubuntu, but choose the ROS distro that matches your Ubuntu release:
1) WSL Ubuntu `22.04` -> `humble`
2) WSL Ubuntu `24.04` -> `jazzy`

---

# F) macOS status

As of `February 18, 2026`, this ROS2 + Gazebo workflow is not reliably working on macOS in our experience.

---

## Troubleshooting

1) Duplicate `/controller_manager` or duplicate action servers  
Cause: stale Gazebo/ROS processes.  
Fix:

```bash
pkill -f "ign gazebo" || true
pkill -f "ros2 launch ur_simulation_gz" || true
pkill -f "ros2 launch bcr_arm" || true
```

2) `No 3D sensor plugin(s) defined for octomap updates` in MoveIt  
Non-blocking for basic planning and trajectory execution demos.

3) EGL / `nvidia-drm` warnings in Gazebo GUI  
Common on this Jetson setup and usually non-blocking for simulation/control.

# MuJoCo ROS2 Jazzy Integration Tutorial
This tutorial launches a simple cart visualizer in ROS2 + MuJoCo focusing on development environment setup. [Credits to GitHub user: `rxdu`](#credit) and their repo [`mujoco_sim_ros2`](https://github.com/rxdu/mujoco_sim_ros2/tree/devel).
>Note: macOS and Linux not verified, only WSL is confirmed to work.
>
Specific versions tested
* Ubuntu 24.04
* ROS2 Jazzy
* Mujoco 3.3.0
## Pre-setup directories
```bash
# Choose a directory where you want to keep your project info
# I choose to make my project in my home `~` directory
cd ~

# Clone official project directory
git clone https://github.com/Cougar-AI/robotarm-cai-ruh.git

# Change directory `cd` into MuJoCo workspace
cd robotarm-cai-ruh/software/ws_mjc
```
## Download ROS2
Linux/macOS/Windows_Subsytem_for_Linux (WSL)
```bash
# Locale specification (some ROS builds fail w/o this)
sudo apt update && sudo apt install -y locales curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Common build tools for ROS install
sudo apt install -y software-properties-common

# Security keys allowing streamlined OS-to-ROS2 install
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

# Jazzy desktop + build tools (similar to make/cmake) + common ROS2 build dependencies
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool git

# rosdep tool for ROS2 dependencies
sudo rosdep init || true
rosdep update

# Auto-source (persists across cli/terminal sessions)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## MuJoCo ROS2 Additional Dependencies
```bash
# GLFW3, mesa, egl/gl renderer dependency (needed later)
sudo apt install -y libglfw3-dev libgl1-mesa-dev libegl1-mesa-dev

# MuJoCo build from source dependencies e.g. CMake
sudo apt install -y build-essential cmake ninja-build pkg-config

# Install ROS2 package: MoveIt
sudo apt install -y ros-jazzy-moveit

# Install ROS2 control + controller packages
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# Ensure the needed Python modules for ROS via apt (system Python)
sudo apt install -y python3-catkin-pkg-modules python3-empy
```
```bash
# Verify the system Python and modules
python3 -c "import sys; print(sys.executable)"
python3 -c "import catkin_pkg, em; print('ROS python deps OK')"

# Should be /usr/bin/python3 and python 3.12.x
which python3; python3 -V

# Append system python to PATH, first before any .local install
export PATH=/usr/bin:$PATH
```

### Make PATH + system Python persist

Run this once. It (a) removes any old lines we might have added before, (b) inserts a tiny guarded block in `~/.bashrc` that always puts `/usr/bin` first, **before** `~/.local/bin`, conda, etc., and (c) reloads your shell.

```bash
# 1) Remove any previous PATH guards we added
sed -i '/### BEGIN SYSTEM_PYTHON_FIRST ###/,/### END SYSTEM_PYTHON_FIRST ###/d' ~/.bashrc

# 2) Add an idempotent PATH guard to ~/.bashrc
cat >> ~/.bashrc <<'EOF'
### BEGIN SYSTEM_PYTHON_FIRST ###
# Always prefer system Python (and tools like catkin_pkg from apt)
# Put /usr/bin at the very front of PATH so ~/.local or conda won't override it.
case ":$PATH:" in
  *:/usr/bin:*) ;;                     # already somewhere in PATH
  *) export PATH="/usr/bin:$PATH" ;;   # prepend if missing
esac
### END SYSTEM_PYTHON_FIRST ###
EOF

# 3) Apply the change now
source ~/.bashrc

# 4) Sanity checks
which python3; python3 -V
python3 -c 'import sys,catkin_pkg,em; print("catkin_pkg:",catkin_pkg.__file__); print("em:",em.__file__); print("OK")'
```

If the last line points to `/usr/lib/python3/dist-packages/...`, you’re golden.
If it points into `~/.local/lib/...`, nuke the user installs:

```bash
python3 -m pip uninstall -y catkin_pkg empy || true
sudo apt install -y python3-catkin-pkg-modules python3-empy
```

## Setup mujoco control project

```bash
# Print current working directory
pwd
# Analyze, navigate to robotarm-cai-ruh/software/ws_mjc/src

# If following guide, should be in ~/robotarm-cai-ruh/software/ws_mjc, therefore:
cd ~/robotarm-cai-ruh/software/ws_mjc/src

# Clone repo with MuJoCo + jazzy + ROS2 tested with demo
git clone -b 3.3.0 https://github.com/google-deepmind/mujoco.git
git clone https://github.com/rxdu/mujoco_sim_ros2.git
git clone https://github.com/rxdu/mujoco_ros2_control.git
git clone https://github.com/rxdu/mujoco_demo_robot.git

# Navigate out of source, into ws_mjc directory
cd ..

# Build with colcon
colcon build --symlink-install
```
Ignore the warnings, they are not critical.
## Run demo
```bash
# Use ros2 for demo in ws_mjc/src/mujoco_demo_robot/launch/cart_effort.launch.py
ros2 launch mujoco_demo_robot cart_effort.launch.py
```
# If the launch fails and you may need to source setup.bash
```bash
#source setup.bash
source ~/robotarm-cai-ruh/software/ws_mjc/install/setup.bash

#Then launch again
ros2 launch mujoco_demo_robot cart_effort.launch.py

```
## Credit

This tutorial heavily follows and builds upon the excellent setup and examples from **[rxdu/mujoco_sim_ros2](https://github.com/rxdu/mujoco_sim_ros2)** (and related repos **mujoco_ros2_control** and **mujoco_demo_robot**). Big thanks to that project for a clean integration of the MuJoCo [`simulate`](https://github.com/google-deepmind/mujoco/tree/main/simulate) app with ROS 2 Jazzy [pluginlib](https://github.com/ros/pluginlib).

## Troubleshooting tips
### Keep system Python first (prevents “catkin_pkg not found” & similar)

To avoid accidental overrides from `~/.local/bin` or conda, ensure `/usr/bin` is **first** on your PATH every shell:

```bash
# Append system python to PATH, first before any .local install
# (Persisted via ~/.bashrc)
case ":$PATH:" in
  *:/usr/bin:*) ;; 
  *) export PATH="/usr/bin:$PATH" ;;
esac
```

> Tip: If you ever see Python modules coming from `~/.local/...`, remove those user installs:
> `python3 -m pip uninstall -y catkin_pkg empy`
> and (re)install the apt versions:
> `sudo apt install -y python3-catkin-pkg-modules python3-empy`.

### Quick verification before building

```bash
# Should be /usr/bin/python3 and 3.12.x on Ubuntu 24.04
which python3 && python3 -V

# Make sure ROS Python deps come from the system packages
python3 -c 'import catkin_pkg, em; print("catkin_pkg:",catkin_pkg.__file__); print("em:",em.__file__)'
```

### Build & run (recap)

```bash
cd ~/robotarm-cai-ruh/software/ws_mjc/src
git clone -b 3.3.0 https://github.com/google-deepmind/mujoco.git
git clone https://github.com/rxdu/mujoco_sim_ros2.git
git clone https://github.com/rxdu/mujoco_ros2_control.git
git clone https://github.com/rxdu/mujoco_demo_robot.git
cd ..
colcon build --symlink-install
source install/setup.bash
ros2 launch mujoco_demo_robot cart_effort.launch.py
```

### WSLg-specific notes:
```bash
# (Optional) Enforce WSLg GPU acceleration
export GALLIUM_DRIVER=d3d12
# (Optional) Make this change persist across sessions
echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
source ~/.bashrc

# Verify GPU acceleration (should print d3d12)
echo $GALLIUM_DRIVER

# (Optional) sanity check: should say “D3D12 (…GPU…)” and “Accelerated: yes”
glxinfo -B | sed -n '1,20p'

# (Optional) If dedicated GPU [dGPU] needed over integrated GPU [iGPU] (e.g. NVIDIA over Intel/AMD/etc.), adjust adapter name:
export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"

# Persist over time
echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"' >> ~/.bashrc
source ~/.bashrc
```
* Make sure you didn’t start from a conda shell (`conda deactivate`) and that `which python3` is `/usr/bin/python3`.

### Frequent problems

* **`ModuleNotFoundError: catkin_pkg`** – your Python is coming from `~/.local` or conda. Use the **Keep system Python first** block above and install `python3-catkin-pkg-modules` via apt.
* **Controller load warnings** – install/control stack:

  ```bash
  sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
                      ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster \
                      ros-jazzy-joint-trajectory-controller
  ```
* **Wrong MuJoCo version** – this tutorial assumes **MuJoCo 3.3.0** (the repo clone pins it). Don’t mix 3.3.7 or any other versions headers/libs.
---
## TUTORIAL ENDS HERE
### Developer notes for maintainers of this tutorial
#### Build MuJoCo from source
Too complicated and didn't work
```bash
# MuJoCo directory environment variables (needed later) to build from source
MJ_VERSION="3.3.0"
MJ_DIR_NAME="mujoco-${MJ_VERSION}"
MUJOCO_DIR="${HOME}/mujoco/${MJ_DIR_NAME}"

# Auto-activate (Persist across sessions)
echo "Setting MUJOCO_DIR in ~/.bashrc..."
# Remove old MUJOCO_DIR lines if they exist to avoid duplicates
sed -i '/export MUJOCO_DIR/d' ~/.bashrc
    
# Add the new line
echo "export MUJOCO_DIR=${MUJOCO_DIR}" >> ~/.bashrc
    
# Export for the current session
export MUJOCO_DIR=${MUJOCO_DIR}
echo "MUJOCO_DIR set to ${MUJOCO_DIR}"

# Navigate to home directory
cd ~

# Clone mujoco repo
git clone https://github.com/google-deepmind/mujoco.git

# Navigate into repo
cd mujoco

# Choose version
git checkout 3.3.0

# Build with CMake
# Setup build
cmake -S . -B build -G Ninja   -DCMAKE_BUILD_TYPE=Release   -DCMAKE_INSTALL_PREFIX="$MUJOCO_DIR"   -DBUILD_TESTING=OFF
# Build and compile
cmake --build build -j"$(nproc)"
cmake --install build
```
Using incompatible version of MuJoCo (3.3.7)
```bash
# Old MuJoCo system binary install
# MuJoCo dependency (needed later)
# This script will detect your OS/Arch and download the correct MuJoCo system binaries
MJ_VERSION="3.3.7"
BASE_URL="https://github.com/google-deepmind/mujoco/releases/download/${MJ_VERSION}"
MJ_FILE=""
MJ_DIR_NAME="mujoco-${MJ_VERSION}"

OS_NAME=$(uname -s)
ARCH_NAME=$(uname -m)

if [ "$OS_NAME" = "Linux" ]; then
    echo "Detected Linux OS."
    if [ "$ARCH_NAME" = "x86_64" ]; then
        echo "Detected x86_64 architecture."
        MJ_FILE="mujoco-${MJ_VERSION}-linux-x86_64.tar.gz"
    elif [ "$ARCH_NAME" = "aarch64" ] || [ "$ARCH_NAME" = "arm64" ]; then
        echo "Detected ARM (aarch64) architecture."
        MJ_FILE="mujoco-${MJ_VERSION}-linux-aarch64.tar.gz"
    else
        echo "Unsupported Linux architecture: $ARCH_NAME"
        exit 1
    fi
elif [ "$OS_NAME" = "Darwin" ]; then
    echo "Detected macOS."
    MJ_FILE="mujoco-${MJ_VERSION}-macos-universal2.dmg"
else
    echo "Unsupported OS: $OS_NAME"
    exit 1
fi

if [ -z "$MJ_FILE" ]; then
    echo "Could not determine MuJoCo download file."
    exit 1
fi

# Download the file
echo "Downloading $MJ_FILE..."
curl -L "${BASE_URL}/${MJ_FILE}" -o "${MJ_FILE}"

# Extract and set up
if [[ "$MJ_FILE" == *.tar.gz ]]; then
    echo "Extracting..."
    mkdir -p ~/mujoco
    tar -xzf "${MJ_FILE}" -C ~/mujoco
    rm "${MJ_FILE}"
    
    FINAL_MJ_PATH="${HOME}/mujoco/${MJ_DIR_NAME}"
    echo "MuJoCo extracted to ${FINAL_MJ_PATH}"
    
    echo "Setting MUJOCO_DIR in ~/.bashrc..."
    # Remove old MUJOCO_DIR lines if they exist to avoid duplicates
    sed -i '/export MUJOCO_DIR/d' ~/.bashrc
    
    # Add the new line
    echo "export MUJOCO_DIR=${FINAL_MJ_PATH}" >> ~/.bashrc
    
    # Export for the current session
    export MUJOCO_DIR=${FINAL_MJ_PATH}
    echo "MUJOCO_DIR set to ${MUJOCO_DIR}"

elif [[ "$MJ_FILE" == *.dmg ]]; then
    echo "Download complete: ${MJ_FILE}"
    echo "ACTION REQUIRED: Please open the DMG file and install MuJoCo."
    echo "You must then manually set the MUJOCO_DIR variable."
    echo "Example for .bashrc or .zshrc:"
    echo '# export MUJOCO_DIR="/Applications/MuJoCo.app/Contents/Frameworks/MuJoCo.framework"'
fi
```

# Unitree G1 Description (URDF & MJCF) — End‑to‑End CLI Tutorial

This README takes you from a **clean macOS, Linux, or Windows (WSL)** install to **visualizing the Unitree G1 robot** in the MuJoCo viewer. The approach is **command‑line first**, uses **uv** for Python environments/deps, and **Visual Studio Code** for editing and an integrated terminal. On Windows we use **WSLg** so Linux GUIs open like native apps.

> **Outcome**  
> After completing the steps for your OS, you will:
> 1) Have **uv** installed and a `robotsim` virtual environment created,  
> 2) Have MuJoCo and its dependencies installed,  
> 3) Have Visual Studio Code set up with its integrated terminal,  
> 4) Have cloned `unitree_ros`, navigated to `robots/g1_description`, and  
> 5) Launched `mujoco.viewer` or `open_model.py` to inspect G1 URDF/MJCF files.

---

## Quickstart (experienced users)

```bash
# macOS / Linux / Windows (inside WSL Ubuntu)
# 1) Install uv: https://docs.astral.sh/uv/getting-started/installation/
curl -LsSf https://astral.sh/uv/install.sh | sh

# 2) Create a project folder and a Python 3.11 venv named robotsim:
mkdir -p ~/robot-viewer && cd ~/robot-viewer
uv python install 3.11
uv venv robotsim
source robotsim/bin/activate  # macOS/Linux/WSL
# PowerShell if using native Windows (not recommended for this tutorial):
# robotsim\Scripts\Activate.ps1

# 3) Add deps:
uv pip install mujoco glfw numpy pyopengl

# 4) Get the models and print path to .urdf/.xml:
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots/g1_description
pwd

# (Optional) Windows/WSL custom viewer script and vars needed.
# See section C.4 to create open_model.py. Drag-and-drop not allowed.
# export GALLIUM_DRIVER=d3d12
# uv run python ~/robot-viewer/open_model.py <path/to/model.xml> 

# 5) (macOS/Linux) Run the viewer (drag/drop a .xml or .urdf into it):
python -m mujoco.viewer
```

If you’re new to uv/WSL/CLI, follow the full, OS‑specific steps below.

---

## Approach and Tools

- **CLI-first**: Everything is done from the terminal to make the setup deterministic and repeatable.
- **uv**: Modern, fast, and reproducible Python env/packaging tool.
- **VS Code**: Used for code editing and its **Integrated Terminal**, so all commands are run consistently.
- **WSL (Windows)**: On Windows, we install Ubuntu under WSL with built-in GUI support (WSLg) so `mujoco.viewer` opens like a native app.

> **Why uv vs Conda?**  
> Conda’s C++ runtime can override system C++ runtime, breaking window creation with WSLg. See **Appendix D** for a patched conda + WSL environment.

---

# A) macOS Guide (Apple Silicon or Intel)

### A.1 Install Homebrew (optional but recommended)

Open the **Terminal** app and run the official installer:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```
Follow any on-screen instructions to add Homebrew to your PATH (brew will print the commands). Then verify:

```bash
brew --version
```

### A.2 Install Git and Visual Studio Code

```bash
brew install git
brew install --cask visual-studio-code
git --version
```

(You can install Git/VS Code from their websites if you prefer.)

### A.3 Install uv and create the project env

Using **Homebrew**:

```bash
# Install uv (see https://docs.astral.sh/uv/install)
# Using Homebrew
brew install uv

# Verify installation
uv --version
```

Using **curl**:

```bash
# Install uv (see https://docs.astral.sh/uv/install)
# Using curl
curl -LsSf https://astral.sh/uv/install.sh | sh
# Make uv command persist by adding to path if uv recommends it. 
# (Optional) For example:
echo 'source "$HOME/.local/bin/env"' >> ~/.zshrc
source ~/.zshrc

# Verify installation
uv --version
```
Initialize project and virtual environment:
```bash
# Create a folder for this tutorial and set up a venv on Python 3.11
mkdir -p ~/robot-viewer && cd ~/robot-viewer
uv python install 3.11
uv venv robotsim
source robotsim/bin/activate
uv pip install mujoco glfw numpy pyopengl

# Verify environment (should see mujoco listed)
uv pip list
```

### A.4 Clone the Unitree repository and open VS Code

```bash
# Clone repository
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots/g1_description

# Verify .urdf/.xml files present
ls

# Print current path (where you are)
pwd
```

Launch **Visual Studio Code**, open the `unitree_ros` folder in Explorer (output of pwd), then open a terminal (**Terminal → New Terminal**). In that terminal:

```bash
source ~/robot-viewer/robotsim/bin/activate
cd robots/g1_description
python -m mujoco.viewer
```

When the empty MuJoCo window opens, open the Finder app and locate the `unitree_ros` folder. Finally, **drag and drop** a G1 `*.xml` or `*.urdf` file into the MuJoCo window to visualize.


---

# B) Linux Guide (Ubuntu 22.04/24.04)

### B.1 Install prerequisites

```bash
sudo apt update
sudo apt install -y git curl build-essential
git --version
```

### B.2 Install VS Code (optional but recommended)

```bash
sudo apt install -y wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /usr/share/keyrings/packages.microsoft.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update && sudo apt install -y code
```

### B.3 Install uv and create the project env

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
# Restart shell and add to path if prompted

# Verify installation
uv --version

# Create project directory
mkdir -p ~/robot-viewer && cd ~/robot-viewer

# Create uv environment
uv python install 3.11
uv venv robotsim
source robotsim/bin/activate
uv pip install mujoco glfw numpy pyopengl

# Verify environment (should see mujoco listed)
uv pip list
```

### B.4 Clone and run

```bash
# Clone repo
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots/g1_description

# Verify .urdf/.xml files present
ls

# Launch viewer
python -m mujoco.viewer
```

Open native files app, and **drag & drop** a G1 model file into the MuJoCo viewer window to view it.

---

# C) Windows 11 Guide (via WSL Ubuntu + VS Code)

> Requires **Windows 11** for WSLg. If you’re on Windows 10, either upgrade or use a third‑party X server (not covered here).

### C.1 Enable WSL and install Ubuntu (PowerShell as Administrator)

```powershell
wsl --install -d Ubuntu
# Reboot if prompted, then complete Ubuntu user setup
wsl --update   # optional but recommended
```

### C.2 Install VS Code (Windows) and the WSL extension

- Download and install **Visual Studio Code for Windows**: https://code.visualstudio.com/  
- Launch VS Code → **Extensions** → install **“WSL”** and **“Python”** extensions.


### C.3 Set up the WSL environment (sys + uv + WSLg)

- VS Code: **Ctrl+Shift+P → “WSL: New WSL Window” → Ubuntu**.  
- **Terminal → New Terminal** (this is a Linux shell inside WSL).

```bash
# Basics
sudo apt update
sudo apt install -y git curl build-essential mesa-utils

# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh
# (Optional) Add uv to path across terminal sessions:
echo 'source "$HOME/.local/bin/env"' >> ~/.bashrc
source ~/.bashrc

# Verify installation
uv --version

# Project & env
mkdir -p ~/robot-viewer && cd ~/robot-viewer
uv python install 3.11
uv venv robotsim
source robotsim/bin/activate
uv pip install mujoco glfw numpy pyopengl

# Verify environment (should see mujoco listed)
uv pip list

# Get the models
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots/g1_description

# Verify .urdf/.xml files present
ls
```
WSLg-specific notes:
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
Note: Drag-and-drop from **Windows Explorer → MuJoCo (WSLg)** does not work. Nautilus (Linux Files App) drag-and-drop to MuJoCo is unreliable. We must script it.

### C.4 Launch viewer

Create viewer launch script
```bash
# If code command links to VSCode, run:
code ~/robot-viewer/open_model.py 
```
Paste this code onto the script and save (File -> Save or Ctrl+S):
```python
# open_model.py
import argparse, os, sys, time
import mujoco
import mujoco.viewer

def main():
    ap = argparse.ArgumentParser(description="Open an MJCF/URDF model in MuJoCo viewer")
    ap.add_argument("model", help="Path to .xml (MJCF) or .urdf")
    args = ap.parse_args()

    path = os.path.abspath(args.model)
    if not os.path.isfile(path):
        print(f"[ERROR] File not found: {path}", file=sys.stderr)
        sys.exit(1)

    # Heads-up for URDF
    if path.lower().endswith(".urdf"):
        print("[NOTE] URDF support is limited; prefer the provided MJCF .xml files if available.")

    try:
        model = mujoco.MjModel.from_xml_path(path)   # works for MJCF; URDF may work if compatible
    except Exception as e:
        print("[ERROR] MuJoCo failed to load the model. Details:", file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(2)

    data = mujoco.MjData(model)
    print(f"[OK] Loaded: {path}")
    with mujoco.viewer.launch_passive(model, data) as v:
        # simple idle loop; press ESC to quit
        while v.is_running():
            time.sleep(0.01)

if __name__ == "__main__":
    main()
```
Launch the custom viewer:
```bash
# Navigate to the hardware descriptors directory
cd unitree_ros/robots/g1_description
# Specify the .urdf/.xml file path to open with the viewer
# Recommended .xml file (MJCF)
uv run python ~/robot-viewer/open_model.py g1_23dof.xml 
```

> If the viewer opens but GPU isn’t used, confirm the `glxinfo -B` renderer shows **D3D12 (…GPU…)** rather than **llvmpipe** (CPU only).

---

## Repository Layout and Model Files (G1)

The G1 description files live in:

```
unitree_ros/robots/g1_description
```

To visualize: run `python -m mujoco.viewer` and **drag & drop** a `*.xml` (MJCF) or `*.urdf` into the window.

On WSL, prefer: `uv run python ~/robot-viewer/open_model.py <file.xml>` (drag-and-drop from Windows Explorer isn’t supported).

---

## Troubleshooting

### Windows/WSL: viewer won’t start or uses CPU (llvmpipe)
- Ensure **WSLg** is active (Windows 11) and you see `/dev/dxg` inside WSL:
  ```bash
  ls -l /dev/dxg
  ```
- Force Mesa’s D3D12 backend (hardware acceleration):
  ```bash
  export GALLIUM_DRIVER=d3d12
  glxinfo -B | grep -E 'OpenGL renderer|string|Accelerated'
  ```
  You should see `OpenGL renderer string: D3D12 (NVIDIA/AMD/Intel …)` and `Accelerated: yes`.

### VS Code terminal doesn’t see the venv `robotsim`
- Activate it explicitly each session:
  ```bash
  source ~/robot-viewer/robotsim/bin/activate
  ```

### MuJoCo viewer opens but is blank / crashes
- Update MuJoCo:
  ```bash
  uv pip install -U mujoco
  ```
- On macOS, if Gatekeeper blocks a binary, right‑click → **Open** and allow in **System Settings → Privacy & Security**.

---

## Appendix D — Conda users (optional)

If you must use Conda on **Windows/WSL**, be aware of this common pitfall:

- Conda brings its own **C++ runtime (`libstdc++`)** into the environment.
- WSLg’s windowing stack (GLFW/EGL) expects the **system** `libstdc++` ABI.
- The Conda runtime can override the system runtime, causing an ABI/symbol mismatch that breaks window creation (e.g., `GLXFBConfig`/GLFW errors).

**Workaround when using Conda (WSL):**

```bash
# 1) Use hardware accel:
export GALLIUM_DRIVER=d3d12

# 2) Prefer the system C++ runtime over Conda’s (avoids ABI mismatch):
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
# (Optional) Make the change persist across sessions
echo 'LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6' >> ~/.bashrc
source ~/.bashrc

# 3) Run
python -m mujoco.viewer
```

If you stick with Conda on Linux/macOS (non‑WSL), the `LD_PRELOAD` step is usually **not** needed.

---

## Optional: make this a tiny uv project

If you want a minimal repo to hold notes/scripts:

```bash
mkdir -p ~/robot-viewer && cd ~/robot-viewer
uv init  # creates pyproject.toml and a src/ package
uv add mujoco glfw numpy pyopengl
git init
git add .
git commit -m "robot-viewer: uv project with MuJoCo"
```

You can then add small helpers under `src/` (e.g., a script that launches the viewer).

---

## Credits

- [Unitree Robotics](https://www.unitree.com/)
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- Thanks to WSLg + Mesa D3D12 Gallium for cross‑platform GPU acceleration.
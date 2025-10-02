# Unitree G1 Description (URDF & MJCF) — End‑to‑End CLI Tutorial

This README takes you from a **clean macOS, Linux, or Windows installation** (no robotics stack installed) to **visualizing the Unitree G1 robot** in the MuJoCo viewer. The approach is **command‑line first** and uses **Miniconda** for Python environment management and **Visual Studio Code** for editing and an integrated terminal. On Windows we use **WSL (Windows Subsystem for Linux)** so everyone follows essentially the same Linux‑style CLI.

> **Outcome**  
> After completing the steps for your OS, you will:
> 1) Have Miniconda installed and initialized,  
> 2) Have a `robotsim` Conda environment with MuJoCo installed,  
> 3) Have Visual Studio Code installed and using its integrated terminal,  
> 4) Have cloned `unitree_ros`, navigated to `robots/g1_description`, and  
> 5) Launched `mujoco.viewer` to inspect G1 URDF/MJCF files.

---

## Quickstart (experienced users)

```bash
# macOS/Linux (or Windows inside WSL Ubuntu)
# 1) Install Miniconda (see full instructions below for your OS)
# 2) Then:
conda create -n robotsim python=3.11 -y
conda activate robotsim
pip install mujoco
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots/g1_description
python -m mujoco.viewer  # drag & drop g1_XXX.xml or .urdf into the viewer
```

If you are new to Conda/WSL/CLI, follow the full instructions below.

---

## Approach and Tools

- **CLI-first**: Everything is done from the terminal to make the setup deterministic and repeatable.
- **Miniconda**: Lightweight Python distribution to create an isolated environment named `robotsim`.
- **VS Code**: Used for editing and its **Integrated Terminal** so all commands are run consistently.
- **WSL (Windows)**: On Windows, we install Ubuntu under WSL with built-in GUI support (WSLg) so `mujoco.viewer` opens like a native app.

---

# A) macOS Guide (Apple Silicon or Intel)

### A.1 Install Homebrew (package manager)

Open the **Terminal** app and run the official installer:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Follow any on-screen instructions to add Homebrew to your PATH (brew will print the commands). Then verify:

```bash
brew --version
```

### A.2 Install Git and Visual Studio Code via Homebrew

```bash
brew install git
brew install --cask visual-studio-code
git --version
```

### A.3 Install Miniconda (non-interactive, CLI)

Choose your CPU type:

- **Apple Silicon (M1/M2/M3):**
  ```bash
  curl -LO https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh
  bash Miniconda3-latest-MacOSX-arm64.sh -b -p "$HOME/miniconda3"
  ```

- **Intel (x86_64):**
  ```bash
  curl -LO https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh
  bash Miniconda3-latest-MacOSX-x86_64.sh -b -p "$HOME/miniconda3"
  ```

Initialize Conda for your shell (macOS defaults to **zsh**):

```bash
# For zsh:
eval "$("$HOME/miniconda3/bin/conda" shell.zsh hook)"
conda init zsh
exec zsh  # restart the shell to apply changes
```

(If you use bash instead, replace `zsh` with `bash` in the commands above.)

Verify:

```bash
conda --version
```

### A.4 Create and activate the `robotsim` environment, install MuJoCo

```bash
conda create -n robotsim python=3.11 -y
conda activate robotsim
pip install mujoco
```

Test that the viewer starts (you can close it right away):

```bash
python -m mujoco.viewer
```

### A.5 Clone the Unitree repository and open VS Code

```bash
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots/g1_description
```

Now launch **Visual Studio Code** from your Applications folder. In VS Code:

1. **File → Open Folder…** and open the `unitree_ros` folder you just cloned.  
2. **Terminal → New Terminal** (this terminal starts in your repo folder).  
3. In the VS Code terminal:
   ```bash
   conda activate robotsim
   cd robots/g1_description
   python -m mujoco.viewer
   ```
4. When the empty MuJoCo window opens, **drag and drop** the G1 `*.xml` or `*.urdf` model file to visualize.

---

# B) Linux Guide (Ubuntu 22.04/24.04)

### B.1 Install prerequisites (Git, build tools, curl)

Open **Terminal** and run:

```bash
sudo apt update
sudo apt install -y git curl build-essential
git --version
```

### B.2 Install Visual Studio Code (official Microsoft repository)

```bash
sudo apt install -y wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /usr/share/keyrings/packages.microsoft.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update
sudo apt install -y code
code --version
```

### B.3 Install Miniconda (non-interactive, CLI)

```bash
curl -LO https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p "$HOME/miniconda3"
```

Initialize Conda for bash:

```bash
eval "$("$HOME/miniconda3/bin/conda" shell.bash hook)"
conda init bash
exec bash  # restart your shell
```

Verify:

```bash
conda --version
```

### B.4 Create and activate the `robotsim` environment, install MuJoCo

```bash
conda create -n robotsim python=3.11 -y
conda activate robotsim
pip install mujoco
python -m mujoco.viewer  # quick sanity check; close it
```

### B.5 Clone the Unitree repository and open VS Code

```bash
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots/g1_description
code ..  # opens the repo root in VS Code
```

In VS Code:
1. **Terminal → New Terminal**.  
2. In the integrated terminal:
   ```bash
   conda activate robotsim
   cd robots/g1_description
   python -m mujoco.viewer
   ```
3. **Drag and drop** the desired G1 `*.xml` or `*.urdf` file into the viewer.

---

# C) Windows 11 Guide (via WSL Ubuntu + VS Code)

> These steps assume **Windows 11** (WSLg provides built‑in Linux GUI support). If you are on Windows 10, see the troubleshooting notes below for X‑server setup.

### C.1 Enable WSL and install Ubuntu (PowerShell as Administrator)

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu
```

Reboot if prompted. After reboot, Windows will complete the Ubuntu install—choose a **username** and **password** for Linux.

Update WSL itself (optional but recommended):

```powershell
wsl --update
```

### C.2 Install VS Code (Windows) and the WSL extension

- Download and install **Visual Studio Code for Windows**: https://code.visualstudio.com/  
- Launch VS Code → **Extensions** → install **“WSL”** and **“Python”** extensions.

### C.3 Open a WSL (Ubuntu) window in VS Code

- In VS Code: **Ctrl+Shift+P → “WSL: New WSL Window” → Ubuntu**.  
- This opens a VS Code window connected to your Linux environment.  
- Open the terminal: **Terminal → New Terminal** (this is a Linux shell).

### C.4 Install system basics inside WSL (Ubuntu)

In the **VS Code WSL terminal**:

```bash
sudo apt update
sudo apt install -y git curl build-essential
git --version
```

GUI support (WSLg) is built in. If you hit OpenGL/GUI errors later, see Troubleshooting.

### C.5 Install Miniconda in WSL (Linux)

```bash
curl -LO https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p "$HOME/miniconda3"
```

Initialize Conda for bash:

```bash
eval "$("$HOME/miniconda3/bin/conda" shell.bash hook)"
conda init bash
exec bash  # restart your shell inside the terminal
```

Verify:

```bash
conda --version
```

### C.6 Create and activate the `robotsim` environment, install MuJoCo

```bash
conda create -n robotsim python=3.11 -y
conda activate robotsim
pip install mujoco
python -m mujoco.viewer  # sanity check; should open a GUI window in Windows via WSLg
```

### C.7 Clone the Unitree repository and visualize

```bash
git clone https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros/robots/g1_description
python -m mujoco.viewer
```

When the viewer opens, **drag and drop** the G1 `*.xml` or `*.urdf` file from the repository to display the model.

---

## Repository Layout and Model Files (G1)

You will find G1 description files under:

```
unitree_ros/robots/g1_description
```

MJCF/URDF for the G1 robot:

| MJCF/URDF file name                      | `mode_machine` | hip.{roll, pitch} gear ratio | wrist motor | lock waist | Update status | dof#leg | dof#waist | dof#arm | dof#hand |
|------------------------------------------|:--------------:|------------------------------|-------------|------------|---------------|:-------:|:---------:|:-------:|:--------:|
| `g1_23dof_mode_10`                       |       10       | {22.5, 22.5}                 | null        | no         | Up-to-date    |   6*2   |     1     |   5*2   |   null   |
| `g1_29dof_mode_11`                       |       11       | {22.5, 22.5}                 | 4010        | no         | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_12`                       |       12       | {22.5, 22.5}                 | 4010        | yes        | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_13`                       |       13       | {14.3, 22.5}                 | 5010(new)   | no         | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_14`                       |       14       | {14.3, 22.5}                 | 5010(new)   | yes        | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_15`                       |       15       | {22.5, 22.5}                 | 5010(new)   | no         | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_16`                       |       16       | {22.5, 22.5}                 | 5010(new)   | yes        | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_23dof_rev_1_0`                       |       4        | {14.3, 22.5}                 | null        |            | Up-to-date    |   6*2   |     1     |   5*2   |   null   |
| `g1_29dof_rev_1_0`                       |       5        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_with_hand_rev_1_0`             |       5        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   7*2    |
| `g1_29dof_rev_1_0_with_inspire_hand_DFQ` |       5        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   12*2   |
| `g1_29dof_rev_1_0_with_inspire_hand_FTP` |       5        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   12*2   |
| `g1_29dof_lock_waist_rev_1_0`            |       6        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_lock_waist_with_hand_rev_1_0`  |       6        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   7*2    |
| `g1_dual_arm`                            |       9        | null                         | 4010        |            | Up-to-date    |  null   |   null    |   7*2   |   null   |
| ~~`g1_23dof`~~                           |       1        | {14.3, 14.5}                 | null        |            | Deprecated    |   6*2   |     1     |   5*2   |   null   |
| ~~`g1_29dof`~~                           |       2        | {14.3, 14.5}                 | 4010        |            | Deprecated    |   6*2   |     3     |   7*2   |   null   |
| ~~`g1_29dof_with_hand`~~                 |       2        | {14.3, 14.5}                 | 4010        |            | Deprecated    |   6*2   |     3     |   7*2   |   7*2    |
| ~~`g1_29dof_lock_waist`~~                |       3        | {14.3, 14.5}                 | 4010        |            | Deprecated    |   6*2   |     3     |   7*2   |   null   |

**Note**: The robot's `mode_machine` ID can be viewed in the Unitree app under  
**Device → Data → Robot → Machine Type**.

---

## Launching the Viewer

From the VS Code integrated terminal (with `robotsim` active) and inside `unitree_ros/robots/g1_description`:

```bash
python -m mujoco.viewer
```

When the empty viewer window appears, **drag and drop** the desired G1 `*.xml` (MJCF) or `*.urdf` file into the window to visualize the model.

---

## Troubleshooting

### WSL GUI issues (Windows)
- Requires **Windows 11** (WSLg) for out‑of‑the‑box Linux GUI. Update WSL if needed:
  ```powershell
  wsl --update
  ```
- If the viewer fails to start or you see OpenGL errors in WSL, install common GL/X11 libs:
  ```bash
  sudo apt update
  sudo apt install -y libgl1 libx11-6 libxcb1 libxrandr2 libxi6 libxxf86vm1 libxinerama1
  ```
- On **Windows 10** (no WSLg): install an X server (e.g., VcXsrv), start it, and set `export DISPLAY=:0` in the WSL shell before running the viewer.

### Conda not available in terminal
- Ensure you ran the correct `conda init <shell>` and restarted your shell.  
- You can always load it explicitly per session:
  ```bash
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
  conda activate robotsim
  ```

### MuJoCo viewer opens but is blank / crashes
- Update GPU drivers on Windows (for WSLg), or try:
  ```bash
  pip install --upgrade mujoco
  ```

### macOS Gatekeeper blocks installer
- If macOS blocks Miniconda or VS Code, right‑click → **Open** and allow it in **System Settings → Privacy & Security**.

---

## Credits

- [Unitree Robotics](https://www.unitree.com/)
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- This tutorial standardizes a CLI workflow across macOS, Linux, and Windows via WSL for reproducibility.

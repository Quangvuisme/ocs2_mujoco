# OCS2 MuJoCo Control Framework

A comprehensive framework for Model Predictive Control (MPC) of various robotic systems using [OCS2](https://leggedrobotics.github.io/ocs2/) and [MuJoCo](https://mujoco.org/) simulation.

## Demo Videos

| ANYmal C | Unitree Go1 | Unitree Go2 |
|----------|-------------|-------------|
| ![ANYmal C demo](doc/gifs/anymal.gif) | ![Unitree Go1 demo](doc/gifs/go1.gif) | ![Unitree Go2 demo](doc/gifs/go2.gif) |

| Quadrotor NMPC | Cartpole Balance |
|----------------|------------------|
| ![Quadrotor NMPC demo](doc/gifs/quadrotor.gif) | ![Cartpole Balance demo](doc/gifs/cartpole.gif) |

| Roadster Kickflip on Cold-Gas Jets (OCS2 plan + NMPC) | ANYmal C Perceptive NMPC (elevation map from MuJoCo, no ROS) |
|--------------------------------------------------------|--------------------------------------------------------------|
| ![Roadster kickflip demo](doc/gifs/roadster.gif) | ![ANYmal C perceptive NMPC demo](doc/gifs/anymal_perceptive.gif) |

| Unitree G1 humanoid, 12 DoF (NMPC, no ROS) | Unitree G1 Perceptive NMPC (stairs, elevation map from MuJoCo, no ROS) |
|--------------------------------------------|------------------------------------------------------------------------|
| ![Unitree G1 walking demo](doc/gifs/g1.gif) | ![Unitree G1 perceptive NMPC demo](doc/gifs/g1_perceptive.gif) |

| Unitree G1 29 DoF backflip (whole-body NMPC of wb_humanoid_mpc, no ROS) |
|--------------------------------------------------------------------------|
| ![Unitree G1 backflip demo](doc/gifs/g1_backflip.gif) |

## 1. Summary

This project provides a complete framework for optimal control of legged robots and aerial vehicles using Model Predictive Control (MPC). It integrates OCS2's powerful optimization algorithms with MuJoCo's high-fidelity physics simulation, enabling real-time control of complex robotic systems.

### What's New (2026.11)

**Standing backflip of the 29-DoF Unitree G1 (`g1_backflip_mujoco`)**
- Built on the whole-body NMPC of [wb_humanoid_mpc](https://github.com/manumerous/wb_humanoid_mpc) (git submodule
  `src/wb_humanoid_mpc`): its OCS2 fork (SQP + HPIPM) and humanoid MPC compiled with plain CMake, without ROS
- Offline SQP plan over the whole manoeuvre from a keyframe guide (torque limits, friction, CoP, feet / knees above
  the floor), then the SQP NMPC tracks it in MuJoCo; a `flip_base` root frame avoids the Euler singularity
- Fixes a floating-base mass-matrix bug in wb_humanoid_mpc's whole-body dynamics (momentum was not conserved in flight)
- Lands and stands up in all scripted runs (different start times, pushes); lock-step, not real time

**Perceptive NMPC for the Unitree G1 humanoid (`g1_perceptive_mujoco`)**
- The ANYmal perception stack (elevation map ray cast from MuJoCo, footholds, swing / shin clearance) on the G1 biped
- Sole-aware footholds: toe and heel both on steppable cells of one tread; joint limits in the MPC; RNEA with
  acceleration feedforward and torque-level joint PD
- Climbs and descends stairs (0.08 m rise, 0.45 m tread) and crosses rough blocks: 23 of 25 scripted runs complete the course

**Unitree G1 humanoid, 12 leg DoF (`g1_walking_mujoco`)**
- Port of [cyugai/g1_control](https://github.com/cyugai/g1_control) without ROS, in the same format as the ANYmal C package:
  plain C++ / CMake, OCS2 compiled from `src/ocs2_ros2`, MuJoCo GUI, headless lock-step runs and GIF recording
- Centroidal NMPC with toe + heel point contacts per foot, biped gaits (walk, trot, quick_trot), height command via leg IK
- Walks forward / backward / sideways and turns; recovers from 200 N lateral pushes while walking
- Fix in the vendored OCS2 RNEA conversion: contact forces of points on the same link are now summed

**Perceptive NMPC for ANYmal C (`anymal_perceptive_mujoco`)**
- Elevation map ray-cast directly from the MuJoCo scene (`mj_ray` per cell): no elevation_mapping / grid_map /
  plane segmentation packages
- Steppability, edge distance, foothold selection with convex regions, terrain-aware swing trajectories,
  swing-foot and shank clearance, base pose and leg posture following the terrain
- Plain C++ / CMake build without ROS: the needed OCS2 libraries are compiled straight from `src/ocs2_ros2`
- Climbs and descends stairs and crosses rough blocks and a beam (headless lock-step test and offscreen GIF recording)

### What's New (2026.01)

**Keyboard Controller Support**
- Added KeyboardController alongside Xbox controller for all packages
- Full keyboard control: WASD for movement, IJKL for height/yaw, number keys for mode selection
- No Xbox controller required - pure keyboard control available
- Works on all platforms including WSL2

**Enhanced Control Framework**
- Unified interface between Xbox and Keyboard controllers
- Non-blocking terminal input with smooth control decay
- Thread-based input processing for responsive control

### Key Features

- **Multiple Robot Support**: ANYmal C, Unitree Go1/Go2 quadrupeds, Unitree G1 humanoid, quadrotors, cartpole, a jet-powered roadster
- **Real-time MPC**: Using OCS2 Centroidal Model and DDP/SQP solvers  
- **Whole-Body Control**: Converts MPC outputs to joint torques via RNEA
- **MuJoCo Simulation**: High-fidelity physics simulation with interactive GUI
- **UDP Communication**: Modular architecture separating controller and simulator
- **Dual Input Support**: Xbox controller OR keyboard control
- **FSM Controller**: Finite State Machine for mode transitions (Passive, PD, Standing, Walking)
- **Multiple Gaits**: Trot, pace, bound, and custom walking patterns

### Tested Platforms

* **Ubuntu 22.04 LTS** - ROS2 Humble
* **Ubuntu 24.04 LTS** - ROS2 Jazzy  
* **WSL2** (Ubuntu 22.04) - ROS2 Humble
* **Linux x86_64** with NVIDIA GPU (recommended for visualization)

## 2. Architecture

```
┌──────────────────────────────────────────────────────────┐
│                 Controller (OCS2 )                      │
│  ┌──────────┐  ┌──────────┐  ┌─────────┐  ┌─────────┐  │
│  │   Xbox   │  │ Keyboard │  │   FSM   │  │   MPC   │  │
│  │Controller│  │Controller│──│ Manager │──│ Thread  │  │
│  └──────────┘  └──────────┘  └─────────┘  │(DDP/SQP)│  │
│        │             │             │       └─────────┘  │
│        └─────────────┴─────────────┤                    │
│                                    │                    │
│              ┌─────────────────────▼──────────────┐     │
│              │       Control Thread               │     │
│              │                                    │     |
│              └──────────────┬─────────────────────┘     │
└─────────────────────────────│────────────────────────────┘
                              │ UDP (State/Control)
┌─────────────────────────────▼────────────────────────────┐
│                    MuJoCo Simulator                      │
│  ┌────────────┐  ┌────────────┐  ┌──────────────────┐   │
│  │   Robot    │  │  Physics   │  │  Interactive GUI │   │
│  │   Model    │  │   Engine   │  │  with Callbacks  │   │
│  └────────────┘  └────────────┘  └──────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

## 3. Packages

### Core Packages

| Package | Description | Features |
|---------|-------------|----------|
| `quadruped_nmpc` | Quadruped NMPC controller | Multi-robot (ANYmal/Go1/Go2), FSM, Gaits |
| `anymal_walking_online` | ANYmal walking control | MuJoCo GUI, Gait switching|
| `anymal_standing_online` | ANYmal standing control | MuJoCo GUI, Pose control|
| `ocs2_quadrotor_nmpc` | Quadrotor NMPC via UDP | Multiple targets, Joystick control |
| `quadrotor_online` | Quadrotor with GUI | Interactive MuJoCo, Real-time MPC |
| `ocs2_cartpole_upright` | Cartpole balance | UDP control |
| `roadster_stunt` | Roadster kickflip on cold-gas jets | OCS2 SLQ plan + NMPC, MuJoCo GUI, GIF recording |
| `anymal_perceptive_mujoco` | ANYmal C perceptive NMPC (no ROS) | Elevation map from MuJoCo, footholds, stairs, plain CMake |
| `g1_walking_mujoco` | Unitree G1 humanoid (12 DoF) NMPC (no ROS) | Toe/heel contacts, biped gaits, MuJoCo GUI, plain CMake |
| `g1_perceptive_mujoco` | Unitree G1 perceptive NMPC (no ROS) | Elevation map from MuJoCo, sole footholds, stairs, plain CMake |

### Support Packages

| Package | Description |
|---------|-------------|
| `anymal_centroidal` | ANYmal centroidal dynamics testing |
| `anymal_pinocchio` | ANYmal Pinocchio visualization |
| `quadrotor_offline` | Offline quadrotor trajectory optimization |
| `ocs2_ros2` | OCS2 core libraries (submodule) |

## 4. Dependencies

### 4.1 Prerequisites

* **Operating System**: Ubuntu 22.04 LTS (Jammy) or Ubuntu 24.04 LTS (Noble)
* **ROS2**: Humble (22.04) or Jazzy (24.04)
* **C++ Compiler**: GCC 11+ with C++17 support
* **CMake**: 3.14+
* **Git**: For cloning repositories

### 4.2 Required Libraries

| Library | Version | Purpose |
|---------|---------|---------|
| **Eigen3** | ≥3.3.9 | Linear algebra |
| **Boost** | ≥1.71 | Utilities (filesystem, system) |
| **Pinocchio** | ≥2.6 | Rigid body dynamics |
| **MuJoCo** | ≥3.0.0 | Physics simulation |
| **GLFW3** | ≥3.3 | OpenGL window management |
| **OCS2** | (submodule) | Optimal control framework |

### 4.3 Optional Dependencies

- **Xbox Controller**: For gamepad input (not required with keyboard support)
- **NVIDIA GPU**: Recommended for smooth visualization

## 5. Installation

### 5.1 Install ROS2

Follow the official [ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html) for your Ubuntu version.

```bash
# Ubuntu 22.04 - ROS2 Humble
sudo apt install ros-humble-desktop

# Ubuntu 24.04 - ROS2 Jazzy  
sudo apt install ros-jazzy-desktop
```

### 5.2 Install System Dependencies

```bash
# Install required packages
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libboost-all-dev \
    libglfw3-dev \
    libglfw3 \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-kdl-parser
```

### 5.3 Install MuJoCo

```bash
# Download MuJoCo 3.3.0 (or latest)
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.0/mujoco-3.3.0-linux-x86_64.tar.gz
tar -xzf mujoco-3.3.0-linux-x86_64.tar.gz
sudo mv mujoco-3.3.0 /usr/local/mujoco

# Add to environment (add to ~/.bashrc)
export MUJOCO_DIR=/usr/local/mujoco
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
```

### 5.4 Clone and Build Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws_ocs2/src
cd ~/ros2_ws_ocs2/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/ocs2_mujoco_control.git .
git submodule update --init --recursive

# Install ROS dependencies
cd ~/ros2_ws_ocs2
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

# Source workspace
source install/setup.bash
```

### 5.5 Build Simulator

```bash
# Build MuJoCo simulator
cd ~/ros2_ws_ocs2/simulate
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## 6. Quick Start Examples

This section provides step-by-step instructions for running each example.

### 6.1 Quadruped Walking (ANYmal C)

<details>
<summary>🐕 Click to expand Quadruped Walking example</summary>

**Build:**
```bash
cd ~/ros2_ws_ocs2
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select anymal_walking_online
source install/setup.bash
```

**Run:**
```bash
# Launch with MuJoCo GUI
./install/anymal_walking_online/lib/anymal_walking_online/anymal_walking_mujoco
```

**Controls:**

| Input | Action |
|-------|--------|
| **Mode Selection** |
| 1 | PASSIVE mode (motors off) |
| 2 | PD_CONTROL (stand up with PD) |
| 3 | STANDING mode (MPC control) |
| 4 | WALKING mode (MPC + Gait) |
| **Movement (WASD)** |
| W/S | Forward/Backward velocity |
| A/D | Left/Right strafe |
| I/K | Height up/down |
| J/L | Yaw rotation |
| **Commands** |
| Enter | Toggle MPC on/off |
| G | Switch gait (Trot→Pace→Bound) |
| -/= | Adjust height preset |
| Q | Emergency stop |
| Space | Reset velocities to zero |

**Available Gaits:**
- Trot (default)
- Pace
- Bound
- Flying Trot

</details>

### 6.2 Quadruped Standing (ANYmal C)

<details>
<summary>🦿 Click to expand Quadruped Standing example</summary>

**Build:**
```bash
colcon build --packages-select anymal_standing_online
source install/setup.bash
```

**Run:**
```bash
./install/anymal_standing_online/lib/anymal_standing_online/anymal_standing_mujoco
```

**Controls:**

| Input | Action |
|-------|--------|
| 1 | STANDING mode |
| 2 | PASSIVE mode |
| 3 | RECOVERY mode |
| W/S/A/D | Body position X/Y |
| I/K/J/L | Height / Yaw |
| R | Reset to default pose |
| Enter | Toggle MPC |

</details>

### 6.3 Quadruped NMPC (Multi-Robot)

<details>
<summary>🤖 Click to expand Multi-Robot Quadruped example</summary>

**Build:**
```bash
colcon build --packages-select quadruped_nmpc
source install/setup.bash
```

**Run with UDP Simulator:**

Terminal 1 - Simulator:
```bash
cd simulate/build
./ocs2_mujoco anymal_c  # or: unitree_go1, unitree_go2
```

Terminal 2 - Controller:
```bash
source install/setup.bash
./install/quadruped_nmpc/lib/quadruped_nmpc/quadruped_nmpc_main 0
# Arguments: 0=ANYmal-C, 1=Unitree-Go1, 2=Unitree-Go2
```

**Supported Robots:**
- ANYmal C (ANYbotics)
- Unitree Go1
- Unitree Go2

</details>

### 6.4 Quadrotor Interactive

<details>
<summary>🚁 Click to expand Quadrotor example</summary>

**Build:**
```bash
colcon build --packages-select quadrotor_online
source install/setup.bash
```

**Run:**
```bash
./install/quadrotor_online/lib/quadrotor_online/quadrotor_interactive_gui
```

**Controls:**

| Input | Action |
|-------|--------|
| 1-4 | Select target position |
| W/S | Move forward/backward |
| A/D | Move left/right |
| I/K | Move up/down |
| J/L | Rotate yaw |
| Enter | Toggle run/pause |
| Space | Reset position |

**Features:**
- Real-time MPC trajectory optimization
- Interactive target selection via UI
- Smooth joystick/keyboard control

</details>

### 6.5 Quadrotor NMPC (UDP)

<details>
<summary>🛸 Click to expand Quadrotor NMPC example</summary>

**Build:**
```bash
colcon build --packages-select ocs2_quadrotor_nmpc
source install/setup.bash
```

**Run with UDP Simulator:**

Terminal 1 - Simulator:
```bash
cd simulate/build
./ocs2_mujoco quadrotor
```

Terminal 2 - Controller:
```bash
./install/ocs2_quadrotor_nmpc/lib/ocs2_quadrotor_nmpc/ocs2_quadrotor_nmpc_udp
```

**Features:**
- UDP communication between controller and simulator
- Multiple preset targets
- Velocity-based joystick control

</details>

### 6.6 Cartpole Balance

<details>
<summary>🛒 Click to expand Cartpole example</summary>

**Build:**
```bash
colcon build --packages-select ocs2_cartpole_upright
source install/setup.bash
```

**Run:**

Terminal 1 - Simulator:
```bash
cd simulate/build
./ocs2_mujoco cartpole
```

Terminal 2 - Controller:
```bash
./install/ocs2_cartpole_upright/lib/ocs2_cartpole_upright/cartpole_udp_nmpc
```

**Features:**
- Swing-up and balance control
- Real-time MPC stabilization
- Classic control benchmark

</details>

### 6.7 Roadster Kickflip on Cold-Gas Jets

<details>
<summary>🚗 Click to expand Roadster stunt example</summary>

A 2 t sports car drives at 18 m/s, lifts off on 12 on/off compressed-air jets, does a full barrel roll over a
row of parked vehicles and lands on its wheels. Inspired by the Tesla Roadster demo; scene and car numbers follow
[xkhainguyen/roadster-stunt](https://github.com/xkhainguyen/roadster-stunt), re-implemented with OCS2 + MuJoCo.

**Build:**
```bash
colcon build --packages-up-to roadster_stunt
source install/setup.bash
```

**Run:**
```bash
# Live MuJoCo window (SPACE pause, R restart, L loop, F follow camera, ESC exit)
./install/roadster_stunt/lib/roadster_stunt/roadster_stunt_mujoco --slowmo 0.5

# Headless run with a touchdown report and a CSV flight log
./install/roadster_stunt/lib/roadster_stunt/roadster_stunt_mujoco --headless --log flight.csv

# Re-record doc/gifs/roadster.gif (needs ffmpeg, uses xvfb-run without a display)
src/roadster_stunt/scripts/make_gif.sh
```

**Features:**
- Offline stunt plan with OCS2 SLQ (planar roll model, jet limits and clearance as soft constraints)
- Closed-loop OCS2 NMPC (50 Hz, feedback policy at 100 Hz) tracking the plan and landing softly
- Pitch/yaw hold, bounded least-squares allocation to 12 jets, on/off valves, limited air tank
- Optional mid-roll gust disturbance (`config/task.info`)

</details>

### 6.8 Perceptive NMPC on Terrain (ANYmal C, no ROS)

<details>
<summary>🪜 Click to expand perceptive locomotion example</summary>

The MPC sees the terrain through an elevation map that is ray-cast straight from the MuJoCo scene, with no ROS
and no perception packages. It picks footholds on steppable cells, lifts the swing feet over steps and obstacles, and
lets the base follow the terrain. Plain CMake build; details in
[src/anymal_perceptive_mujoco/README.md](src/anymal_perceptive_mujoco/README.md).

**Build (no ROS):**
```bash
pip install pin cmeel-urdfdom-headers          # pinocchio 3 (or robotpkg / any pinocchio 3 install)
export MUJOCO_DIR=/path/to/mujoco-3.3.0         # MuJoCo release directory
src/anymal_perceptive_mujoco/scripts/build.sh   # -> build/anymal_perceptive/
```

**Run:**
```bash
# MuJoCo GUI on the stairs / rough-terrain course (keys 2 -> 3 -> 4, then W/A/S/D in the terminal)
build/anymal_perceptive/anymal_perceptive_mujoco

# Deterministic headless run over the whole course (exit code 0 when the goal is reached)
build/anymal_perceptive/anymal_perceptive_mujoco --headless --vx 0.3 --goal-x 8.3

# The same run with a blind MPC (flat-ground assumption), for comparison
build/anymal_perceptive/anymal_perceptive_mujoco --headless --vx 0.3 --goal-x 8.3 --blind

# Re-record doc/gifs/anymal_perceptive.gif (needs ffmpeg, uses xvfb-run without a display)
src/anymal_perceptive_mujoco/scripts/make_gif.sh
```

**Features:**
- Robot-centric 3.2 x 3.2 m elevation map at 2 cm from `mj_ray`, 20 Hz, robot geoms filtered by geom group
- Steppability (plane fit, slope, roughness), edge distance, inflated layer for swing collision avoidance
- Foothold search around the Raibert foothold, convex foot placement regions as soft constraints
- Swing apex above the highest obstacle on the way, swing-foot and shank clearance above the map
- Base height / pitch / roll and leg IK posture from the terrain; standing trot by default
- Completed the course in 37 of 40 scripted runs (3 speeds x 6 start offsets)

</details>

### 6.9 Unitree G1 Humanoid Walking (12 DoF, no ROS)

<details>
<summary>🚶 Click to expand humanoid walking example</summary>

Centroidal NMPC for the Unitree G1 with its 12 leg joints (waist and arms fixed), ported from
[cyugai/g1_control](https://github.com/cyugai/g1_control) without ROS. Same format as the ANYmal C package; details in
[src/g1_walking_mujoco/README.md](src/g1_walking_mujoco/README.md).

**Build (no ROS):**
```bash
pip install pin cmeel-urdfdom-headers          # pinocchio 3 (or robotpkg / any pinocchio 3 install)
export MUJOCO_DIR=/path/to/mujoco-3.3.0         # MuJoCo release directory
src/g1_walking_mujoco/scripts/build.sh          # -> build/g1_walking/
```

**Run:**
```bash
# MuJoCo GUI (keys 2 -> 3 -> 4 in the terminal, then W/A/S/D, J/L, G for the next gait)
build/g1_walking/g1_walking_mujoco

# Deterministic headless run: walk 2 m at 0.3 m/s and stop (exit code 0 = goal reached without falling)
build/g1_walking/g1_walking_mujoco --headless --vx 0.3 --goal-x 2.0

# Re-record doc/gifs/g1.gif (needs ffmpeg, uses xvfb-run without a display)
src/g1_walking_mujoco/scripts/make_gif.sh
```

**Features:**
- 4 point contacts (toe + heel of each foot), friction cones, ankle roll lock, swing height splines
- Gaits: `walk` (with double support), `trot` and `quick_trot` (alternating single support)
- Pelvis height command with a leg IK posture reference (0.66 – 0.76 m)
- RNEA whole-body torques + joint PD at 1 kHz, MPC (SLQ) at 100 Hz, ~6 ms per solve while walking

</details>

### 6.10 Perceptive NMPC on Terrain (Unitree G1, no ROS)

<details>
<summary>🧗 Click to expand humanoid perceptive locomotion example</summary>

The G1 MPC of 6.9 with the perception of 6.8: an elevation map ray cast from the MuJoCo scene, footholds where the toe
and the heel of a sole both land on one steppable tread, swing and shin clearance above the map. Details in
[src/g1_perceptive_mujoco/README.md](src/g1_perceptive_mujoco/README.md).

**Build (no ROS):**
```bash
pip install pin cmeel-urdfdom-headers          # pinocchio 3 (or robotpkg / any pinocchio 3 install)
export MUJOCO_DIR=/path/to/mujoco-3.3.0         # MuJoCo release directory
src/g1_perceptive_mujoco/scripts/build.sh       # -> build/g1_perceptive/
```

**Run:**
```bash
# MuJoCo GUI on the stairs / rough-block course (keys 2 -> 3 -> 4 in the terminal, then W/A/S/D)
build/g1_perceptive/g1_perceptive_mujoco

# Deterministic headless run over the whole course (exit code 0 when the goal is reached)
build/g1_perceptive/g1_perceptive_mujoco --headless --vx 0.25 --goal-x 8.4

# The same run with a blind MPC (flat-ground assumption): falls at the first step
build/g1_perceptive/g1_perceptive_mujoco --headless --vx 0.25 --goal-x 8.4 --blind

# Re-record doc/gifs/g1_perceptive.gif (needs ffmpeg, uses xvfb-run without a display)
src/g1_perceptive_mujoco/scripts/make_gif.sh
```

**Features:**
- Foothold search for a whole sole: toe and heel on steppable cells, on one level, away from step edges
- Pelvis height over the terrain and 6D leg IK posture with the soles flat on the terrain
- Swing toe / heel / ankle and shin clearance above the map, joint limits in the MPC
- 23 of 25 scripted runs complete the course (0.2 – 0.3 m/s, lateral offsets up to ±0.3 m)

</details>
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

# Source workspace
source install/setup.bash
```

### 5.5 Build Simulator (Optional)

```bash
# Build MuJoCo simulator
cd ~/ros2_ws_ocs2/simulate
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## 7. Input Control Reference

### 7.1 Keyboard Controls

All packages support full keyboard control without requiring Xbox controller:

| Category | Key | Function |
|----------|-----|----------|
| **Movement** | W | Forward |
| | S | Backward |
| | A | Strafe left |
| | D | Strafe right |
| **Altitude/Yaw** | I | Up/Height increase |
| | K | Down/Height decrease |
| | J | Rotate left (yaw) |
| | L | Rotate right (yaw) |
| **Mode Selection** | 1 | Mode 1 (PASSIVE/Target 1) |
| | 2 | Mode 2 (PD_CONTROL/Target 2) |
| | 3 | Mode 3 (STANDING/Target 3) |
| | 4 | Mode 4 (WALKING/Target 4) |
| **Commands** | Enter | Start/Toggle MPC |
| | Space | Reset stick values |
| | Q | Emergency stop (legged robots) |
| | G | Switch gait (walking mode) |
| | - | Decrease height preset |
| | = | Increase height preset |
| | R | Reset to default pose |

### 7.2 Xbox Controller Mapping

Xbox controller provides the same functionality through analog sticks and buttons:

| Input | Quadruped | Quadrotor |
|-------|-----------|-----------|
| **Left Stick** | X/Y velocity | X/Y position |
| **Right Stick** | Yaw / Height | Yaw / Z position |
| **A Button** | PASSIVE mode | Target 1 |
| **B Button** | PD_CONTROL | Target 2 |
| **X Button** | STANDING | Target 3 |
| **Y Button** | WALKING | Target 4 |
| **Start** | Toggle MPC | Run/Pause |
| **D-Pad** | Gait selection | - |
| **LB/RB** | Height preset | - |

## 8. Configuration Files

### 8.1 MPC Configuration

| File | Description |
|------|-------------|
| `config/mpc/task.info` | MPC cost weights, horizons, solver settings |
| `config/mpc/ddp.info` | DDP solver parameters |
| `config/mpc/sqp.info` | SQP solver parameters |
| `config/mpc/ipm.info` | Interior Point Method settings |

### 8.2 Gait Configuration

| File | Description |
|------|-------------|
| `config/command/gait.info` | Gait cycle definitions (duty factor, phase) |
| `config/command/mode_schedule.info` | FSM mode transitions |
| `config/command/target_trajectories.info` | Reference trajectories |

### 8.3 Robot Configuration

| File | Path | Description |
|------|------|-------------|
| URDF | `robots/*/urdf/*.urdf` | Robot kinematic structure |
| MuJoCo Model | `robots/*/mujoco/*.xml` | Simulation model |
| Joint Limits | `config/robots/*/joint_limits.info` | Actuator constraints |

## 9. Supported Robots

### 9.1 Quadrupeds

| Robot | DOF | Mass | Features |
|-------|-----|------|----------|
| **ANYmal C** | 12 | 50 kg | Hydraulic actuators, high payload |
| **Unitree Go1** | 12 | 12 kg | Consumer-grade, affordable |
| **Unitree Go2** | 12 | 15 kg | Improved Go1, better sensors |

### 9.2 Humanoids

| Robot | DOF | Mass | Features |
|-------|-----|------|----------|
| **Unitree G1** | 12 (legs; waist / arms fixed) | 32 kg | Toe + heel contacts per foot, biped gaits |

### 9.3 Aerial Vehicles

| Robot | DOF | Control | Features |
|-------|-----|---------|----------|
| **Quadrotor** | 6 (SE(3)) | 4 motors | X-configuration, aggressive maneuvers |

### 9.4 Others

| Robot | DOF | Type | Features |
|-------|-----|------|----------|
| **Cartpole** | 2 | Underactuated | Swing-up, balance control |

## 10. Troubleshooting

### 10.1 Common Issues

<details>
<summary>Build errors with OCS2</summary>

```bash
# Clean and rebuild
cd ~/ros2_ws_ocs2
rm -rf build/ install/ log/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
</details>

<details>
<summary>MuJoCo library not found</summary>

```bash
# Ensure MuJoCo is in library path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/mujoco/lib
# Add to ~/.bashrc to make permanent
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/mujoco/lib' >> ~/.bashrc
```
</details>

<details>
<summary>Keyboard input not working</summary>

- Make sure terminal has focus (click on terminal window)
- Try running with `sudo` if permission issues
- Check that termios is properly configured
</details>

<details>
<summary>Robot falls immediately</summary>

- Press appropriate mode key (3 for STANDING, 4 for WALKING)
- Wait for MPC to initialize (1-2 seconds)
- Check that MPC is enabled (press Enter to toggle)
</details>

### 10.2 Performance Tips

- **Use Release build**: `-DCMAKE_BUILD_TYPE=Release` gives 10x speedup
- **Adjust MPC horizon**: Shorter horizon = faster computation
- **Reduce visualization FPS**: Lower render rate if simulation is slow
- **Use GPU**: NVIDIA GPU significantly improves visualization

## 11. Advanced Usage

### 11.1 Custom Robot Integration

See [doc/CUSTOM_ROBOT.md](doc/CUSTOM_ROBOT.md) for detailed guide on adding new robots.

### 11.2 Tuning MPC Parameters

Key parameters to tune in `config/mpc/task.info`:

```ini
; Cost weights
stateWeight              ; Tracking error penalty
inputWeight              ; Control effort penalty
endEffectorWeight        ; Foot position tracking

; Timing
mpc.timeHorizon          ; Prediction horizon (s)
mpc.dt                   ; Discretization timestep

; Solver
ddp.maxNumIterations     ; Max DDP iterations
sqp.maxNumIterations     ; Max SQP iterations
```

### 11.3 Custom Gaits

Edit `config/command/gait.info` to define custom gaits:

```ini
; Example: Custom trot gait
[trot]
numLegs              4
cycleTime            0.5           ; Total cycle duration
dutyFactor           0.6           ; Stance phase ratio
phaseOffset          0.0 0.5 0.5 0.0  ; LF RF LH RH
```

## 12. Documentation

- [MATH DOC](doc/quadruped_doc/00_INDEX.md.md) - In-depth system design

## 13. Related Projects

- [OCS2 ROS2](https://github.com/legubiao/ocs2_ros2) -  OCS2 ROS2 
- [OCS2](https://github.com/leggedrobotics/ocs2) OCS2

## 14. License

This project is licensed under the **BSD 3-Clause License** - see the [LICENSE](LICENSE) file for details.

### Third-Party Licenses

| Component | License | Copyright |
|-----------|---------|-----------|
| OCS2 | BSD 3-Clause | Farbod Farshidian, ETH Zurich |
| ANYmal C | BSD 3-Clause | ANYbotics AG |
| Unitree Robots | BSD 3-Clause | Unitree Robotics |
| g1_control (G1 12-DoF model) | MIT | botianhao |
| MuJoCo | Apache 2.0 | DeepMind Technologies |
| Pinocchio | BSD 2-Clause | CNRS, INRIA |

## 15. Acknowledgments

This project builds upon excellent open-source work:

- **[OCS2](https://leggedrobotics.github.io/ocs2/)** - Optimal Control for Switched Systems by ETH Zurich
- **[MuJoCo](https://mujoco.org/)** - Multi-Joint dynamics with Contact by DeepMind
- **[Pinocchio](https://github.com/stack-of-tasks/pinocchio)** - Rigid body dynamics library
- **[ANYbotics](https://www.anybotics.com/)** - ANYmal robot design and inspiration
- **[Unitree Robotics](https://www.unitree.com/)** - Affordable quadruped platforms

Special thanks to:
- Farbod Farshidian and the OCS2 team
- Ruben Grandia for legged robot locomotion research
- The ROS2 and open robotics community

## 16. Citation

If you use this framework in your research, please cite:

```bibtex

@inproceedings{farshidian2017efficient,
  title={Efficient optimal control for switched systems with application to locomotion},
  author={Farshidian, Farbod and Neunert, Michael and Winkler, Alexander and Grandia, Ruben and Buchli, Jonas},
  booktitle={2017 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={5531--5538},
  year={2017},
  organization={IEEE}
}
```

## 17. Contributing

We welcome contributions! Here's how you can help:

### Ways to Contribute

- 🐛 **Report bugs** - Open an issue with reproduction steps
- 💡 **Suggest features** - Propose new capabilities or improvements
- 📝 **Improve documentation** - Fix typos, add examples, clarify instructions
- 🔧 **Submit code** - Fix bugs, add features, optimize performance
- 🤖 **Add robots** - Integrate new robot models

### Development Workflow

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Test thoroughly (build, run examples)
5. Commit with clear messages (`git commit -m 'Add amazing feature'`)
6. Push to your fork (`git push origin feature/amazing-feature`)
7. Open a Pull Request

### Code Style

- Follow [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- Use meaningful variable names
- Add comments for complex logic
- Include docstrings for public functions

## 18. Contact & Support

- 📧 **Email**: vuquangcl8a@gmail.com
- 💬 **Discussions**: Use GitHub Discussions for questions
- 🐛 **Issues**: Report bugs via GitHub Issues
- 📖 **Wiki**: Check the [Wiki](../../wiki) for FAQs

---

**Star ⭐ this repository if you find it useful!**

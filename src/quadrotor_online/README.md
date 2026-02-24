# VM Quadrotor Online NMPC

Online quadrotor NMPC with real-time MuJoCo physics simulation.

## Overview

This package demonstrates **real-time Model Predictive Control (MPC)** of a quadrotor in MuJoCo physics simulation.

**Key differences from offline version:**
- MuJoCo runs actual physics simulation (not just visualization)
- MPC computes optimal control at each timestep (closed-loop)
- Control inputs are applied to the quadrotor in real-time
- State feedback from MuJoCo creates a true closed-loop system

## Two Versions Available

### 1. Simple Version (quadrotor_online_mujoco)
Basic GUI with essential controls and state visualization.

### 2. Interactive GUI Version (quadrotor_interactive_gui) ⭐ NEW
Full-featured interactive GUI similar to the `simulate` folder with:
- Complete MuJoCo UI panels
- Joint/actuator controls
- Visualization options
- Camera presets
- Advanced rendering controls
- Profiler and sensor displays

## Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Target    │────>│     MPC     │────>│   MuJoCo    │
│  Position   │     │  Controller │     │  Simulation │
└─────────────┘     └─────────────┘     └─────────────┘
                          ^                    │
                          │    State Feedback  │
                          └────────────────────┘
```

## Features

- ✅ Real-time MPC control at 50 Hz
- ✅ MuJoCo physics simulation
- ✅ Closed-loop state feedback
- ✅ Multiple target presets (press 1-4)
- ✅ Interactive camera controls
- ✅ Pause/Resume simulation
- ✅ Reset to initial state
- ✅ Debug output showing state and control
- ✅ Full interactive GUI with UI panels (NEW)

## Building

```bash
cd ~/ros2_ws_ocs2
colcon build --packages-select quadrotor_online
source install/setup.bash
```

## Running

### Simple Version
```bash
ros2 run quadrotor_online quadrotor_online_mujoco
```

Or directly:
```bash
./build/quadrotor_online/quadrotor_interactive_gui
```

### Interactive GUI Version ⭐ RECOMMENDED
```bash
# Using the convenience script
./src/quadrotor_online/build_and_run_gui.sh

# Or manually:
ros2 run quadrotor_online quadrotor_interactive_gui

# Or directly:
./install/quadrotor_online/lib/quadrotor_online/quadrotor_interactive_gui
```

## Controls

### Simple Version
| Key | Action |
|-----|--------|
| SPACE | Pause/Resume simulation |
| R | Reset to initial state |
| 1 | Target 1: [0.5, 0.5, 1.5] (default) |
| 2 | Target 2: [1.0, 0.0, 1.5] |
| 3 | Target 3: [0.0, 1.0, 2.0] |
| 4 | Target 4: [-0.5, -0.5, 0.8] |
| V | Reset camera view |
| Mouse | Rotate/Zoom/Pan camera |
| ESC | Exit |

### Interactive GUI Version
The interactive version includes all controls from the simple version plus:
- **UI Panels**: Access via Tab/Shift-Tab
- **Mouse Controls**: 
  - Left drag: Rotate view
  - Right drag: Pan view
  - Scroll/Middle drag: Zoom
  - Double-click: Select objects
- **Keyboard**:
  - Space: Play/Pause
  - F1-F5: Toggle various displays
  - [ ]: Cycle through cameras
  - +/-: Adjust simulation speed

## Configuration

Edit `config/task.info` to modify:
- Initial state
- Target position
- MPC horizon and settings
- Cost weights (Q, R matrices)
- Quadrotor parameters

### Key MPC Parameters

```
mpc
{
  timeHorizon     3.0    ; MPC prediction horizon [s]
  solutionTimeWindow  1.0  ; Solution window for warm start
  mpcDesiredFrequency  50  ; MPC update rate [Hz]
}

ddp
{
  maxNumIterations  3    ; More iterations for better convergence
  useFeedbackPolicy true ; Enable feedback control
}
```

## File Structure

```
quadrotor_online/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── task.info
├── include/quadrotor_online/
│   ├── QuadrotorDefinition.h
│   ├── QuadrotorDynamics.h
│   ├── QuadrotorInterface.h
│   ├── QuadrotorParameters.h
│   └── package_path.h.in
├── robots/
│   └── quadrotor.xml
└── src/
    ├── QuadrotorDynamics.cpp
    ├── QuadrotorInterface.cpp
    └── main_online_mujoco.cpp
```

## State and Control

**State (12D):**
```
[x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
```

**Control Input (4D):**
```
[Fz, Mx, My, Mz] = [Thrust, Roll moment, Pitch moment, Yaw moment]
```

## Tuning Guide

### If quadrotor doesn't reach target:
1. **Increase Q weights** - Makes the controller more aggressive on tracking
2. **Decrease R weights** - Allows larger control inputs
3. **Increase MPC horizon** - Gives the controller more planning time
4. **Increase maxNumIterations** - Better optimization convergence

### If quadrotor becomes unstable:
1. **Decrease Q weights** - Less aggressive tracking
2. **Increase R weights** - Penalize large control inputs
3. **Reduce control saturation limits** in code
4. **Check model match** - Ensure OCS2 dynamics match MuJoCo model

### Current tuned values (config/task.info):
```info
; Conservative Q weights for stability
Q { scaling 1e+0
  (0,0) 10.0  ; x position
  (2,2) 20.0  ; z position (stronger)
  (3,3) 5.0   ; roll
}

; R weights to limit control effort
R { scaling 1e+0
  (0,0) 0.01  ; Thrust (allow variation)
  (1,1) 10.0  ; Torques (penalized more)
}

; MPC settings
mpc {
  timeHorizon  3.0   ; 3 second horizon
  maxNumIterations 5 ; 5 iterations per solve
}
```

## Comparison: Online vs Offline

| Feature | Offline | Online |
|---------|---------|--------|
| Trajectory | Pre-computed | Real-time |
| MuJoCo | Visualization only | Physics simulation |
| Feedback | Open-loop | Closed-loop |
| Disturbances | Cannot handle | Can reject |
| Computational cost | One-time | Continuous |

## Dependencies

- OCS2 (ocs2_core, ocs2_ddp, ocs2_mpc)
- MuJoCo
- GLFW3
- OpenGL
- Eigen3
- Boost

## Troubleshooting

### Warning: "The solution time window might be shorter than the MPC delay"
- Increase `solutionTimeWindow` in `config/task.info`
- Reduce `mpcDesiredFrequency` 

### Quadrotor crashes/unstable
- Check control saturation limits in `main_online_mujoco.cpp`
- Reduce Q weights, increase R weights
- Verify MuJoCo model parameters match `task.info`

### Slow response
- Increase Q weights
- Decrease R weights  
- Increase `maxNumIterations`


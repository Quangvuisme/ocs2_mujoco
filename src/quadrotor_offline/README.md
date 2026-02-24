# VM Quadrotor Offline NMPC

Offline quadrotor trajectory optimization with MuJoCo visualization.

## Overview

This package demonstrates:
1. **Offline trajectory optimization** using OCS2's iLQR algorithm
2. **MuJoCo visualization** for trajectory replay (no real-time control)

The quadrotor trajectory is computed offline, then visualized in MuJoCo. This is useful for:
- Testing and validating NMPC algorithms
- Visualizing optimal trajectories
- Debugging trajectory generation

## Features

- ✅ Offline trajectory computation using iLQR
- ✅ Pre-computed trajectory replay in MuJoCo
- ✅ Interactive camera controls
- ✅ Playback speed control
- ✅ Loop mode for continuous replay

## Building

```bash
cd ~/ros2_ws_ocs2
colcon build --packages-select quadrotor_offline
source install/setup.bash
```

## Running

```bash
ros2 run quadrotor_offline quadrotor_offline_mujoco
```

## Controls

| Key | Action |
|-----|--------|
| SPACE | Pause/Resume playback |
| L | Toggle loop mode |
| UP/DOWN | Adjust playback speed |
| R | Reset camera view |
| Mouse Left | Rotate camera |
| Mouse Right | Pan camera |
| Scroll | Zoom in/out |
| ESC | Exit |

## Configuration

Edit `config/task.info` to modify:
- Initial state
- Target position
- Cost weights (Q, R matrices)
- Solver settings

### Key Parameters
```info
; Solver settings for offline optimization
ddp
{
  algorithm         ILQR
  maxNumIterations  100      ; More iterations for better convergence
  minRelCost        1e-12    ; Very tight convergence
}

; Time horizon
mpc
{
  timeHorizon      2.0      ; 2 second trajectory
}

; Cost weights
Q { scaling 1.0
  (0,0) 10.0   ; x position
  (1,1) 10.0   ; y position
  (2,2) 10.0   ; z position
}

R { scaling 1e-3
  (0,0) 1.0    ; Thrust
  (1,1) 1.0    ; Roll torque
}
```

## File Structure

```
quadrotor_offline/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── task.info           # Configuration file
├── include/
│   └── quadrotor_offline/
│       ├── QuadrotorDefinition.h
│       ├── QuadrotorDynamics.h
│       ├── QuadrotorInterface.h
│       ├── QuadrotorParameters.h
│       └── package_path.h.in
├── robots/
│   └── quadrotor.xml       # MuJoCo model
└── src/
    ├── QuadrotorDynamics.cpp
    ├── QuadrotorInterface.cpp
    └── main_offline_mujoco.cpp
```

## State and Input Definitions

**State (12D):**
- Position: `[x, y, z]`
- Orientation: `[roll, pitch, yaw]`
- Linear velocity: `[vx, vy, vz]`
- Angular velocity: `[wx, wy, wz]`

**Input (4D):**
- `Fz`: Total thrust force
- `Mx`: Roll moment
- `My`: Pitch moment
- `Mz`: Yaw moment

## Comparison: Offline vs Online

| Feature | Offline (this package) | Online |
|---------|---------|--------|
| Trajectory | Pre-computed once | Real-time MPC |
| MuJoCo role | Visualization only | Physics simulation |
| Feedback | Open-loop replay | Closed-loop control |
| Use case | Testing, validation | Real-time control |
| Speed | Fast (one-time) | Requires real-time compute |

## When to Use Offline Version

1. **Testing dynamics model** - Verify trajectory generation without physics
2. **Tuning cost weights** - Quick iteration on Q, R matrices  
3. **Visualization** - Show ideal trajectories
4. **Debugging** - Isolate trajectory generation from control issues

## Dependencies

- OCS2 (ocs2_core, ocs2_ddp, ocs2_mpc, ocs2_robotic_tools)
- MuJoCo
- GLFW3
- Eigen3
- Boost

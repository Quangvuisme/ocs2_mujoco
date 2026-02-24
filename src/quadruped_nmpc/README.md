# VM Quadruped NMPC

Standalone Quadruped NMPC Controller via UDP communication.

## Overview

This package provides a complete OCS2 Centroidal MPC + WBC controller for quadruped robots (ANYmal C) that communicates with MuJoCo simulation via UDP bridge.

## Features

- **OCS2 Centroidal MPC**: Model Predictive Control using the centroidal dynamics model
- **Whole-Body Control (WBC)**: Converts MPC output to joint torques
- **UDP Communication**: Real-time communication with MuJoCo via quadruped_bridge
- **FSM Controller**: Finite State Machine for mode transitions
- **Xbox Controller Support**: Control robot via gamepad
- **Gait Support**: Standing and walking gaits (trot, pace, bound, etc.)

## Architecture

```
┌────────────────────────────────────────────────────────┐
│                  quadruped_nmpc                     │
│                                                        │
│  ┌──────────┐  ┌─────────┐  ┌──────────────────────┐  │
│  │   Xbox   │──│   FSM   │──│       MPC Thread     │  │
│  │Controller│  │         │  │  (OCS2 Centroidal)   │  │
│  └──────────┘  └─────────┘  └──────────────────────┘  │
│                     │                  │               │
│                     │       ┌──────────▼──────────┐   │
│                     │       │   Control Thread    │   │
│                     │       │  (WBC + Torque)     │   │
│                     │       └──────────┬──────────┘   │
│                     │                  │               │
│              ┌──────▼──────────────────▼──────┐       │
│              │         UDP Communication       │       │
│              │   State:9101  Command:9102      │       │
│              └────────────────┬────────────────┘       │
└───────────────────────────────│────────────────────────┘
                                │
                    ┌───────────▼───────────┐
                    │   quadruped_bridge    │
                    │   (MuJoCo Adapter)    │
                    └───────────┬───────────┘
                                │
                    ┌───────────▼───────────┐
                    │       MuJoCo          │
                    │   (ANYmal C Sim)      │
                    └───────────────────────┘
```

## Control Modes (FSM)

| Mode | Button | Description |
|------|--------|-------------|
| PASSIVE | A | No torques, robot free-falling |
| PD_CONTROL | B | Pure PD control to default stance |
| STANDING | X | MPC control with 4-leg stance |
| WALKING | Y | MPC control with gait switching |

## UDP Communication

### State Packet (Port 9101)
```cpp
struct StatePacket {  // 296 bytes
    uint64_t timestamp;      // Simulation time in nanoseconds
    double roll, pitch, yaw; // Euler angles
    double x, y, z;          // Base position
    double jointPos[12];     // Joint positions (OCS2 order)
    double angVelWorld[3];   // Angular velocity (world frame)
    double linVel[3];        // Linear velocity
    double jointVel[12];     // Joint velocities
};
```

### Command Packet (Port 9102)
```cpp
struct CommandPacket {  // 104 bytes
    uint64_t timestamp;
    double jointTorques[12]; // Joint torques (OCS2 order)
};
```

### Joint Order (OCS2)
```
LF: 0=HAA, 1=HFE, 2=KFE
LH: 3=HAA, 4=HFE, 5=KFE
RF: 6=HAA, 7=HFE, 8=KFE
RH: 9=HAA, 10=HFE, 11=KFE
```

## Build

```bash
cd ~/ros2_ws_ocs2
colcon build --packages-select quadruped_nmpc
source install/setup.bash
```

## Usage

1. Start MuJoCo with quadruped_bridge:
```bash
# In one terminal <cartpole,quadrotor,go1,go2,anymal_c,...>
cd ~/ros2_ws_ocs2/simulate/build
./mujoco_ocs <ROBOT_NAME>
```

2. Start the controller:
```bash
# In another terminal
source ~/ros2_ws_ocs2/install/setup.bash
ros2 run quadruped_nmpc quadruped_nmpc_main
```
```bash
./build/quadruped_nmpc/quadruped_nmpc_main
```
```bash
# ANYmal C (default)
./build/quadruped_nmpc/quadruped_nmpc_main 0

# Unitree Go1
./build/quadruped_nmpc/quadruped_nmpc_main 1

# Unitree Go2
./build/quadruped_nmpc/quadruped_nmpc_main 2
```


3. Use Xbox controller or keyboard to control:
   - Press **B** to go to PD_CONTROL (stand up)
   - Press **X** to enable STANDING (MPC control)
   - Press **Y** to enable WALKING (with gaits)
   - Use joysticks to control position/velocity


This script simulates the MuJoCo bridge and tests:
- **PASSIVE mode**: Expects zero torques
- **PD_CONTROL mode**: Expects torques to drive joints to default pose

## Configuration

- `config/mpc/task.info` - MPC settings
- `config/command/reference.info` - Reference trajectory settings
- `config/command/gait.info` - Gait definitions

## Dependencies

- ocs2_core
- ocs2_ddp
- ocs2_mpc
- ocs2_robotic_tools
- ocs2_pinocchio_interface
- ocs2_centroidal_model
- ocs2_robotic_assets
- Pinocchio
- Eigen3
- Boost

## License

BSD-3-Clause

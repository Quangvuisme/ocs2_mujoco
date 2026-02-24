# ANYmal Walking Control with OCS2 + MuJoCo

This package provides walking control for ANYmal C quadruped robot using OCS2 Centroidal Model Predictive Control (MPC) with MuJoCo physics simulation.

## Features

- **Centroidal MPC**: Nonlinear MPC with contact force optimization
- **Whole Body Control (WBC)**: RNEA-based torque computation
- **Gait Switching**: Support for multiple gaits (stance, trot, pace, walk)
- **Xbox Controller**: Full joystick control with mode switching
- **FSM Controller**: Finite State Machine for mode management

## Control Modes

| Button | Mode | Description |
|--------|------|-------------|
| A | PASSIVE | No torques, robot free-falling |
| B | PD_CONTROL | Pure PD control to default stance |
| X | STANDING | MPC + WBC with 4-leg stance |
| Y | WALKING | MPC + Gait switching (trot/pace/walk) |

## Joystick Mapping

### Standing Mode (X)
- Left stick Y: Forward/backward position
- Left stick X: Left/right position
- Right stick Y: Height adjustment (0.35-0.65m)
- Right stick X: Yaw rotation
- LB/RB: Height presets

### Walking Mode (Y)
- Left stick Y: Forward velocity (±0.5 m/s)
- Left stick X: Lateral velocity (±0.3 m/s)
- Right stick X: Yaw rate (±0.5 rad/s)
- RB: Switch gait (trot → pace → walk)

## Build

```bash
cd ~/ros2_ws_ocs2
colcon build --packages-select anymal_walking_online
```

## Run

```bash
source ./install/setup.bash
./build/anymal_walking_online/anymal_walking_mujoco
```

## Workflow

1. Start simulation: Robot spawns in PASSIVE mode
2. Press **B** to enter PD_CONTROL and stabilize
3. Press **X** for STANDING mode with MPC
4. Press **Y** for WALKING mode
5. Use left stick to control velocity
6. Press **RB** to switch gaits

## Configuration Files

- `config/mpc/task.info`: MPC parameters
- `config/command/reference.info`: Reference trajectory settings
- `config/command/gait.info`: Gait definitions (trot, pace, walk, etc.)
- `robots/anymal_c/`: URDF and MuJoCo models
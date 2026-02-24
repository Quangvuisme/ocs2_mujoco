# vm_anymal_standing_online - Comprehensive Documentation

## 📖 Table of Contents

This documentation provides a **detailed, tutorial-style explanation** of the `vm_anymal_standing_online` project, which implements a **Model Predictive Controller (MPC)** for standing control of the **ANYmal C quadruped robot** using the **OCS2 framework** and **MuJoCo simulation**.

---

## Document Structure

| Document | Description |
|----------|-------------|
| [01_ARCHITECTURE.md](./01_ARCHITECTURE.md) | High-level system architecture, component diagram, thread model |
| [02_MATH_FOUNDATIONS.md](./02_MATH_FOUNDATIONS.md) | Mathematical foundations: Centroidal Dynamics, RNEA, WBC formulation |
| [03_CODE_WALKTHROUGH.md](./03_CODE_WALKTHROUGH.md) | Line-by-line explanation of `main_anymal_mujoco.cpp` |
| [04_CONFIG_FILES.md](./04_CONFIG_FILES.md) | Detailed explanation of `task.info` and `reference.info` |
| [05_WBC_IMPLEMENTATION.md](./05_WBC_IMPLEMENTATION.md) | Whole Body Control implementation details |
| [06_COMPARISON_ETH.md](./06_COMPARISON_ETH.md) | Comparison with ETH RSL's perceptive_anymal |
| [07_DEBUGGING_GUIDE.md](./07_DEBUGGING_GUIDE.md) | Common issues and debugging techniques |

---

## Quick Start

```bash
# Build the project
cd /home/quangvd7/ros2_ws_ocs2
colcon build --packages-select vm_anymal_standing_online

# Source environment
source install/setup.bash

# Run simulation
ros2 run vm_anymal_standing_online anymal_mujoco
```

---

## Key Components

### 1. **OCS2 MPC Framework**
- Optimal Control for Switched Systems (OCS2)
- Centroidal Model for legged robots
- GaussNewtonDDP solver

### 2. **Whole Body Control (WBC)**
- Converts MPC contact forces to joint torques
- Uses RNEA (Recursive Newton-Euler Algorithm)
- Formula: `τ = RNEA(q, v, a + Kp*(q_des - q) + Kd*(v_des - v), f_ext)`

### 3. **MuJoCo Simulation**
- Physics engine for simulation
- GUI visualization
- Real-time control loop

---

## Project Structure

```
vm_anymal_standing_online/
├── config/
│   ├── command/
│   │   └── reference.info      # Reference trajectory settings
│   └── mpc/
│       └── task.info           # MPC parameters, Q/R weights
├── include/
│   └── vm_anymal_standing_online/
│       ├── LeggedRobotInterface.h
│       └── ... (other headers)
├── src/
│   ├── main_anymal_mujoco.cpp  # Main simulation file (784 lines)
│   ├── LeggedRobotInterface.cpp
│   └── ... (other sources)
└── robots/
    └── anymal_c/
        ├── urdf/anymal.urdf
        └── scene.xml           # MuJoCo model
```

---

## Control Flow Summary

```
┌─────────────────────────────────────────────────────────────────────┐
│                        MAIN EXECUTION FLOW                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. Initialize OCS2 Interface (LeggedRobotInterface)                │
│     └── Load URDF, task.info, reference.info                        │
│     └── Create PinocchioInterface                                   │
│     └── Setup OptimalControlProblem (dynamics, costs, constraints)  │
│                                                                     │
│  2. Initialize MPC (GaussNewtonDDP_MPC)                             │
│     └── Create MPC_MRT_Interface                                    │
│     └── Initialize CentroidalModelRbdConversions (for WBC)          │
│                                                                     │
│  3. Start Threads                                                   │
│     ├── MPC Thread (50 Hz): advanceMpc()                            │
│     ├── Physics Thread: MuJoCo simulation                           │
│     └── Control Thread (1000 Hz): WBC torque computation            │
│                                                                     │
│  4. Control Loop                                                    │
│     ├── [0-3s] PD control only (stabilization)                      │
│     └── [3s+] MPC + WBC control                                     │
│           ├── Get optimal state/input from MPC                      │
│           ├── Convert to joint torques via RNEA                     │
│           └── Apply torques to MuJoCo                               │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Author Notes

This documentation was created based on debugging sessions that resolved the "robot falling" issue. The key insight was that **MPC outputs contact forces, not joint torques**, and therefore a **Whole Body Controller (WBC)** is required to convert these forces to actuator commands.

**Important Formula:**
$$\tau = \text{RNEA}(q, \dot{q}, \ddot{q}_{cmd}, F_{contact})$$

Where:
- $\tau$ = joint torques (output)
- $q$ = generalized coordinates
- $\dot{q}$ = generalized velocities
- $\ddot{q}_{cmd}$ = commanded accelerations (with PD feedback)
- $F_{contact}$ = contact forces from MPC

---

**Next:** [01_ARCHITECTURE.md](./01_ARCHITECTURE.md) - System Architecture Overview

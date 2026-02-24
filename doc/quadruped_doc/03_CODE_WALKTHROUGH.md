# 03. Code Walkthrough - main_anymal_mujoco.cpp

This document provides a **line-by-line explanation** of the main simulation file.

## 📄 File Structure Overview

```
main_anymal_mujoco.cpp (784 lines)
├── Lines 1-99:     Includes & Global Variables
├── Lines 100-175:  mujocoToRbdState() - State conversion
├── Lines 176-189:  mujocoToOcs2State() - To centroidal state
├── Lines 190-225:  extractJointTargets() - Joint position extraction
├── Lines 226-252:  computeWbcTorques() - WBC computation
├── Lines 253-282:  extractJointTorques() - Torque mapping
├── Lines 283-311:  mpcThread() - MPC optimization loop
├── Lines 312-410:  controlThread() - Control loop
├── Lines 411-605:  physicsThread() - Physics simulation
└── Lines 606-784:  main() - Initialization & startup
```

---

## 📦 Section 1: Includes & Global Variables (Lines 1-99)

### Lines 1-45: System Includes

```cpp
#include <atomic>              // Thread-safe flags (mpcReady, exitRequest)
#include <chrono>              // Timing (sleep, duration)
#include <cmath>               // Math functions (atan2, sin, cos)
#include <fstream>             // File checking
#include <iostream>            // Console output
#include <memory>              // Smart pointers (unique_ptr)
#include <mutex>               // Thread synchronization
#include <string>              // String handling
#include <thread>              // Multi-threading
```

### Lines 46-60: MuJoCo Includes

```cpp
#include <mujoco/mujoco.h>              // MuJoCo physics engine
#include <mujoco/glfw_adapter.h>        // GLFW window adapter
#include <mujoco/simulate.h>            // GUI simulation class
```

**Why MuJoCo?**
- High-fidelity physics simulation
- GPU-accelerated rendering
- Built-in GUI for visualization
- Accurate contact/friction modeling

### Lines 61-75: OCS2 Includes

```cpp
#include <ocs2_centroidal_model/AccessHelperFunctions.h>   // State access helpers
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>  // WBC!
#include <ocs2_centroidal_model/ModelHelperFunctions.h>    // Gravity compensation
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>                   // DDP-based MPC
#include <ocs2_mpc/MPC_MRT_Interface.h>                    // Real-time interface
#include <ocs2_oc/oc_data/LoopshapingPrimalSolution.h>     // MPC solution type
```

### Lines 76-99: Global Variables

```cpp
// === Configuration ===
constexpr double KP = 80.0;             // PD position gain
constexpr double KD = 2.0;              // PD velocity gain
constexpr double MAX_TORQUE = 80.0;     // Maximum joint torque [Nm]
constexpr bool USE_MPC_TRAJECTORY = true;   // Use MPC output
constexpr bool USE_WBC_TORQUE = true;       // Use WBC (crucial!)

// === Pointers to OCS2 Objects ===
std::unique_ptr<LeggedRobotInterface> robotInterface;    // Main OCS2 interface
std::unique_ptr<GaussNewtonDDP_MPC> mpcSolver;           // MPC solver
std::unique_ptr<MPC_MRT_Interface> mpcMrt;               // Real-time wrapper
std::unique_ptr<CentroidalModelRbdConversions> rbdConversions;  // WBC converter

// === Thread Synchronization ===
std::atomic<bool> mpcReady{false};      // Flag: MPC initialized and running
std::atomic<bool> exitRequest{false};   // Flag: Request threads to exit
std::mutex g_mpcMutex;                  // Protects MPC data access
std::mutex g_torqueMutex;               // Protects torque data access

// === Shared Data ===
vector_t g_optimalState;                // Latest optimal state from MPC
vector_t g_optimalInput;                // Latest optimal input from MPC
Eigen::Matrix<double, 12, 1> g_torques; // Joint torques to apply
```

**Why these specific values?**
- `KP = 80.0`: High enough for stiff control, low enough to avoid oscillation
- `KD = 2.0`: Critical damping ratio ≈ √(KP/mass) for joint
- `MAX_TORQUE = 80.0`: ANYmal C actuator limit

---

## 🔄 Section 2: State Conversion Functions (Lines 100-189)

### Function: `mujocoToRbdState()` (Lines 100-175)

**Purpose:** Convert MuJoCo state to OCS2 RBD state format.

```cpp
vector_t mujocoToRbdState(const double* qpos, const double* qvel,
                          const CentroidalModelInfo& info) {
    const int nJoints = 12;
    vector_t rbdState = vector_t::Zero(2 * (6 + nJoints));  // 36 elements
```

**RBD State Structure (36 elements):**
```
[0-2]:   Base orientation (yaw, pitch, roll) - ZYX Euler
[3-5]:   Base position (x, y, z)
[6-17]:  Joint positions (OCS2 order)
[18-20]: Base angular velocity (wx, wy, wz)
[21-23]: Base linear velocity (vx, vy, vz)
[24-35]: Joint velocities
```

### Quaternion to Euler Conversion (Lines 108-117)

```cpp
    // MuJoCo quaternion: qpos[3]=w, qpos[4]=x, qpos[5]=y, qpos[6]=z
    double qw = qpos[3], qx = qpos[4], qy = qpos[5], qz = qpos[6];
    double yaw, pitch, roll;
    quatToZYXEuler(qw, qx, qy, qz, yaw, pitch, roll);
    
    rbdState(0) = yaw;    // Rotation around Z (vertical)
    rbdState(1) = pitch;  // Rotation around Y (lateral)
    rbdState(2) = roll;   // Rotation around X (forward)
```

**Why ZYX Euler?**
- OCS2/Pinocchio uses `SphericalZYX` base convention
- Yaw-Pitch-Roll is intuitive for legged robots
- No gimbal lock for typical walking poses

### Joint Reordering (Lines 124-148)

```cpp
    // CRITICAL: Joint ordering differs between MuJoCo and OCS2!
    // MuJoCo: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
    // OCS2:   LF(0-2), LH(3-5), RF(6-8), RH(9-11)
    
    // LF -> LF (same)
    rbdState(6 + 0) = qpos[7 + 0];  // LF_HAA
    rbdState(6 + 1) = qpos[7 + 1];  // LF_HFE
    rbdState(6 + 2) = qpos[7 + 2];  // LF_KFE
    
    // LH: OCS2 index 3-5, MuJoCo index 6-8
    rbdState(6 + 3) = qpos[7 + 6];  // LH_HAA
    rbdState(6 + 4) = qpos[7 + 7];  // LH_HFE
    rbdState(6 + 5) = qpos[7 + 8];  // LH_KFE
    
    // RF: OCS2 index 6-8, MuJoCo index 3-5
    rbdState(6 + 6) = qpos[7 + 3];  // RF_HAA
    rbdState(6 + 7) = qpos[7 + 4];  // RF_HFE
    rbdState(6 + 8) = qpos[7 + 5];  // RF_KFE
    
    // RH -> RH (same)
    rbdState(6 + 9) = qpos[7 + 9];   // RH_HAA
    rbdState(6 + 10) = qpos[7 + 10]; // RH_HFE
    rbdState(6 + 11) = qpos[7 + 11]; // RH_KFE
```

**Why the reordering?**
- MuJoCo URDF: `LF, RF, LH, RH` (front-back, left-right)
- OCS2 model: `LF, LH, RF, RH` (left-right, front-back)
- Incorrect ordering = wrong torques to wrong joints!

### Function: `mujocoToOcs2State()` (Lines 176-189)

```cpp
vector_t mujocoToOcs2State(const double* qpos, const double* qvel,
                           const CentroidalModelInfo& info) {
    vector_t rbdState = mujocoToRbdState(qpos, qvel, info);
    return rbdConversions->computeCentroidalStateFromRbdModel(rbdState);
}
```

**What does `computeCentroidalStateFromRbdModel` do?**
1. Converts RBD coordinates to Pinocchio format
2. Computes Centroidal Momentum Matrix $A_G$
3. Computes normalized momentum $\tilde{h} = A_G \dot{q} / m$
4. Returns 24-dim centroidal state

---

## 🎮 Section 3: WBC Functions (Lines 190-282)

### Function: `computeWbcTorques()` (Lines 226-252)

**THE MOST IMPORTANT FUNCTION!**

```cpp
vector_t computeWbcTorques(const vector_t& desiredState,
                           const vector_t& desiredInput,
                           const vector_t& measuredRbdState) {
    if (!rbdConversions) {
        return vector_t::Zero(18);  // Safety: return zeros if not initialized
    }
    
    const auto& info = robotInterface->getCentroidalModelInfo();
    
    // Zero accelerations for standing (quasi-static)
    vector_t jointAccelerations = vector_t::Zero(info.actuatedDofNum);
```

**Why zero accelerations?**
- Standing is quasi-static (no desired acceleration)
- PD gains in WBC provide feedback acceleration

### PD Gains Setup (Lines 238-245)

```cpp
    // PD gains: [base(6), joints(12)]
    vector_t pGains = vector_t::Zero(info.generalizedCoordinatesNum);  // 18
    vector_t dGains = vector_t::Zero(info.generalizedCoordinatesNum);  // 18
    
    // Base gains are zero (6 DOF) - we don't directly control base
    // Joint gains (indices 6-17)
    for (int i = 0; i < 12; i++) {
        pGains(6 + i) = 100.0;   // Position gain for joints
        dGains(6 + i) = 5.0;     // Velocity gain for joints
    }
```

**Why base gains = 0?**
- Base is floating (unactuated)
- We control base indirectly through contact forces
- Setting non-zero would create impossible torques

### RNEA Call (Lines 247-252)

```cpp
    // τ = RNEA(q, v, a + Kp*e_q + Kd*e_v, F_ext)
    return rbdConversions->computeRbdTorqueFromCentroidalModelPD(
        desiredState, desiredInput, jointAccelerations,
        measuredRbdState, pGains, dGains);
}
```

**What happens inside?**
1. Extract desired $q_{des}, \dot{q}_{des}$ from centroidal state
2. Extract measured $q_{meas}, \dot{q}_{meas}$ from RBD state
3. Compute feedback: $\ddot{q}_{fb} = K_p(q_{des} - q_{meas}) + K_d(\dot{q}_{des} - \dot{q}_{meas})$
4. Transform contact forces to end-effector wrenches
5. Call Pinocchio RNEA: `pinocchio::rnea(model, data, q, v, a_cmd, f_ext)`
6. Return $[\tau_{base}, \tau_{joints}]$ (18-dim)

### Function: `extractJointTorques()` (Lines 253-282)

```cpp
void extractJointTorques(const vector_t& wbcTorques,
                         Eigen::Matrix<double, 12, 1>& mujocoTorques) {
    // WBC output: [base_wrench(6), joint_torques_OCS2_order(12)]
    // Skip base wrench, reorder joints for MuJoCo
    
    // LF: OCS2[6-8] -> MuJoCo[0-2]
    mujocoTorques(0) = wbcTorques(6 + 0);
    mujocoTorques(1) = wbcTorques(6 + 1);
    mujocoTorques(2) = wbcTorques(6 + 2);
    
    // RF: OCS2[12-14] -> MuJoCo[3-5]
    mujocoTorques(3) = wbcTorques(6 + 6);
    mujocoTorques(4) = wbcTorques(6 + 7);
    mujocoTorques(5) = wbcTorques(6 + 8);
    
    // LH: OCS2[9-11] -> MuJoCo[6-8]
    mujocoTorques(6) = wbcTorques(6 + 3);
    mujocoTorques(7) = wbcTorques(6 + 4);
    mujocoTorques(8) = wbcTorques(6 + 5);
    
    // RH: OCS2[15-17] -> MuJoCo[9-11]
    mujocoTorques(9) = wbcTorques(6 + 9);
    mujocoTorques(10) = wbcTorques(6 + 10);
    mujocoTorques(11) = wbcTorques(6 + 11);
}
```

---

## 🧵 Section 4: Thread Functions (Lines 283-605)

### MPC Thread (Lines 283-311)

```cpp
void mpcThread() {
    const double mpcPeriod = 1.0 / robotInterface->mpcSettings().mpcDesiredFrequency_;
    // mpcDesiredFrequency = 50 Hz → period = 0.02s
    
    while (!exitRequest) {
        if (elapsed >= mpcPeriod && mpcReady.load()) {
            std::lock_guard<std::mutex> lock(g_mpcMutex);
            mpcMrt->advanceMpc();  // Run one MPC iteration
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
```

**What does `advanceMpc()` do?**
1. Get current observation (state, time)
2. Solve OCP over horizon [t, t+T]
3. Update internal policy
4. Policy ready for evaluation

### Control Thread (Lines 312-410)

```cpp
void controlThread(mujoco::Simulate* sim) {
    // Default standing joint positions
    double defaultJoints[12] = {
        -0.25,  0.60, -0.85,   // LF: HAA, HFE, KFE
         0.25,  0.60, -0.85,   // RF
        -0.25, -0.60,  0.85,   // LH (note: HFE/KFE signs differ!)
         0.25, -0.60,  0.85    // RH
    };
```

**Why these joint values?**
- Produce standing pose at z ≈ 0.575m
- HAA: ±0.25 rad → legs slightly spread
- HFE: ±0.60 rad → thigh angle
- KFE: ±0.85 rad → knee bend

### Main Control Logic (Lines 350-395)

```cpp
    if (USE_WBC_TORQUE && useMpc && rbdConversions) {
        // WBC mode: Use MPC contact forces
        vector_t wbcTorques = computeWbcTorques(desiredState, desiredInput,
                                                 measuredRbdState);
        extractJointTorques(wbcTorques, torques);
    } else {
        // PD mode: Direct joint control
        for (int i = 0; i < 12; i++) {
            double pos = sim->d_->qpos[7 + i];
            double vel = sim->d_->qvel[6 + i];
            torques(i) = KP * (targetJoints[i] - pos) - KD * vel;
        }
    }
    
    // Torque saturation
    for (int i = 0; i < 12; i++) {
        torques(i) = std::clamp(torques(i), -MAX_TORQUE, MAX_TORQUE);
    }
```

**Control Flow:**
```
            ┌──────────────────┐
            │ MPC Ready?       │
            └────────┬─────────┘
                     │
          ┌──────────┴──────────┐
          │ Yes                 │ No
          ▼                     ▼
    ┌───────────┐         ┌───────────┐
    │ WBC Mode  │         │ PD Mode   │
    │           │         │           │
    │ τ = RNEA  │         │ τ = Kp*e  │
    │ (forces)  │         │ - Kd*v    │
    └───────────┘         └───────────┘
          │                     │
          └──────────┬──────────┘
                     ▼
              ┌─────────────┐
              │ Clamp τ     │
              │ [-80, 80]   │
              └─────────────┘
```

### Physics Thread (Lines 411-605)

**Model Loading (Lines 430-460):**
```cpp
    mjModel* m = mj_loadXML(modelPath.c_str(), nullptr, error, 1000);
    mjData* d = mj_makeData(m);
    
    // Initial pose
    d->qpos[2] = 0.57;   // Standing height
    d->qpos[3] = 1.0;    // Quaternion w (identity rotation)
```

**MPC Initialization Delay (Lines 540-570):**
```cpp
    double mpcStartDelay = 3.0;  // Wait 3 seconds
    
    if (!mpcStarted && sim->d_->time > mpcStartDelay) {
        // Initialize MPC with current stabilized state
        vector_t initState = mujocoToOcs2State(...);
        
        // Set gravity-compensating reference
        vector_t gravCompInput = weightCompensatingInput(info, contactFlags);
        
        mpcReady.store(true);
        mpcStarted = true;
    }
```

**Why 3-second delay?**
1. Let physics settle initial transients
2. PD control stabilizes the robot first
3. MPC starts from a "good" initial state
4. Avoids MPC divergence from bad initialization

---

## 🚀 Section 5: Main Function (Lines 606-784)

### Initialization Sequence

```cpp
int main(int argc, char** argv) {
    // [1] Setup paths
    std::string basePath = getPath();
    g_taskFile = basePath + "/config/mpc/task.info";
    g_urdfFile = basePath + "/robots/anymal_c/urdf/anymal.urdf";
    g_referenceFile = basePath + "/config/command/reference.info";
    g_mujocoModel = basePath + "/robots/anymal_c/scene.xml";
```

### OCS2 Interface Initialization (Lines 680-700)

```cpp
    // [2] Initialize OCS2
    robotInterface = std::make_unique<LeggedRobotInterface>(
        g_taskFile, g_urdfFile, g_referenceFile,
        useHardFrictionConeConstraint);
    
    // [3] Initialize WBC converter
    rbdConversions = std::make_unique<CentroidalModelRbdConversions>(
        robotInterface->getPinocchioInterface(),
        robotInterface->getCentroidalModelInfo());
```

### MPC Initialization (Lines 705-730)

```cpp
    // [4] Initialize MPC solver
    mpcSolver = std::make_unique<GaussNewtonDDP_MPC>(
        robotInterface->mpcSettings(),
        robotInterface->ddpSettings(),
        robotInterface->getRollout(),
        robotInterface->getOptimalControlProblem(),
        robotInterface->getInitializer());
    
    // [5] Create real-time interface
    mpcMrt = std::make_unique<MPC_MRT_Interface>(*mpcSolver);
    mpcMrt->initRollout(&robotInterface->getRollout());
```

### Thread Startup (Lines 760-770)

```cpp
    std::thread mpcWorker(mpcThread);
    std::thread physicsWorker(physicsThread, sim.get(), g_mujocoModel);
    std::thread controlWorker(controlThread, sim.get());
    
    // Run GUI in main thread
    sim->RenderLoop();
```

**Thread Startup Order:**
1. MPC thread (waits for `mpcReady`)
2. Physics thread (loads model, runs simulation)
3. Control thread (waits for model load)
4. Main thread handles GUI

---

## 📊 Key Execution Flow Summary

```
Time
│
├── t=0.0s: Program starts
│   ├── Load config files
│   ├── Initialize OCS2 interface
│   ├── Initialize MPC solver
│   └── Start threads
│
├── t=0.0-3.0s: PD Control Phase
│   ├── Physics: Robot settles under gravity
│   ├── Control: PD to default joint positions
│   └── MPC: Waiting (mpcReady = false)
│
├── t=3.0s: MPC Enabled
│   ├── Physics: Sets initial observation
│   ├── Control: Switches to WBC mode
│   └── MPC: Starts optimization loop
│
├── t=3.0s+: MPC+WBC Control Phase
│   ├── MPC (50 Hz): advanceMpc()
│   ├── Control (1000 Hz): evaluatePolicy() + WBC
│   └── Physics: Apply torques, step simulation
│
└── t=∞: Shutdown
    └── exitRequest = true, join threads
```

---

**Next:** [04_CONFIG_FILES.md](./04_CONFIG_FILES.md) - Configuration File Explanation

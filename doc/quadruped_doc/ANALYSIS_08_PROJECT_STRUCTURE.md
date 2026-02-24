# Analysis 08: Project Structure & OCS2 Dependencies

## 1. Workspace Overview

```
ros2_ws_ocs2/
├── src/
│   ├── vm_anymal_standing_online/     # Main project (ANYmal standing control)
│   ├── ocs2_ros2/                     # OCS2 framework (dependency)
│   └── ...other packages
├── build/                              # Build artifacts
├── install/                            # Installed packages
├── log/                                # Build logs
└── doc/                                # Documentation
```

## 2. vm_anymal_standing_online Structure

### 2.1 Directory Layout

```
vm_anymal_standing_online/
├── CMakeLists.txt                 # Build configuration
├── package.xml                    # ROS2 package manifest
├── README.md                      # Package documentation
│
├── config/                        # Configuration files
│   ├── mpc/
│   │   └── task.info             # MPC settings, costs, constraints
│   └── command/
│       └── reference.info        # Initial state, gait settings
│
├── robots/                        # Robot models
│   └── anymal_c/
│       ├── urdf/
│       │   └── anymal.urdf       # Robot URDF model
│       └── scene.xml             # MuJoCo scene
│
├── include/vm_anymal_standing_online/   # Header files
│   ├── LeggedRobotInterface.h          # Main interface
│   ├── LeggedRobotPreComputation.h     # Pre-computation cache
│   ├── common/                          # Common utilities
│   ├── constraint/                      # Constraint definitions
│   ├── cost/                            # Cost function definitions
│   ├── dynamics/                        # System dynamics
│   ├── foot_planner/                    # Swing trajectory planning
│   ├── gait/                            # Gait definitions
│   ├── initialization/                  # Initial trajectory
│   └── reference_manager/               # Reference management
│
└── src/                           # Source files
    ├── main_anymal_mujoco.cpp    # Main entry point
    ├── LeggedRobotInterface.cpp  # Interface implementation
    ├── LeggedRobotPreComputation.cpp
    ├── common/
    ├── constraint/
    ├── dynamics/
    ├── foot_planner/
    ├── gait/
    ├── initialization/
    ├── reference_manager/
    └── mujoco_gui/               # MuJoCo visualization
```

### 2.2 Source Code Components

#### A. Main Entry Point
| File | Purpose |
|------|---------|
| `main_anymal_mujoco.cpp` | Main simulation with MuJoCo + MPC + Xbox/FSM |

#### B. Interface Layer (`LeggedRobotInterface`)
| File | Purpose | OCS2 Dependency |
|------|---------|-----------------|
| `LeggedRobotInterface.h/cpp` | Main interface, creates OCP | `ocs2_mpc`, `ocs2_ddp`, `ocs2_centroidal_model` |
| `LeggedRobotPreComputation.h/cpp` | Caches kinematics, Jacobians | `ocs2_oc/PreComputation` |

#### C. Dynamics (`dynamics/`)
| File | Purpose | OCS2 Dependency |
|------|---------|-----------------|
| `LeggedRobotDynamicsAD.cpp` | Centroidal dynamics with AD | `ocs2_centroidal_model`, `CppAD` |

**Key Concept:**
```cpp
// Centroidal dynamics: ẋ = f(x, u)
// State: [h_normalized, q_base, v_base, q_joints] (24D)
// Input: [f_LF, f_RF, f_LH, f_RH, q̈_joints] (24D)
```

#### D. Constraints (`constraint/`)
| File | Purpose | OCS2 Dependency |
|------|---------|-----------------|
| `FrictionConeConstraint.cpp` | Friction cone constraint | `ocs2_core/StateInputConstraint` |
| `ZeroForceConstraint.cpp` | Zero contact force in swing | `ocs2_core/StateInputConstraint` |
| `ZeroVelocityConstraintCppAd.cpp` | Zero foot velocity in stance | `PinocchioEndEffectorKinematicsCppAd` |
| `NormalVelocityConstraintCppAd.cpp` | Normal velocity constraint | `PinocchioEndEffectorKinematicsCppAd` |
| `EndEffectorLinearConstraint.cpp` | Linear EE constraints | `ocs2_core/StateInputConstraint` |

**Constraint Types:**
```
Standing Mode (STANCE = 15):
├── All 4 feet: ZeroVelocityConstraint (feet don't move)
├── All 4 feet: FrictionConeConstraint (f_t ≤ μ·f_n)
└── Swing feet: ZeroForceConstraint (f = 0 when not in contact)
```

#### E. Cost Functions (`cost/`)
| File | Purpose | OCS2 Dependency |
|------|---------|-----------------|
| `LeggedRobotQuadraticTrackingCost.h` | Quadratic tracking cost | `ocs2_core/QuadraticStateCost` |

**Cost Function:**
```
J = ∫ (x - x_ref)ᵀ Q (x - x_ref) + (u - u_ref)ᵀ R (u - u_ref) dt
```

#### F. Gait System (`gait/`)
| File | Purpose | Description |
|------|---------|-------------|
| `Gait.cpp` | Gait definition | Duration, event phases, mode sequence |
| `GaitSchedule.cpp` | Gait scheduling | Tiles gaits over time horizon |
| `LegLogic.cpp` | Leg contact logic | Maps mode number to contact flags |
| `ModeSequenceTemplate.cpp` | Mode templates | Gait pattern templates |
| `MotionPhaseDefinition.h` | Mode numbers | 16 contact modes (0-15) |

**Mode Numbers:**
```cpp
enum ModeNumber {
    FLY = 0,          // No contact
    RH = 1,           // Only RH in contact
    LH = 2,           // Only LH in contact
    LH_RH = 3,        // LH + RH (trot back)
    RF = 4,           // Only RF
    RF_RH = 5,        // RF + RH (pace right)
    RF_LH = 6,        // RF + LH (diagonal)
    RF_LH_RH = 7,     // 3-leg stance
    LF = 8,           // Only LF
    LF_RH = 9,        // LF + RH (diagonal)
    LF_LH = 10,       // LF + LH (pace left)
    LF_LH_RH = 11,    // 3-leg stance
    LF_RF = 12,       // LF + RF (trot front)
    LF_RF_RH = 13,    // 3-leg stance
    LF_RF_LH = 14,    // 3-leg stance
    STANCE = 15,      // All 4 feet in contact
};
```

#### G. Foot Planner (`foot_planner/`)
| File | Purpose | Description |
|------|---------|-------------|
| `SwingTrajectoryPlanner.cpp` | Swing foot trajectory | Plans Z-height during swing |
| `CubicSpline.cpp` | Cubic spline interpolation | Smooth swing trajectories |
| `SplineCpg.cpp` | CPG-based splines | Central Pattern Generator |

**Swing Trajectory:**
```
Lift-off → Mid-height → Touch-down
    ↑          ↑            ↑
  0.0m      0.15m         0.0m
```

#### H. Reference Manager (`reference_manager/`)
| File | Purpose | OCS2 Dependency |
|------|---------|-----------------|
| `SwitchedModelReferenceManager.cpp` | Manages mode schedule & targets | `ocs2_oc/ReferenceManager` |

**Key Functions:**
```cpp
void modifyReferences(initTime, finalTime, initState, 
                      targetTrajectories, modeSchedule) {
    // 1. Get mode schedule from gait scheduler
    modeSchedule = gaitSchedulePtr_->getModeSchedule(...);
    
    // 2. Update swing trajectory planner
    swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}
```

#### I. Initialization (`initialization/`)
| File | Purpose | OCS2 Dependency |
|------|---------|-----------------|
| `LeggedRobotInitializer.cpp` | Initial trajectory guess | `ocs2_oc/Initializer` |

#### J. Common Utilities (`common/`)
| File | Purpose |
|------|---------|
| `ModelSettings.cpp` | Load model settings from task.info |
| `Types.h` | Type definitions (contact_flag_t, feet_array_t) |
| `utils.h` | Utility functions |

---

## 3. OCS2 Framework Structure

### 3.1 Core Libraries (`ocs2_ros2/core/`)

```
ocs2_core/
├── Types.h                    # vector_t, matrix_t, scalar_t
├── Dynamics/
│   ├── SystemDynamicsBase.h   # Base class for dynamics
│   └── LinearSystemDynamics.h
├── Constraint/
│   ├── StateConstraint.h
│   ├── StateInputConstraint.h
│   └── RelaxedBarrierPenalty.h
├── Cost/
│   ├── QuadraticStateCost.h
│   └── QuadraticStateInputCost.h
├── Misc/
│   ├── LoadData.h             # Load from .info files
│   └── Lookup.h               # Time lookup utilities
└── Integration/               # ODE integrators

ocs2_oc/
├── OptimalControlProblem.h    # OCP definition
├── PreComputation.h           # Caching interface
├── ReferenceManager.h         # Target trajectory management
├── Rollout/                   # Simulation rollout
└── StateTriggeredRollout.h    # Event-triggered rollout
```

### 3.2 MPC Solvers (`ocs2_ros2/mpc/`)

```
ocs2_ddp/
├── GaussNewtonDDP_MPC.h       # Main DDP MPC solver (used in standing)
├── DDP_Settings.h             # DDP configuration
└── ILQR.h                     # Iterative LQR

ocs2_sqp/
├── SqpMpc.h                   # SQP-based MPC
└── SqpSettings.h

ocs2_ipm/
├── IpmMpc.h                   # Interior Point Method MPC
└── IpmSettings.h

ocs2_mpc/
├── MPC_BASE.h                 # Base MPC class
├── MPC_MRT_Interface.h        # MPC-MRT interface (real-time)
└── MPC_Settings.h
```

### 3.3 Robotics Libraries (`ocs2_ros2/robotics/`)

```
ocs2_pinocchio/
├── ocs2_pinocchio_interface/
│   ├── PinocchioInterface.h           # Pinocchio wrapper
│   └── PinocchioInterfaceCppAd.h      # Auto-diff version
├── ocs2_centroidal_model/
│   ├── CentroidalModelInfo.h          # Model info struct
│   ├── CentroidalModelRbdConversions.h # State conversions
│   ├── CentroidalModelPinocchioMapping.h
│   └── AccessHelperFunctions.h        # State/input access
└── ocs2_self_collision/               # Self-collision avoidance

ocs2_robotic_tools/
├── EndEffectorKinematics.h    # EE kinematics interface
└── common/                    # Common robotics utilities
```

---

## 4. Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    vm_anymal_standing_online                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │ task.info    │    │ anymal.urdf  │    │reference.info│       │
│  │ (MPC config) │    │ (Robot model)│    │(Initial/Gait)│       │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘       │
│         │                   │                   │                │
│         └───────────────────┼───────────────────┘                │
│                             ▼                                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              LeggedRobotInterface                         │   │
│  │  ┌─────────────────────────────────────────────────────┐ │   │
│  │  │ Creates:                                             │ │   │
│  │  │ • PinocchioInterface (kinematics/dynamics)          │ │   │
│  │  │ • CentroidalModelInfo (state/input dims)            │ │   │
│  │  │ • OptimalControlProblem (costs, constraints)        │ │   │
│  │  │ • SwitchedModelReferenceManager (gait, targets)     │ │   │
│  │  │ • LeggedRobotInitializer (initial guess)            │ │   │
│  │  │ • Rollout (simulation)                              │ │   │
│  │  └─────────────────────────────────────────────────────┘ │   │
│  └──────────────────────────┬───────────────────────────────┘   │
│                             │                                    │
│                             ▼                                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              GaussNewtonDDP_MPC (OCS2)                    │   │
│  │  • Solves OCP at each MPC cycle                          │   │
│  │  • Returns optimal state/input trajectories              │   │
│  └──────────────────────────┬───────────────────────────────┘   │
│                             │                                    │
│                             ▼                                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │         CentroidalModelRbdConversions (WBC)              │   │
│  │  • Converts contact forces → joint torques               │   │
│  │  • τ = RNEA(q, v, a, f_ext)                             │   │
│  └──────────────────────────┬───────────────────────────────┘   │
│                             │                                    │
│                             ▼                                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    MuJoCo Simulation                      │   │
│  │  • Applies joint torques                                  │   │
│  │  • Steps physics                                          │   │
│  │  • Returns sensor data                                    │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 5. Key OCS2 Components Used

### 5.1 From `ocs2_centroidal_model`

```cpp
// State representation (24D for ANYmal)
// [0-2]: Normalized angular momentum (h_ω / m)
// [3-5]: Normalized linear momentum (v_CoM)
// [6-8]: Base position (x, y, z)
// [9-11]: Base orientation (ZYX Euler)
// [12-23]: Joint positions (12 joints)

// Input representation (24D)
// [0-11]: Contact forces (3 per foot × 4 feet)
// [12-23]: Joint accelerations
```

### 5.2 From `ocs2_ddp`

```cpp
// DDP solver settings
ddp.algorithm = SLQ           // Sequential Linear Quadratic
ddp.nThreads = 4              // Parallel threads
ddp.maxNumIterations = 10     // Max iterations per MPC cycle
ddp.minRelCost = 1e-3         // Convergence criteria
```

### 5.3 From `ocs2_pinocchio_interface`

```cpp
// Pinocchio for kinematics/dynamics
PinocchioInterface pinocchio(urdf);

// End-effector kinematics with auto-diff
PinocchioEndEffectorKinematicsCppAd eeKinematics(
    pinocchio, mapping, {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"});
```

---

## 6. Configuration Files

### 6.1 task.info (MPC Configuration)

```ini
; MPC settings
mpc.timeHorizon = 1.0          ; Planning horizon [s]
mpc.solutionTimeWindow = 0.02  ; Solution validity window
mpc.mpcDesiredFrequency = 100  ; MPC update rate [Hz]

; DDP settings
ddp.algorithm = SLQ
ddp.maxNumIterations = 10

; Cost weights
Q.diagonal = (100, 100, 100, ...)  ; State cost
R.diagonal = (0.01, 0.01, ...)    ; Input cost

; Friction cone
frictionCoefficient = 0.7
```

### 6.2 reference.info (Initial State & Gait)

```ini
; Initial state
initialState = (...)  ; 24D initial state

; Default joint positions
defaultJointState = (-0.25, 0.60, -0.85, ...)

; Gait settings
gaitSequence = STANCE  ; Standing gait (mode 15)
```

---

## 7. Summary Table

| Component | Location | OCS2 Dependency | Purpose |
|-----------|----------|-----------------|---------|
| Interface | `LeggedRobotInterface` | `ocs2_mpc`, `ocs2_ddp` | Creates OCP |
| Dynamics | `dynamics/` | `ocs2_centroidal_model` | System equations |
| Constraints | `constraint/` | `ocs2_core` | Physical limits |
| Costs | `cost/` | `ocs2_core` | Tracking objective |
| Gait | `gait/` | Custom | Contact scheduling |
| Foot Planner | `foot_planner/` | Custom | Swing trajectories |
| Reference | `reference_manager/` | `ocs2_oc` | Target management |
| WBC | `CentroidalModelRbdConversions` | `ocs2_centroidal_model` | Force→Torque |

# 05. Whole Body Control (WBC) Implementation

## 🎯 The Core Problem

MPC outputs **contact forces**, but robot actuators need **joint torques**.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        THE WBC PROBLEM                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   MPC Output:              We Need:                                         │
│   ───────────              ────────                                         │
│                                                                             │
│   Contact Forces           Joint Torques                                    │
│                                                                             │
│       F_LF = [0, 0, 128]N      τ_LF = [τ1, τ2, τ3]                          │
│       F_RF = [0, 0, 128]N      τ_RF = [τ4, τ5, τ6]                          │
│       F_LH = [0, 0, 128]N      τ_LH = [τ7, τ8, τ9]                          │
│       F_RH = [0, 0, 128]N      τ_RH = [τ10, τ11, τ12]                       │
│                                                                             │
│       ↓                        ↑                                            │
│       └────────── WBC ─────────┘                                            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## ❌ Wrong Approach: Simple Jacobian Transpose

### The Naive Solution

$$\tau = -J^T F$$

Where $J$ is the foot Jacobian mapping joint velocities to foot velocities.

### Why It Fails

```cpp
// DON'T DO THIS! (It was our first attempt)
for (int leg = 0; leg < 4; leg++) {
    Matrix3 J = pinocchio::getFrameJacobian(...);  // 3x3 for 3-DOF leg
    Vector3 F = contactForces.segment<3>(3*leg);
    torques.segment<3>(3*leg) = -J.transpose() * F;
}
```

**Problems:**
1. **Ignores gravity:** Robot would sag under its own weight
2. **Ignores Coriolis forces:** Wrong during motion
3. **Ignores leg inertia:** Accelerations not accounted for
4. **No feedback:** Open-loop, no error correction

**Result:** Robot collapses! (Our original bug)

---

## ✅ Correct Approach: RNEA with External Forces

### The Full Inverse Dynamics

$$\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) - J^T F_{ext}$$

Where:
- $M(q)$: Mass/inertia matrix
- $C(q, \dot{q})$: Coriolis/centrifugal terms
- $G(q)$: Gravity compensation
- $J^T F_{ext}$: Contact force contribution

### Pinocchio's RNEA

The **Recursive Newton-Euler Algorithm** efficiently computes this:

```cpp
tau = pinocchio::rnea(model, data, q, v, a, f_ext);
```

**Inputs:**
- `model`: Robot model (from URDF)
- `data`: Pinocchio data structure
- `q`: Generalized coordinates (18: 6 base + 12 joints)
- `v`: Generalized velocities (18)
- `a`: Desired accelerations (18)
- `f_ext`: External forces at each joint frame

**Output:**
- `tau`: Generalized forces (18: 6 base wrench + 12 joint torques)

---

## 📝 OCS2's CentroidalModelRbdConversions

### Class Purpose

Converts between:
- OCS2 centroidal state/input ↔ Pinocchio RBD quantities
- MPC policy ↔ Joint torques for execution

### Key Function: `computeRbdTorqueFromCentroidalModelPD`

**File:** `ocs2_centroidal_model/src/CentroidalModelRbdConversions.cpp`

```cpp
vector_t CentroidalModelRbdConversions::computeRbdTorqueFromCentroidalModelPD(
    const vector_t& desiredState,           // MPC optimal state (24-dim)
    const vector_t& desiredInput,           // MPC optimal input (24-dim)
    const vector_t& desiredJointAccelerations,  // Usually zeros for standing
    const vector_t& measuredRbdState,       // Current robot state (36-dim)
    const vector_t& pGains,                 // Position gains (18-dim)
    const vector_t& dGains)                 // Velocity gains (18-dim)
```

### Step-by-Step Breakdown

#### Step 1: Compute Desired Kinematics

```cpp
Vector6 desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration;
computeBaseKinematicsFromCentroidalModel(
    desiredState, desiredInput, desiredJointAccelerations,
    desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration);
```

This uses the Centroidal Momentum Matrix to compute base motion from centroidal state.

#### Step 2: Build Desired Configuration

```cpp
vector_t qDesired(18), vDesired(18), aDesired(18);
qDesired << desiredBasePose, centroidal_model::getJointAngles(desiredState, info);
vDesired << desiredBaseVelocity, centroidal_model::getJointVelocities(desiredInput, info);
aDesired << desiredBaseAcceleration, desiredJointAccelerations;
```

#### Step 3: Transform Contact Forces to End-Effector Wrenches

```cpp
pinocchio::container::aligned_vector<pinocchio::Force> fextDesired(model.njoints);

for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
    const auto jointIndex = model.frames[frameIndex].parentJoint;
    
    // Transform force from world frame to joint frame
    const Matrix3 rotationWorldToJoint = data.oMi[jointIndex].rotation().transpose();
    const Vector3 contactForce = rotationWorldToJoint * 
        centroidal_model::getContactForces(desiredInput, i, info);
    
    // Compute moment from offset
    const Vector3 offset = model.frames[frameIndex].placement.translation();
    
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = offset.cross(contactForce);
}
```

**Important:** Forces must be in the **parent joint frame**, not world frame!

#### Step 4: Extract Measured Configuration

```cpp
vector_t qMeasured(18), vMeasured(18);
// Convert from RBD state format (euler,pos,joints, angvel,linvel,jointvel)
qMeasured.head<3>() = measuredRbdState.segment<3>(3);  // Position
qMeasured.segment<3>(3) = measuredRbdState.head<3>();  // Euler angles
qMeasured.tail(12) = measuredRbdState.segment(6, 12);  // Joint angles

vMeasured.head<3>() = measuredRbdState.segment<3>(21); // Linear velocity
vMeasured.segment<3>(3) = transform(measuredRbdState.segment<3>(18)); // Angular
vMeasured.tail(12) = measuredRbdState.segment(24, 12); // Joint velocities
```

#### Step 5: Compute PD Feedback

```cpp
const vector_t pdFeedback = pGains.cwiseProduct(qDesired - qMeasured) 
                          + dGains.cwiseProduct(vDesired - vMeasured);
```

This creates a position/velocity error signal that gets added to accelerations.

#### Step 6: Call RNEA

```cpp
const vector_t aAugmented = aDesired + pdFeedback;
return pinocchio::rnea(model, data, qDesired, vDesired, aAugmented, fextDesired);
```

**The magic:** By adding PD feedback to accelerations, RNEA produces torques that:
1. Achieve the desired feedforward trajectory (from `aDesired`)
2. Correct errors (from `pdFeedback`)
3. Account for dynamics (mass, gravity, Coriolis)
4. Include contact force effects (from `fextDesired`)

---

## 🔧 Our Implementation in main_anymal_mujoco.cpp

### computeWbcTorques Function

```cpp
vector_t computeWbcTorques(const vector_t& desiredState, 
                           const vector_t& desiredInput,
                           const vector_t& measuredRbdState) {
    const auto& info = robotInterface->getCentroidalModelInfo();
    
    // Zero accelerations for quasi-static standing
    vector_t jointAccelerations = vector_t::Zero(info.actuatedDofNum);  // 12
    
    // PD gains: [base(6), joints(12)]
    vector_t pGains = vector_t::Zero(18);  // Base gains = 0 (unactuated)
    vector_t dGains = vector_t::Zero(18);
    
    // Only joint gains are non-zero
    for (int i = 0; i < 12; i++) {
        pGains(6 + i) = 100.0;  // Position gain
        dGains(6 + i) = 5.0;    // Velocity gain
    }
    
    return rbdConversions->computeRbdTorqueFromCentroidalModelPD(
        desiredState, desiredInput, jointAccelerations,
        measuredRbdState, pGains, dGains);
}
```

### Why Base Gains = 0?

```
    Base (floating):           Joints (actuated):
    ────────────────           ──────────────────
    We cannot directly         We CAN control these
    apply torques to base      with motor torques
    
    pGains[0:5] = 0           pGains[6:17] = 100
    dGains[0:5] = 0           dGains[6:17] = 5
```

The base is controlled **indirectly** through contact forces. Setting non-zero base gains would produce impossible base wrenches.

---

## 📊 Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           WBC DATA FLOW                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  MPC Output                    Current State                                │
│  ──────────                    ─────────────                                │
│                                                                             │
│  desiredState (24)             measuredRbdState (36)                        │
│  ├── h_norm [0:5]              ├── euler [0:2]                              │
│  ├── base_pose [6:11]          ├── position [3:5]                           │
│  └── joints [12:23]            ├── joints [6:17]                            │
│                                ├── ang_vel [18:20]                          │
│  desiredInput (24)             ├── lin_vel [21:23]                          │
│  ├── F_LF [0:2]                └── joint_vel [24:35]                        │
│  ├── F_RF [3:5]                                                             │
│  ├── F_LH [6:8]                                                             │
│  ├── F_RH [9:11]                                                            │
│  └── foot_vel [12:23]                                                       │
│                                                                             │
│           │                              │                                  │
│           └──────────────┬───────────────┘                                  │
│                          │                                                  │
│                          ▼                                                  │
│           ┌──────────────────────────────┐                                  │
│           │     WBC Processing           │                                  │
│           │                              │                                  │
│           │  1. Extract q_des, v_des     │                                  │
│           │  2. Transform forces to      │                                  │
│           │     joint frames             │                                  │
│           │  3. Compute PD feedback      │                                  │
│           │  4. Call RNEA                │                                  │
│           └──────────────┬───────────────┘                                  │
│                          │                                                  │
│                          ▼                                                  │
│           ┌──────────────────────────────┐                                  │
│           │  WBC Output (18-dim)         │                                  │
│           │                              │                                  │
│           │  ├── base_wrench [0:5]       │  (ignored - can't actuate)       │
│           │  └── joint_torques [6:17]    │  (used!)                         │
│           └──────────────┬───────────────┘                                  │
│                          │                                                  │
│                          ▼                                                  │
│           ┌──────────────────────────────┐                                  │
│           │  extractJointTorques()       │                                  │
│           │                              │                                  │
│           │  Reorder:                    │                                  │
│           │  OCS2 [LF,LH,RF,RH]          │                                  │
│           │     →                        │                                  │
│           │  MuJoCo [LF,RF,LH,RH]        │                                  │
│           └──────────────┬───────────────┘                                  │
│                          │                                                  │
│                          ▼                                                  │
│           ┌──────────────────────────────┐                                  │
│           │  sim->d_->ctrl[0:11]         │                                  │
│           │                              │                                  │
│           │  Final joint torques         │                                  │
│           │  applied to MuJoCo           │                                  │
│           └──────────────────────────────┘                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 🧪 Numerical Example

### Standing with 4-Point Contact

**MPC Output (typical values):**
```
desiredInput (contact forces):
  F_LF = [0.1, -0.2, 109.5] N
  F_RF = [0.1,  0.2, 109.5] N
  F_LH = [-0.1, -0.2, 109.5] N
  F_RH = [-0.1,  0.2, 109.5] N
  
Total Fz = 4 × 109.5 = 438 N ≈ 52.1 kg × 9.81 m/s² × (slight margin)
```

**WBC Output (typical values):**
```
joint_torques (Nm):
  LF: [0.3, 15.2, -9.8]    # HAA, HFE, KFE
  RF: [-0.3, 15.2, -9.8]
  LH: [0.3, -14.8, 9.5]
  RH: [-0.3, -14.8, 9.5]

Note: 
- HFE/KFE torques are significant (~15 Nm) to support weight
- HAA torques are small (~0.3 Nm) for lateral stability
- Signs differ between front/hind legs due to leg geometry
```

### Physical Interpretation

```
    Side View of One Leg:
    
        HAA (hip abduction)
         │
         ├──HFE─────┐
                    │
                    │ Thigh
                    │
                   KFE
                    │
                    │ Shank
                    │
                   ▼
                  Foot
                   ↑
                   F_z (ground reaction)
    
    To support weight:
    - HFE must produce torque to prevent thigh from falling
    - KFE must produce torque to prevent shank from collapsing
    - These are computed by RNEA from contact forces
```

---

## 🔑 Key Insights

### Why WBC Works Better Than Pure PD

| Aspect | Pure PD | WBC (RNEA) |
|--------|---------|------------|
| Gravity | Must be overcome by error | Pre-computed |
| Contact Forces | Ignored | Properly converted |
| Dynamics | Ignored | Included via RNEA |
| Stability | Requires high gains | Natural |
| Accuracy | Position error always | Can achieve zero error |

### Why We Keep PD in WBC

The PD terms in WBC provide:
1. **Robustness:** Correct model errors
2. **Tracking:** Follow desired trajectory
3. **Damping:** Prevent oscillations

Without PD, pure feedforward would drift due to:
- Model inaccuracies
- Sensor noise
- Unmodeled disturbances

---

## 🐛 Common WBC Bugs

### Bug 1: Wrong Joint Ordering

**Symptom:** Robot twists or falls sideways
**Cause:** OCS2 and MuJoCo joint orders differ
**Fix:** Use proper mapping functions

### Bug 2: Wrong Force Frame

**Symptom:** Forces seem to push wrong direction
**Cause:** Forces not transformed to joint frame
**Fix:** Apply rotation: `R_world_to_joint * F_world`

### Bug 3: Sign Errors in Torques

**Symptom:** Robot pushes itself into ground or jumps
**Cause:** Sign convention mismatch
**Fix:** Verify `τ = RNEA(..., F_ext)` not `τ = RNEA(..., -F_ext)`

### Bug 4: Base Gains Non-Zero

**Symptom:** Unstable behavior, NaN values
**Cause:** Trying to apply torques to floating base
**Fix:** Set `pGains[0:5] = 0`, `dGains[0:5] = 0`

---

**Next:** [06_COMPARISON_ETH.md](./06_COMPARISON_ETH.md) - Comparison with ETH RSL's Implementation

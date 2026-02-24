# 06. Comparison with ETH RSL's Implementation

## 📚 Reference Implementations

This document compares our implementation with:
1. **OCS2's CentroidalModelRbdConversions** (main OCS2 repo)
2. **ETH RSL's perceptive_anymal** (`ocs2_switched_model_interface`)

---

## 🏗️ Architecture Comparison

### Our Implementation (vm_anymal_standing_online)

```
┌─────────────────────────────────────────────────────────────────┐
│                    vm_anymal_standing_online                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐   │
│  │ MuJoCo Sim    │───>│ OCS2 MPC      │───>│ WBC (RNEA)    │   │
│  │               │<───│ (DDP)         │<───│               │   │
│  └───────────────┘    └───────────────┘    └───────────────┘   │
│                                                                 │
│  Single executable, 3 threads                                   │
│  - Physics @ variable Hz                                        │
│  - MPC @ 50 Hz                                                  │
│  - Control @ 1000 Hz                                            │
└─────────────────────────────────────────────────────────────────┘
```

### ETH RSL's Implementation (perceptive_anymal)

```
┌─────────────────────────────────────────────────────────────────┐
│                      perceptive_anymal                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐   │
│  │ ROS2 Node     │───>│ OCS2 MPC      │───>│ WBC + QP      │   │
│  │ (Gazebo/Real) │<───│ (SLQ/SQP/IPM) │<───│ Hierarchical  │   │
│  └───────────────┘    └───────────────┘    └───────────────┘   │
│                                                                 │
│  Multiple ROS2 nodes:                                           │
│  - MPC node (separate process)                                  │
│  - WBC node (separate process)                                  │
│  - State estimator node                                         │
│  - Terrain perception node                                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔄 WBC Implementation Comparison

### Our Approach: Direct RNEA

```cpp
// vm_anymal_standing_online/src/main_anymal_mujoco.cpp
vector_t computeWbcTorques(...) {
    return rbdConversions->computeRbdTorqueFromCentroidalModelPD(
        desiredState, desiredInput, jointAccelerations,
        measuredRbdState, pGains, dGains);
}
```

**Pros:**
- Simple, single function call
- Uses OCS2's built-in implementation
- Good for standing/slow locomotion

**Cons:**
- No explicit task hierarchy
- No QP for constraint handling
- Less robust for dynamic gaits

### ETH's Approach: TorqueApproximation + Hierarchical WBC

**File:** `ocs2_switched_model_interface/src/core/TorqueApproximation.cpp`

```cpp
// Simplified version of ETH's approach
vector_t TorqueApproximation::torqueApproximation(
    const CentroidalModelPinocchioMapping& mapping,
    const PinocchioInterface& pinocchio,
    const vector_t& rbdState,
    const vector_t& contactFlags,
    const vector_t& desiredBaseAcceleration,
    const vector_t& desiredJointAcceleration,
    const vector_t& desiredContactForce) 
{
    const auto& info = mapping.getCentroidalModelInfo();
    const auto& model = pinocchio.getModel();
    auto& data = pinocchio.getData();
    
    // Extract configuration
    vector_t q = extractConfiguration(rbdState, info);
    vector_t v = extractVelocity(rbdState, info);
    
    // Build desired acceleration
    vector_t a(info.generalizedCoordinatesNum);
    a.head<6>() = desiredBaseAcceleration;
    a.tail(info.actuatedDofNum) = desiredJointAcceleration;
    
    // Build external forces
    auto fext = buildExternalForces(model, data, info, desiredContactForce);
    
    // RNEA
    return pinocchio::rnea(model, data, q, v, a, fext);
}
```

**Additional ETH Components:**

1. **Hierarchical QP WBC:**
```cpp
// Priority 1: Floating base dynamics
// Priority 2: Contact constraints
// Priority 3: Swing foot tracking
// Priority 4: Joint torque limits
// Priority 5: Posture regularization
```

2. **Contact Wrench Optimization:**
```cpp
// Optimize contact forces to satisfy:
// - Friction cone constraints
// - Unilateral contact (Fz > 0)
// - CoP within support polygon
```

---

## 📊 Key Differences

### 1. State Estimation

| Aspect | Our Implementation | ETH RSL |
|--------|-------------------|---------|
| Source | MuJoCo ground truth | EKF/UKF state estimator |
| Base pose | Direct from sim | Fused from IMU + kinematics |
| Contact detection | Hardcoded (all in contact) | Force threshold + scheduling |

### 2. WBC Strategy

| Aspect | Our Implementation | ETH RSL |
|--------|-------------------|---------|
| Method | Direct RNEA + PD | Hierarchical QP |
| Tasks | Single (tracking) | Multiple (hierarchy) |
| Constraints | Via MPC penalty | Explicit QP constraints |
| Robustness | Model-dependent | More robust |

### 3. Contact Force Distribution

| Aspect | Our Implementation | ETH RSL |
|--------|-------------------|---------|
| Method | MPC direct output | MPC + WBC redistribution |
| CoP | Not explicitly controlled | QP ensures CoP validity |
| Friction | Soft penalty in MPC | Hard constraint in QP |

---

## 💻 Code Comparison: Force to Torque Conversion

### Our Code (using OCS2's CentroidalModelRbdConversions)

```cpp
// Inside CentroidalModelRbdConversions::computeRbdTorqueFromCentroidalModelPD

// Transform contact forces to joint frame
for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
    const auto jointIndex = model.frames[frameIndex].parentJoint;
    
    const Vector3 translationJointToContact = 
        model.frames[frameIndex].placement.translation();
    const Matrix3 rotWorldToJoint = 
        data.oMi[jointIndex].rotation().transpose();
    
    // Transform force from world to joint frame
    const Vector3 contactForce = rotWorldToJoint * 
        centroidal_model::getContactForces(desiredInput, i, info);
    
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = 
        translationJointToContact.cross(contactForce);
}

// Call RNEA with external forces
return pinocchio::rnea(model, data, qDesired, vDesired, aAugmented, fextDesired);
```

### ETH's TorqueApproximation (simplified)

```cpp
// From ocs2_switched_model_interface/src/core/TorqueApproximation.cpp

vector_t computeTorque(const vector_t& rbdState, 
                       const vector_t& contactForces,
                       const ContactSchedule& schedule) {
    // Get configuration
    auto q = getGeneralizedCoordinates(rbdState);
    auto v = getGeneralizedVelocities(rbdState);
    
    // Zero acceleration for quasi-static
    vector_t a = vector_t::Zero(q.size());
    
    // Build external forces for active contacts only
    std::vector<pinocchio::Force> fext(model.njoints, pinocchio::Force::Zero());
    
    for (size_t i = 0; i < numLegs; i++) {
        if (schedule.inContact(i)) {
            // Transform force to joint frame
            auto F_world = contactForces.segment<3>(3*i);
            auto F_joint = transformToJointFrame(i, F_world);
            fext[jointIndices[i]] = pinocchio::Force(F_joint, Vector3::Zero());
        }
    }
    
    // RNEA
    return pinocchio::rnea(model, data, q, v, a, fext);
}
```

**Key Difference:** ETH checks `inContact()` for each leg, we assume all legs in contact for standing.

---

## 🎯 ETH's Hierarchical WBC Structure

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ETH RSL Hierarchical WBC                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  PRIORITY 1 (Highest): Equations of Motion                                  │
│  ───────────────────────────────────────────                                │
│  M(q)a + h(q,v) = S^T τ + J_c^T F_c                                         │
│  (Cannot be violated)                                                       │
│                                                                             │
│  PRIORITY 2: Contact Constraints                                            │
│  ───────────────────────────────                                            │
│  - Zero velocity at contact: J_c * v + J̇_c * q = 0                          │
│  - Friction cone: ||F_t|| ≤ μ * F_n                                         │
│  - Unilateral: F_n ≥ 0                                                      │
│                                                                             │
│  PRIORITY 3: Motion Tracking                                                │
│  ─────────────────────────────                                              │
│  - Base pose tracking (from MPC)                                            │
│  - Swing foot tracking (from trajectory planner)                            │
│                                                                             │
│  PRIORITY 4: Torque/Force Limits                                            │
│  ─────────────────────────────────                                          │
│  - τ_min ≤ τ ≤ τ_max                                                        │
│  - F_min ≤ F ≤ F_max                                                        │
│                                                                             │
│  PRIORITY 5 (Lowest): Regularization                                        │
│  ─────────────────────────────────────                                      │
│  - Minimize ||τ||²                                                          │
│  - Minimize ||a||²                                                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Why Hierarchical?**
- Higher priority tasks are satisfied first
- Lower priority tasks only use remaining degrees of freedom
- Prevents conflicts between competing objectives

---

## 🔧 When to Use Each Approach

### Use Our Simple Approach When:
- ✅ Standing control only
- ✅ Slow, quasi-static locomotion
- ✅ Simulation/development
- ✅ Learning/understanding MPC+WBC
- ✅ Simple hardware without strict constraints

### Use ETH's Hierarchical Approach When:
- ✅ Dynamic locomotion (running, jumping)
- ✅ Real hardware deployment
- ✅ Strict torque/force limits
- ✅ Terrain adaptation
- ✅ Multiple concurrent tasks

---

## 📈 Performance Comparison

### Computational Cost

| Metric | Our Implementation | ETH Hierarchical |
|--------|-------------------|------------------|
| WBC time | ~0.1 ms (RNEA) | ~1-5 ms (QP solve) |
| Complexity | O(n) RNEA | O(n³) QP |
| Memory | Low | Moderate |

### Control Quality

| Metric | Our Implementation | ETH Hierarchical |
|--------|-------------------|------------------|
| Standing | ✅ Good | ✅ Excellent |
| Walking | ⚠️ Okay | ✅ Excellent |
| Running | ❌ Poor | ✅ Good |
| Disturbance rejection | ⚠️ Moderate | ✅ Excellent |

---

## 🚀 Upgrading Our Implementation

If you want to improve our implementation toward ETH's approach:

### Step 1: Add Contact Detection

```cpp
// Instead of hardcoded contact
bool isInContact(int legIndex, double* sensorForces) {
    return sensorForces[legIndex] > CONTACT_THRESHOLD;
}
```

### Step 2: Add Force Redistribution

```cpp
// Simple QP for force optimization
vector_t redistributeForces(const vector_t& mpcForces, 
                            const std::vector<bool>& contacts) {
    // Minimize: ||F - F_mpc||² 
    // Subject to: friction cone, Fz > 0
    // Use OSQP or qpOASES
}
```

### Step 3: Add Task Hierarchy

```cpp
class HierarchicalWBC {
public:
    void addTask(int priority, const Task& task);
    vector_t solve(const RobotState& state);
private:
    std::vector<std::pair<int, Task>> tasks_;
};
```

---

## 📝 Summary

| Feature | vm_anymal_standing | ETH perceptive_anymal |
|---------|-------------------|----------------------|
| WBC Method | Direct RNEA | Hierarchical QP |
| MPC Solver | DDP | DDP/SQP/IPM |
| Contact | Hardcoded | Scheduled + detected |
| Terrain | Flat only | Perceptive |
| Real Robot | No | Yes |
| Complexity | Simple | Complex |
| Learning Curve | Low | High |

**Conclusion:** Our implementation is a **simplified, educational version** that demonstrates the core concepts. ETH's implementation is **production-ready** for real robot deployment.

---

**Next:** [07_DEBUGGING_GUIDE.md](./07_DEBUGGING_GUIDE.md) - Common Issues and Debugging

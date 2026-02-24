# 07. Debugging Guide

## 🐛 Common Issues and Solutions

This document covers issues encountered during development and their solutions.

---

## Issue 1: Q/R Matrix Loading Warnings

### Symptoms

```
[LeggedRobotInterface] Loading Q matrix...
WARNING: Q(0,1) = 0 (off-diagonal element)
WARNING: Q(0,2) = 0 (off-diagonal element)
...
```

### Analysis

These warnings appear because `ocs2::loadData::loadEigenMatrix()` reports all elements it loads, including zeros in off-diagonal positions.

### Solution

**This is NOT a bug!** Q and R are diagonal matrices. The warnings are informational and can be safely ignored.

```ini
; Q is diagonal - only diagonal elements matter
Q
{
  (0,0)   15.0    ; Only diagonal
  (1,1)   15.0    ; elements are
  (2,2)   30.0    ; non-zero
  ...
}
```

---

## Issue 2: Robot Falls Despite MPC Running ⭐ CRITICAL

### Symptoms

- MPC shows valid output (forces ~128N per leg)
- Robot still collapses to ground
- Joint torques are small or zero

### Root Cause

**Missing Whole Body Control (WBC)!**

MPC outputs **contact forces**, not **joint torques**. Without WBC to convert forces to torques, the robot has no actuation.

### Original Broken Code

```cpp
// WRONG: Using MPC output directly as joint commands
for (int i = 0; i < 12; i++) {
    double pos = sim->d_->qpos[7 + i];
    double vel = sim->d_->qvel[6 + i];
    torques(i) = KP * (targetJoints[i] - pos) - KD * vel;
    // MPC forces are IGNORED!
}
```

### Fixed Code

```cpp
// CORRECT: Use WBC to convert forces to torques
if (USE_WBC_TORQUE && useMpc && rbdConversions) {
    vector_t wbcTorques = computeWbcTorques(desiredState, desiredInput,
                                             measuredRbdState);
    extractJointTorques(wbcTorques, torques);
}
```

### Verification

After fix, you should see:
```
[WBC] Contact forces Fz: [109.5, 109.5, 109.5, 109.5]
[WBC] Joint torques (MuJoCo): 0.3 15.2 -9.8 -0.3 15.2 -9.8 0.3 -14.8 9.5 -0.3 -14.8 9.5
[DEBUG] optimal_z=0.585
```

---

## Issue 3: Robot Oscillates or Explodes

### Symptoms

- Robot shakes violently
- Joint torques rapidly alternate between extremes
- Eventually NaN values appear

### Possible Causes

#### Cause A: Q scaling too high

```ini
; BAD: Q scaling makes weights too large
Q { scaling 1e+3 }

; GOOD: Balanced scaling
Q { scaling 1e+0 }
```

#### Cause B: PD gains in WBC too high

```cpp
// BAD: Gains too high
pGains(6 + i) = 1000.0;  // Causes oscillation
dGains(6 + i) = 50.0;

// GOOD: Moderate gains
pGains(6 + i) = 100.0;
dGains(6 + i) = 5.0;
```

#### Cause C: Control rate too slow

```cpp
// BAD: Control at 100 Hz
std::this_thread::sleep_for(std::chrono::milliseconds(10));

// GOOD: Control at 1000 Hz
std::this_thread::sleep_for(std::chrono::microseconds(1000));
```

---

## Issue 4: Joint Order Mismatch

### Symptoms

- Robot twists or moves unexpectedly
- Torques seem correct in magnitude but wrong in direction
- Left/right legs behave incorrectly

### Root Cause

MuJoCo and OCS2 use different joint orderings:

```
MuJoCo: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
OCS2:   LF(0-2), LH(3-5), RF(6-8), RH(9-11)
         ↑        ↑ swap  ↑        ↑
```

### Solution

Use proper mapping functions:

```cpp
// mujocoToRbdState: MuJoCo → OCS2
rbdState(6 + 0) = qpos[7 + 0];  // LF → LF
rbdState(6 + 3) = qpos[7 + 6];  // LH ← MuJoCo's LH (index 6-8)
rbdState(6 + 6) = qpos[7 + 3];  // RF ← MuJoCo's RF (index 3-5)
rbdState(6 + 9) = qpos[7 + 9];  // RH → RH

// extractJointTorques: OCS2 → MuJoCo
mujocoTorques(0) = wbcTorques(6 + 0);  // LF
mujocoTorques(3) = wbcTorques(6 + 6);  // RF ← OCS2's RF (index 6-8)
mujocoTorques(6) = wbcTorques(6 + 3);  // LH ← OCS2's LH (index 3-5)
mujocoTorques(9) = wbcTorques(6 + 9);  // RH
```

---

## Issue 5: MPC Not Converging

### Symptoms

```
[MPC] Warning: DDP did not converge
[MPC] Cost: inf
```

### Possible Causes

#### Cause A: Bad initial state

MPC was initialized from a state far from reference.

**Solution:** Wait for PD to stabilize before starting MPC.

```cpp
double mpcStartDelay = 3.0;  // Wait 3 seconds
if (sim->d_->time > mpcStartDelay) {
    mpcReady.store(true);
}
```

#### Cause B: Reference trajectory issue

Reference input doesn't include gravity compensation.

```cpp
// BAD: Zero input reference
target.inputTrajectory = {vector_t::Zero(24)};

// GOOD: Gravity-compensating reference
vector_t gravComp = weightCompensatingInput(info, contactFlags);
target.inputTrajectory = {gravComp, gravComp};
```

#### Cause C: Infeasible constraints

Friction coefficient too low or force limits too tight.

```ini
; Increase friction coefficient
frictionConeSoftConstraint
{
  frictionCoefficient    0.7  ; Was 0.5
}
```

---

## Issue 6: Quaternion/Euler Conversion Errors

### Symptoms

- Orientation seems wrong
- Robot rotates unexpectedly
- Gimbal lock behavior near ±90° pitch

### Solution

Use robust conversion:

```cpp
void quatToZYXEuler(double qw, double qx, double qy, double qz,
                   double& yaw, double& pitch, double& roll) {
    // Handle gimbal lock
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);  // ±90°
    else
        pitch = asin(sinp);
    
    // Standard conversion for roll and yaw
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(siny_cosp, cosy_cosp);
}
```

---

## Issue 7: NaN in Torques

### Symptoms

```
[WBC] Joint torques: nan nan nan nan ...
```

### Debugging Steps

1. **Check state validity:**
```cpp
if (!measuredRbdState.allFinite()) {
    std::cerr << "Invalid state!" << std::endl;
}
```

2. **Check MPC output:**
```cpp
if (!g_optimalInput.allFinite()) {
    std::cerr << "MPC produced NaN!" << std::endl;
}
```

3. **Check RNEA inputs:**
```cpp
// Inside computeWbcTorques
std::cout << "q: " << qDesired.transpose() << std::endl;
std::cout << "v: " << vDesired.transpose() << std::endl;
```

### Common Causes

- Division by zero in orientation conversion
- Invalid contact forces (Fz < 0)
- Numerical overflow in RNEA

---

## 🔍 Debugging Tools

### Debug Print Macro

```cpp
#define DEBUG_PRINT(name, vec) \
    std::cout << "[DEBUG] " << name << ": " << vec.transpose() << std::endl

// Usage
DEBUG_PRINT("MPC state", g_optimalState);
DEBUG_PRINT("WBC torques", wbcTorques);
```

### State Validity Check

```cpp
bool isStateValid(const vector_t& state) {
    // Check for NaN/Inf
    if (!state.allFinite()) return false;
    
    // Check reasonable bounds
    if (state(8) < 0 || state(8) > 2.0) return false;  // z in [0, 2m]
    
    // Check joint limits
    for (int i = 12; i < 24; i++) {
        if (std::abs(state(i)) > 3.14) return false;  // ±π
    }
    
    return true;
}
```

### Torque Logging

```cpp
void logTorques(const std::string& filename) {
    static std::ofstream file(filename);
    static double lastTime = 0;
    
    double currentTime = sim->d_->time;
    file << currentTime << " ";
    for (int i = 0; i < 12; i++) {
        file << g_torques(i) << " ";
    }
    file << std::endl;
}
```

---

## 📊 Diagnostic Checklist

### Before Running

- [ ] Config files exist and are readable
- [ ] URDF model loads correctly
- [ ] MuJoCo model loads correctly
- [ ] CppAD libraries compile

### During Initialization

- [ ] State dimension = 24
- [ ] Input dimension = 24
- [ ] Robot mass ≈ 52.13 kg
- [ ] 4 contact points detected

### At Runtime

- [ ] MPC runs at ~50 Hz
- [ ] Control runs at ~1000 Hz
- [ ] MPC converges (cost decreasing)
- [ ] Contact forces ≈ 110-130 N per leg
- [ ] Joint torques in reasonable range (±40 Nm)
- [ ] Robot height stable at ~0.575-0.585 m

### If Problems Persist

1. **Isolate the issue:**
   - Test MPC alone (print outputs)
   - Test WBC alone (with hardcoded forces)
   - Test physics alone (gravity only)

2. **Compare with known-good:**
   - Use `quadruped_go2` as reference
   - Compare state/input dimensions
   - Compare joint ordering

3. **Simplify:**
   - Reduce to single leg
   - Use shorter horizon
   - Disable constraints temporarily

---

## 🎯 Quick Reference Table

| Symptom | Likely Cause | Quick Fix |
|---------|--------------|-----------|
| Robot falls | No WBC | Enable USE_WBC_TORQUE |
| Robot oscillates | High gains | Reduce Q.scaling to 1e+0 |
| Robot twists | Joint order | Check mapping functions |
| MPC doesn't converge | Bad init | Increase mpcStartDelay |
| NaN torques | Invalid state | Add validity checks |
| Torques too high | Wrong forces | Check gravity compensation |
| Robot jumps | Force sign | Check RNEA sign convention |

---

**End of Documentation**

Return to [00_INDEX.md](./00_INDEX.md) for the full table of contents.

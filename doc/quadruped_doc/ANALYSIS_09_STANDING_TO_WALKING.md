# Analysis 09: From Standing to Walking - Development Guide

## 1. Overview

This document outlines the development path to extend `vm_anymal_standing_online` from a **standing controller** to a full **walking/locomotion controller**.

### Current State (Standing)
```
Mode: STANCE (15) - All 4 feet always in contact
Control: Centroidal MPC + WBC
Capability: Height control, pitch/yaw adjustment
```

### Target State (Walking)
```
Modes: TROT, PACE, WALK, etc. - Dynamic contact switching
Control: Centroidal MPC + WBC + Swing Trajectory Planning
Capability: Velocity commands, gait transitions, terrain adaptation
```

---

## 2. Key Differences: Standing vs Walking

| Aspect | Standing | Walking |
|--------|----------|---------|
| Contact Mode | STANCE (15) always | Dynamic mode switching |
| Gait Schedule | Static | Time-varying pattern |
| Foot Trajectory | N/A | Swing trajectory planning |
| Reference | Position target | Velocity command |
| Constraints | 4 feet always contact | Mode-dependent contact |
| Force Distribution | Static | Dynamic (swing=0, stance>0) |

---

## 3. Components to Modify/Add

### 3.1 Gait System Enhancement

#### Current: Single STANCE mode
```cpp
// Current gait (reference.info)
defaultModeSequenceTemplate
{
  modeSequence { [0] STANCE }
  switchingTimes { [0] 0.0  [1] 1.0 }
}
```

#### Target: Multiple gait patterns
```cpp
// New gait definitions needed
trot
{
  modeSequence
  {
    [0] LF_RH      // Diagonal pair 1 (mode 9)
    [1] RF_LH      // Diagonal pair 2 (mode 6)
  }
  switchingTimes
  {
    [0] 0.0
    [1] 0.35      // 35% duty cycle
    [2] 0.70      // Full gait cycle
  }
}

standing_trot    // With stance phases
{
  modeSequence
  {
    [0] LF_RH
    [1] STANCE
    [2] RF_LH
    [3] STANCE
  }
  switchingTimes { [0] 0.0 [1] 0.30 [2] 0.35 [3] 0.65 [4] 0.70 }
}

pace             // Same-side pairs
{
  modeSequence
  {
    [0] LF_LH     // Left side (mode 10)
    [1] RF_RH     // Right side (mode 5)
  }
  switchingTimes { [0] 0.0 [1] 0.35 [2] 0.70 }
}

static_walk      // 3-leg support always
{
  modeSequence
  {
    [0] LF_RF_RH  // Swing LH (mode 13)
    [1] RF_LH_RH  // Swing LF (mode 7)
    [2] LF_RF_LH  // Swing RH (mode 14)
    [3] LF_LH_RH  // Swing RF (mode 11)
  }
  switchingTimes { [0] 0.0 [1] 0.3 [2] 0.6 [3] 0.9 [4] 1.2 }
}
```

### 3.2 Reference Manager Enhancement

#### Current Implementation
```cpp
void SwitchedModelReferenceManager::modifyReferences(...) {
    // Gets mode schedule from gait scheduler
    modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime, finalTime);
    
    // Updates swing trajectory (currently just height=0 for all)
    swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}
```

#### Required Enhancement
```cpp
void SwitchedModelReferenceManager::modifyReferences(...) {
    // 1. Get mode schedule from gait
    modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime, finalTime);
    
    // 2. Update swing trajectories with proper lift-off/touch-down positions
    feet_array_t<scalar_array_t> liftOffHeights, touchDownHeights;
    for (size_t leg = 0; leg < 4; ++leg) {
        liftOffHeights[leg] = computeLiftOffHeights(leg, modeSchedule);
        touchDownHeights[leg] = computeTouchDownHeights(leg, modeSchedule);
    }
    swingTrajectoryPtr_->update(modeSchedule, liftOffHeights, touchDownHeights);
    
    // 3. Update base reference from velocity command
    targetTrajectories = generateExtrapolatedBaseReference(
        velocityCommand, currentState, horizon);
}
```

### 3.3 Velocity Command Interface

#### New Component: VelocityCommandInterface
```cpp
struct VelocityCommand {
    double vx;           // Forward velocity [m/s]
    double vy;           // Lateral velocity [m/s]
    double yawRate;      // Yaw rate [rad/s]
    double baseHeight;   // Target height [m]
};

class VelocityCommandInterface {
public:
    // From Xbox controller
    void updateFromXbox(float lx, float ly, float rx, float ry) {
        command_.vx = ly * maxVelocity_;      // Left Y -> forward
        command_.vy = lx * maxVelocity_;      // Left X -> lateral
        command_.yawRate = rx * maxYawRate_;  // Right X -> yaw
        // Right Y -> height (keep from standing)
    }
    
    // Generate target trajectory from velocity command
    TargetTrajectories generateTargetTrajectory(
        double initTime, double finalTime,
        const vector_t& initState);
        
private:
    VelocityCommand command_;
    double maxVelocity_ = 0.5;    // m/s
    double maxYawRate_ = 0.5;     // rad/s
};
```

### 3.4 FSM Enhancement

#### Current States
```cpp
enum class FSMState {
    PASSIVE,
    PD_CONTROL,
    STANDING,
    RECOVERY,
    EMERGENCY
};
```

#### Enhanced States for Walking
```cpp
enum class FSMState {
    PASSIVE,       // No torques
    PD_CONTROL,    // PD to default stance
    STANDING,      // MPC standing (current)
    
    // New walking states
    WALK_READY,    // Preparing to walk
    WALKING,       // Active locomotion
    STOPPING,      // Transitioning to stand
    
    RECOVERY,
    EMERGENCY
};

enum class GaitType {
    STANCE,
    TROT,
    STANDING_TROT,
    PACE,
    STATIC_WALK,
    DYNAMIC_WALK
};
```

### 3.5 Swing Trajectory Planning

#### Current: Simple height constraint
```cpp
// SwingTrajectoryPlanner - already implemented!
scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;
scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;
```

#### Enhancement: XY foot placement
```cpp
class EnhancedSwingTrajectoryPlanner : public SwingTrajectoryPlanner {
public:
    // Add XY trajectory planning
    vector3_t getFootPositionConstraint(size_t leg, scalar_t time) const;
    vector3_t getFootVelocityConstraint(size_t leg, scalar_t time) const;
    
    // Foot placement heuristics
    vector3_t computeFootPlacement(size_t leg, const VelocityCommand& cmd,
                                    const vector_t& currentState);
                                    
private:
    // Raibert heuristic: p_foot = p_hip + v * T_stance/2 + k * (v - v_des)
    vector3_t raibertHeuristic(size_t leg, const VelocityCommand& cmd,
                                const vector_t& state);
};
```

---

## 4. Implementation Roadmap

### Phase 1: Gait Infrastructure (Week 1-2)

**Goal**: Enable gait switching without breaking standing

**Tasks**:
1. Create `gait.info` with multiple gait definitions
2. Add `GaitCommandInterface` class
3. Modify `GaitSchedule` to support gait switching
4. Add gait selection to Xbox controller (D-pad)

**Files to modify**:
- `config/command/gait.info` (new)
- `src/gait/GaitSchedule.cpp`
- `include/gait/GaitSchedule.h`
- `src/XboxController.cpp`
- `src/FSMController.cpp`

### Phase 2: Velocity Commands (Week 2-3)

**Goal**: Replace position target with velocity command

**Tasks**:
1. Create `VelocityCommandInterface` class
2. Implement base trajectory extrapolation
3. Modify `SwitchedModelReferenceManager` to use velocity
4. Map Xbox left stick to velocity command

**Files to add**:
- `include/command/VelocityCommandInterface.h`
- `src/command/VelocityCommandInterface.cpp`

**Files to modify**:
- `src/reference_manager/SwitchedModelReferenceManager.cpp`
- `src/main_anymal_mujoco.cpp`

### Phase 3: Swing Trajectory (Week 3-4)

**Goal**: Proper swing foot trajectories

**Tasks**:
1. Enhance `SwingTrajectoryPlanner` with XY planning
2. Implement Raibert heuristic for foot placement
3. Add foot position constraints
4. Test with standing_trot gait

**Files to modify**:
- `src/foot_planner/SwingTrajectoryPlanner.cpp`
- `include/foot_planner/SwingTrajectoryPlanner.h`
- `src/LeggedRobotPreComputation.cpp`

### Phase 4: Integration & Testing (Week 4-5)

**Goal**: Full walking capability

**Tasks**:
1. Integrate all components
2. Tune gait parameters
3. Add gait transition logic
4. Test different gaits
5. Tune MPC cost weights for walking

---

## 5. Code Examples

### 5.1 Gait Selection via Xbox

```cpp
// In XboxController
int getGaitChangeRequest() {
    // D-pad for gait selection
    if (dpad_up_pressed) return 0;    // STANCE
    if (dpad_right_pressed) return 1; // TROT
    if (dpad_down_pressed) return 2;  // PACE
    if (dpad_left_pressed) return 3;  // WALK
    return -1;  // No change
}

// In FSMController
void updateGait(int gaitRequest) {
    if (currentState_ != FSMState::WALKING && 
        currentState_ != FSMState::STANDING) return;
        
    switch (gaitRequest) {
        case 0: setGait(GaitType::STANCE); break;
        case 1: setGait(GaitType::TROT); break;
        case 2: setGait(GaitType::PACE); break;
        case 3: setGait(GaitType::STATIC_WALK); break;
    }
}
```

### 5.2 Velocity to Target Trajectory

```cpp
TargetTrajectories VelocityCommandInterface::generateTargetTrajectory(
    double t0, double tf, const vector_t& x0) {
    
    TargetTrajectories target;
    const double dt = 0.1;  // 100ms steps
    
    // Current base state
    double x = x0(6);
    double y = x0(7);
    double yaw = x0(11);
    
    for (double t = t0; t <= tf; t += dt) {
        // Extrapolate position
        x += command_.vx * cos(yaw) * dt - command_.vy * sin(yaw) * dt;
        y += command_.vx * sin(yaw) * dt + command_.vy * cos(yaw) * dt;
        yaw += command_.yawRate * dt;
        
        vector_t state = x0;  // Start from initial
        state(6) = x;
        state(7) = y;
        state(8) = command_.baseHeight;
        state(11) = yaw;
        
        // Velocities
        state(3) = command_.vx * cos(yaw) - command_.vy * sin(yaw);
        state(4) = command_.vx * sin(yaw) + command_.vy * cos(yaw);
        state(5) = command_.yawRate;
        
        target.timeTrajectory.push_back(t);
        target.stateTrajectory.push_back(state);
        target.inputTrajectory.push_back(computeGravCompInput());
    }
    
    return target;
}
```

### 5.3 Enhanced Swing Planner

```cpp
vector3_t EnhancedSwingTrajectoryPlanner::computeFootPlacement(
    size_t leg, const VelocityCommand& cmd, const vector_t& state) {
    
    // Get hip position in world frame
    vector3_t hipPos = getHipPositionWorld(leg, state);
    
    // Raibert heuristic
    double T_stance = 0.35;  // Stance duration
    double kp = 0.03;        // Feedback gain
    
    vector3_t currentVel(state(3), state(4), 0.0);
    vector3_t desiredVel(cmd.vx, cmd.vy, 0.0);
    
    vector3_t footPlacement = hipPos 
        + currentVel * T_stance / 2.0 
        + kp * (currentVel - desiredVel);
    
    // Keep Z at terrain height
    footPlacement.z() = getTerrainHeight(footPlacement.x(), footPlacement.y());
    
    return footPlacement;
}
```

---

## 6. Configuration Changes

### 6.1 task.info Modifications

```ini
; Walking-specific settings
swing_trajectory_config
{
  liftOffVelocity               0.2    ; m/s vertical lift
  touchDownVelocity            -0.4    ; m/s vertical touch
  swingHeight                   0.10   ; m swing height
  touchdownAfterHorizon         0.2    ; s look-ahead
  swingTimeScale                0.15   ; scaling factor
}

; MPC settings for walking
mpc
{
  timeHorizon                   1.0    ; Longer for walking
  solutionTimeWindow            0.02
  mpcDesiredFrequency           100    ; Hz
}

; Cost weights for walking
Q
{
  ; Increase velocity tracking weight
  (3,3) 50.0   ; vx
  (4,4) 50.0   ; vy
  (5,5) 30.0   ; yaw_rate
}
```

### 6.2 New gait.info

```ini
list
{
  [0] stance
  [1] trot
  [2] standing_trot
  [3] pace
  [4] static_walk
}

; Default gait on startup
defaultGait stance

; Gait transition settings
transition
{
  phaseTransitionStanceTime 0.3  ; Stand briefly between gaits
}
```

---

## 7. Testing Plan

### 7.1 Unit Tests
1. Gait schedule generation
2. Swing trajectory interpolation
3. Velocity command extrapolation
4. Mode number to contact flags

### 7.2 Integration Tests
1. Standing → Trot transition
2. Trot → Standing transition
3. Velocity command tracking
4. Gait switching mid-walk

### 7.3 Simulation Tests
1. Forward walking at 0.3 m/s
2. Lateral walking
3. Turning in place
4. Combined forward + turn
5. Stop from walking

---

## 8. Reference: OCS2 Legged Robot Example

The `ocs2_legged_robot` package provides a complete walking implementation:

```
ocs2_ros2/basic examples/ocs2_legged_robot/
├── config/
│   ├── mpc/task.info         # MPC configuration
│   └── command/
│       ├── reference.info    # Initial state
│       └── gait.info         # All gait definitions
├── src/
│   ├── LeggedRobotInterface.cpp
│   ├── foot_planner/
│   └── gait/
└── launch/
    └── legged_robot.launch.py
```

### Key differences from vm_anymal_standing_online:
1. Multiple gait definitions in `gait.info`
2. Velocity command node (`LeggedRobotPoseCommandNode`)
3. Gait command node (`LeggedRobotGaitCommandNode`)
4. ROS2 integration for real robot

---

## 9. Summary

### What we have (Standing):
- ✅ Centroidal MPC
- ✅ WBC torque computation
- ✅ Gait infrastructure (but only STANCE)
- ✅ Swing trajectory planner (but unused)
- ✅ Xbox controller integration
- ✅ FSM mode management

### What we need (Walking):
- 📝 Multiple gait definitions
- 📝 Gait switching interface
- 📝 Velocity command interface
- 📝 Base trajectory extrapolation
- 📝 Enhanced swing foot placement
- 📝 Walking FSM states
- 📝 Parameter tuning

### Estimated effort: 4-5 weeks

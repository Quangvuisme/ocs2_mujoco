# 04. Configuration Files Explanation

## 📄 File Overview

| File | Path | Purpose |
|------|------|---------|
| `task.info` | `config/mpc/task.info` | MPC parameters, weights, solver settings |
| `reference.info` | `config/command/reference.info` | Reference trajectory, gait schedule |

---

## 📋 task.info - Complete Breakdown

### Section 1: Model Type (Line 1)

```ini
centroidalModelType             1      // 0: FullCentroidalDynamics, 1: Single Rigid Body Dynamics
```

| Value | Model | Description |
|-------|-------|-------------|
| 0 | Full Centroidal | Includes leg inertia effects in CoM dynamics |
| 1 | SRBD | Treats robot as single rigid body (simpler, faster) |

**Why SRBD?**
- Sufficient for standing/slow walking
- Faster computation (simpler equations)
- Less prone to numerical issues

---

### Section 2: Interface Settings (Lines 3-8)

```ini
legged_robot_interface
{
  verbose                               false
  useAnalyticalGradientsDynamics        false
  useAnalyticalGradientsConstraints     false
}
```

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `verbose` | false | Don't print loaded parameters |
| `useAnalyticalGradientsDynamics` | false | Use auto-diff (CppAD) for dynamics Jacobians |
| `useAnalyticalGradientsConstraints` | false | Use auto-diff for constraint Jacobians |

**Auto-diff vs Analytical:**
- Auto-diff: Automatic, always correct, slightly slower
- Analytical: Hand-coded, must maintain, faster

---

### Section 3: Model Settings (Lines 10-18)

```ini
model_settings
{
  positionErrorGain             0.0 ; 20.0
  phaseTransitionStanceTime     0.4

  verboseCppAd                  true
  recompileLibrariesCppAd       true
  modelFolderCppAd              /tmp/ocs2
}
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `positionErrorGain` | 0.0 | Error gain for position constraints (0=off) |
| `phaseTransitionStanceTime` | 0.4 | Time to consider as stance during transitions |
| `verboseCppAd` | true | Print CppAD compilation info |
| `recompileLibrariesCppAd` | true | Recompile auto-diff code on startup |
| `modelFolderCppAd` | /tmp/ocs2 | Where to store compiled libraries |

---

### Section 4: Swing Trajectory (Lines 20-27)

```ini
swing_trajectory_config
{
  liftOffVelocity               0.2
  touchDownVelocity            -0.4
  swingHeight                   0.1
  touchdownAfterHorizon         0.2
  swingTimeScale                0.15
}
```

**For standing control, these are mostly unused**, but they define:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `liftOffVelocity` | 0.2 m/s | Vertical velocity when lifting foot |
| `touchDownVelocity` | -0.4 m/s | Vertical velocity when landing |
| `swingHeight` | 0.1 m | Maximum foot height during swing |
| `touchdownAfterHorizon` | 0.2 s | When to expect touchdown after horizon |
| `swingTimeScale` | 0.15 | Timing scaling for swing trajectory |

---

### Section 5: SQP Settings (Lines 29-48)

```ini
sqp
{
  nThreads                              3
  dt                                    0.015
  sqpIteration                          1
  deltaTol                              1e-4
  g_max                                 1e-2
  g_min                                 1e-6
  inequalityConstraintMu                0.1
  inequalityConstraintDelta             5.0
  projectStateInputEqualityConstraints  true
  printSolverStatistics                 true
  printSolverStatus                     false
  printLinesearch                       false
  useFeedbackPolicy                     true
  integratorType                        RK2
  threadPriority                        50
}
```

**SQP = Sequential Quadratic Programming** (alternative solver, not used with DDP)

---

### Section 6: IPM Settings (Lines 50-73)

```ini
ipm
{
  nThreads                              3
  dt                                    0.015
  ipmIteration                          1
  ...
  initialBarrierParameter               1e-4
  targetBarrierParameter                1e-4
  barrierLinearDecreaseFactor           0.2
  barrierSuperlinearDecreasePower       1.5
  ...
}
```

**IPM = Interior Point Method** (another solver option)

---

### Section 7: DDP Settings (Lines 75-112) ⭐ MOST IMPORTANT

```ini
ddp
{
  algorithm                       SLQ        // Sequential Linear Quadratic
  nThreads                        3          // Parallel threads
  threadPriority                  50

  maxNumIterations                1          // Max iterations per MPC call
  minRelCost                      1e-1       // Convergence tolerance
  constraintTolerance             5e-3       // Constraint satisfaction

  displayInfo                     false
  displayShortSummary             false
  checkNumericalStability         false
  debugPrintRollout               false

  AbsTolODE                       1e-5       // ODE absolute tolerance
  RelTolODE                       1e-3       // ODE relative tolerance
  maxNumStepsPerSecond            10000
  timeStep                        0.015      // 15ms discretization
  backwardPassIntegratorType      ODE45

  constraintPenaltyInitialValue   20.0       // Augmented Lagrangian penalty
  constraintPenaltyIncreaseRate   2.0

  preComputeRiccatiTerms          true
  useFeedbackPolicy               false      // Open-loop (feedforward only)

  strategy                        LINE_SEARCH
  lineSearch
  {
    minStepLength                 1e-2
    maxStepLength                 1.0
    hessianCorrectionStrategy     DIAGONAL_SHIFT
    hessianCorrectionMultiple     1e-5
  }
}
```

**Key Parameters:**

| Parameter | Value | Why This Value |
|-----------|-------|----------------|
| `algorithm` | SLQ | Efficient for nonlinear systems |
| `maxNumIterations` | 1 | Fast real-time (1 iter per MPC call) |
| `timeStep` | 0.015 | 66 Hz discretization (fine enough) |
| `useFeedbackPolicy` | false | We use WBC for feedback instead |
| `constraintPenaltyInitialValue` | 20.0 | Strong initial constraint enforcement |

---

### Section 8: Rollout Settings (Lines 114-122)

```ini
rollout
{
  AbsTolODE                       1e-5
  RelTolODE                       1e-3
  timeStep                        0.015
  integratorType                  ODE45
  maxNumStepsPerSecond            10000
  checkNumericalStability         false
}
```

**Rollout** = Forward simulation of dynamics for MPC prediction.

---

### Section 9: MPC Settings (Lines 124-134) ⭐ CRITICAL

```ini
mpc
{
  timeHorizon                     1.0        // 1 second prediction horizon
  solutionTimeWindow              -1         // Use full horizon
  coldStart                       false      // Warm start from previous

  debugPrint                      false

  mpcDesiredFrequency             50         // 50 Hz MPC rate
  mrtDesiredFrequency             400        // 400 Hz state update
}
```

**Key Parameters:**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `timeHorizon` | 1.0 s | How far ahead to predict |
| `coldStart` | false | Use previous solution as initial guess |
| `mpcDesiredFrequency` | 50 Hz | MPC optimization runs at 50 Hz |
| `mrtDesiredFrequency` | 400 Hz | Observation update rate |

**Why 1 second horizon?**
- Long enough to plan contact sequences
- Short enough for real-time computation
- For standing, shorter horizon (0.5s) would also work

---

### Section 10: Initial State (Lines 136-162)

```ini
initialState
{
   ;; Normalized Centroidal Momentum: [linear, angular] ;;
   (0,0)  0.0     ; vcom_x
   (1,0)  0.0     ; vcom_y
   (2,0)  0.0     ; vcom_z
   (3,0)  0.0     ; L_x / robotMass
   (4,0)  0.0     ; L_y / robotMass
   (5,0)  0.0     ; L_z / robotMass

   ;; Base Pose: [position, orientation] ;;
   (6,0)  0.0     ; p_base_x
   (7,0)  0.0     ; p_base_y
   (8,0)  0.575   ; p_base_z   ← TARGET STANDING HEIGHT
   (9,0)  0.0     ; theta_base_z (yaw)
   (10,0) 0.0     ; theta_base_y (pitch)
   (11,0) 0.0     ; theta_base_x (roll)

   ;; Leg Joint Positions: [LF, LH, RF, RH] ;;
   (12,0) -0.25   ; LF_HAA     ← Note: OCS2 order!
   (13,0)  0.60   ; LF_HFE
   (14,0) -0.85   ; LF_KFE
   (15,0) -0.25   ; LH_HAA
   (16,0) -0.60   ; LH_HFE
   (17,0)  0.85   ; LH_KFE
   (18,0)  0.25   ; RF_HAA
   (19,0)  0.60   ; RF_HFE
   (20,0) -0.85   ; RF_KFE
   (21,0)  0.25   ; RH_HAA
   (22,0) -0.60   ; RH_HFE
   (23,0)  0.85   ; RH_KFE
}
```

**This defines the REFERENCE (target) state that MPC tries to track.**

```
Standing Pose Visualization:

          Front View                    Side View
        ┌───────────┐                ┌───────────┐
        │   Robot   │                │   Robot   │
        │   Body    │                │   Body    │
        └─────┬─────┘                └─────┬─────┘
         /   │   \                        │
       /     │     \                      │
     ┌─┐    ─│─    ┌─┐                   ─│─
     │ │     │     │ │                    │
     └─┘     │     └─┘                    │
    LF       │      RF                    │
             │                            │
        z = 0.575m                   z = 0.575m
```

---

### Section 11: Q Matrix (State Weights) (Lines 164-212) ⭐⭐ CRITICAL

```ini
Q
{
  scaling 1e+0   ← IMPORTANT: Was 1e+3, changed to 1e+0

  ;; Normalized Centroidal Momentum: [linear, angular] ;;
  (0,0)   15.0     ; vcom_x      ← Moderate: allow some velocity
  (1,1)   15.0     ; vcom_y
  (2,2)   30.0     ; vcom_z      ← Higher: penalize vertical velocity
  (3,3)   5.0      ; L_x / mass
  (4,4)   10.0     ; L_y / mass
  (5,5)   10.0     ; L_z / mass

  ;; Base Pose: [position, orientation] ;;
  (6,6)   500.0    ; p_base_x    ← HIGH: strong position tracking
  (7,7)   500.0    ; p_base_y
  (8,8)   500.0    ; p_base_z    ← HEIGHT IS IMPORTANT
  (9,9)   100.0    ; theta_base_z (yaw)
  (10,10) 200.0    ; theta_base_y (pitch)
  (11,11) 200.0    ; theta_base_x (roll)

  ;; Leg Joint Positions: [LF, LH, RF, RH] ;;
  (12,12) 20.0     ; LF_HAA      ← Lower: flexible joint tracking
  (13,13) 20.0     ; LF_HFE
  (14,14) 20.0     ; LF_KFE
  ...
}
```

**Why `scaling 1e+0` instead of `1e+3`?**
- Original `1e+3` made Q weights too high
- Caused MPC to prioritize state tracking over physical feasibility
- Result: Excessive contact forces → robot collapse
- Fix: `1e+0` balances state tracking with force limits

**Weight Design Philosophy:**

| Weight Level | Value | Meaning |
|--------------|-------|---------|
| Very High | 500 | Must track precisely (position) |
| High | 100-200 | Important (orientation) |
| Medium | 15-30 | Track but allow deviation (momentum) |
| Low | 20 | Soft tracking (joints) |

---

### Section 12: R Matrix (Input Weights) (Lines 214-250)

```ini
R
{
  scaling 1e-3   ← Input weights are scaled down

  ;; Feet Contact Forces: [LF, RF, LH, RH] ;;
  (0,0)   1.0       ; LF_Fx
  (1,1)   1.0       ; LF_Fy
  (2,2)   1.0       ; LF_Fz
  ...

  ;; Foot velocity relative to base ;;
  (12,12) 5000.0    ; LF_vx   ← VERY HIGH: feet should not move!
  (13,13) 5000.0    ; LF_vy
  (14,14) 5000.0    ; LF_vz
  ...
}
```

**Interpretation:**

| Input | Raw Weight | Scaled (×1e-3) | Meaning |
|-------|------------|----------------|---------|
| Forces | 1.0 | 0.001 | Forces are "cheap" |
| Foot velocity | 5000.0 | 5.0 | Penalize foot movement |

**Why high foot velocity weight?**
- Standing = feet should stay still
- High weight prevents foot slipping in solution
- For walking, this would be lower during swing phases

---

### Section 13: Friction Cone (Lines 252-259)

```ini
frictionConeSoftConstraint
{
  frictionCoefficient    0.5      // μ = 0.5 (rubber on concrete)
  
  ; relaxed log barrier parameters
  mu                     0.1      // Barrier weight
  delta                  5.0      // Barrier smoothness
}
```

**Friction Cone Constraint:**

$$\sqrt{F_x^2 + F_y^2} \leq \mu F_z$$

With $\mu = 0.5$:
- Max tangential force = 50% of normal force
- Prevents planned slip in MPC solution

---

## 📋 reference.info - Breakdown

### Basic Parameters (Lines 1-4)

```ini
targetDisplacementVelocity          0.5;   // m/s for locomotion commands
targetRotationVelocity              0.3;   // rad/s for turn commands

comHeight                           0.575  // Target CoM height [m]
```

### Default Joint State (Lines 6-20)

```ini
defaultJointState
{
   (0,0)  -0.25   ; LF_HAA
   (1,0)   0.60   ; LF_HFE
   (2,0)  -0.85   ; LF_KFE
   (3,0)  -0.25   ; LH_HAA
   (4,0)  -0.60   ; LH_HFE   ← Note sign flip vs LF
   (5,0)   0.85   ; LH_KFE
   (6,0)   0.25   ; RF_HAA
   (7,0)   0.60   ; RF_HFE
   (8,0)  -0.85   ; RF_KFE
   (9,0)   0.25   ; RH_HAA
   (10,0) -0.60   ; RH_HFE
   (11,0)  0.85   ; RH_KFE
}
```

**Note:** This uses a DIFFERENT joint order than OCS2 state:
- Here: LF, LH, RF, RH (same as task.info initialState)
- OCS2 input foot forces: LF, RF, LH, RH
- Be careful with indexing!

### Mode Schedule (Lines 22-45)

```ini
initialModeSchedule
{
  modeSequence
  {
    [0]  STANCE   // All feet on ground
    [1]  STANCE
  }
  eventTimes
  {
    [0]  0.5      // Switch at t=0.5s (but same mode)
  }
}

defaultModeSequenceTemplate
{
  modeSequence
  {
    [0]  STANCE
  }
  switchingTimes
  {
    [0]  0.0
    [1]  1.0      // Full horizon in stance
  }
}
```

**Mode Definitions:**
- `STANCE`: All 4 feet in contact
- `SWING_*`: Various swing phases (not used for standing)

For standing control, we always stay in `STANCE` mode with all feet grounded.

---

## 🔧 Tuning Guidelines

### If Robot is Unstable:

1. **Reduce Q scaling:**
   ```ini
   Q { scaling 1e-1 }  // More conservative
   ```

2. **Increase friction coefficient:**
   ```ini
   frictionCoefficient    0.7  // More grip
   ```

3. **Reduce MPC frequency:**
   ```ini
   mpcDesiredFrequency    30   // Give more time per solve
   ```

### If Robot Doesn't Track Position:

1. **Increase position weights:**
   ```ini
   (6,6)   1000.0   ; p_base_x
   (7,7)   1000.0   ; p_base_y
   (8,8)   1000.0   ; p_base_z
   ```

2. **Reduce momentum weights:**
   ```ini
   (0,0)   5.0      ; vcom_x
   ```

### If Forces are Too Large:

1. **Increase input scaling:**
   ```ini
   R { scaling 1e-2 }  // Forces become "more expensive"
   ```

2. **Reduce constraint penalty:**
   ```ini
   constraintPenaltyInitialValue   10.0
   ```

---

**Next:** [05_WBC_IMPLEMENTATION.md](./05_WBC_IMPLEMENTATION.md) - Whole Body Control Details

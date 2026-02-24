# Analysis 07: Xbox Controller & FSM Mode Management

## Overview

This document describes the Xbox controller integration and FSM (Finite State Machine) mode management added to `vm_anymal_standing_online` for interactive standing control of ANYmal C robot.

## Architecture

```
┌─────────────────────┐
│    Xbox Controller   │
│    (Linux input)     │
└──────────┬──────────┘
           │ XboxInput (analog sticks, buttons)
           ▼
┌─────────────────────┐
│   FSM Controller     │
│  (State machine)     │
└──────────┬──────────┘
           │ StandingTarget (height, yaw, pitch)
           ▼
┌─────────────────────┐
│  Target State Gen    │
│ (createTargetState)  │
└──────────┬──────────┘
           │ OCS2 State vector (24D)
           ▼
┌─────────────────────┐
│    MPC Solver        │
│ (GaussNewtonDDP)     │
└──────────┬──────────┘
           │ Optimal trajectory
           ▼
┌─────────────────────┐
│    WBC Torque        │
│ (CentroidalModel)    │
└──────────┬──────────┘
           │ Joint torques (12D)
           ▼
┌─────────────────────┐
│   MuJoCo Actuators   │
└─────────────────────┘
```

## FSM States

```cpp
enum class FSMState {
    PASSIVE,      // No MPC, motors relaxed
    STANDUP,      // Transition to standing (interpolate height)
    STANDING,     // Active MPC control
    RECOVERY,     // Recovery from fall
    EMERGENCY     // Emergency stop
};
```

### State Transitions

```
    [PASSIVE] ──A button──> [STANDUP] ──done──> [STANDING]
        ▲                                           │
        │                                           │
        └────────B button───────[SIT_DOWN]<─────────┘
        
    Any State ──Back button──> [EMERGENCY]
```

## Xbox Controller Mapping

### Analog Sticks
| Stick | Axis | Function | Range |
|-------|------|----------|-------|
| Left Y | `ly` | Target height | 0.35m - 0.65m |
| Left X | `lx` | (unused) | - |
| Right Y | `ry` | Target pitch | -0.3 - 0.3 rad |
| Right X | `rx` | Target yaw | -0.5 - 0.5 rad |

### Buttons
| Button | Function |
|--------|----------|
| A | Enter STANDING mode |
| B | Return to PASSIVE mode |
| X | Enter RECOVERY mode |
| Y | Reset to default pose |
| Start | Toggle MPC on/off |
| Back | Emergency stop |
| LB | Decrease height preset |
| RB | Increase height preset |

## Height Control

The target height is controlled via left stick Y axis:
- Center: 0.575m (normal stance)
- Full up: 0.65m (high stance)
- Full down: 0.35m (low stance)

```cpp
// Height calculation
double heightRange = StandingTarget::HEIGHT_HIGH - StandingTarget::HEIGHT_LOW;
double heightOffset = ly * heightRange / 2.0;  // ly is -1 to 1
target.z = StandingTarget::HEIGHT_NORMAL + heightOffset;
target.clampHeight();  // Ensure within 0.30 - 0.70m
```

## Key Files

### Headers
- `include/vm_anymal_standing_online/XboxController.h` - Xbox input handling
- `include/vm_anymal_standing_online/FSMController.h` - FSM state machine

### Sources
- `src/XboxController.cpp` - Linux input event processing
- `src/FSMController.cpp` - State transition logic
- `src/main_anymal_mujoco.cpp` - Integration with MPC/MuJoCo

## XboxController Class

```cpp
class XboxController {
public:
    XboxController();           // Auto-detect and connect
    ~XboxController();          // Auto-cleanup
    
    bool isConnected() const;   // Check connection status
    XboxInput getInput();       // Get current analog values
    
    // Button press events (latched, cleared on read)
    int getModeChangeRequest(); // -1=none, 0=PASSIVE, 1=STANDING, 2=RECOVERY
    bool wasStartPressed();     // Toggle MPC
    bool wasBackPressed();      // Emergency stop
    bool wasResetPressed();     // Reset pose
    int getHeightAdjustRequest(); // -1=down, 0=none, 1=up
    
private:
    void runThread();           // Background input processing
    std::string detectDevicePath(const std::string& nameHint);
};
```

## FSMController Class

```cpp
class FSMController {
public:
    FSMController();
    
    // State management
    void requestStateChange(FSMState newState);
    void update(double dt);
    FSMState getCurrentState() const;
    bool isTransitioning() const;
    
    // Target management
    const StandingTarget& getTarget() const;
    void adjustHeight(double delta);
    void setHeightPreset(int preset);  // 0=low, 1=normal, 2=high
    
    // Xbox integration
    void updateFromXbox(float lx, float ly, float rx, float ry, double dt);
    
    // MPC control
    bool isMpcEnabled() const;
    void setMpcEnabled(bool enabled);
    void toggleMpc();
    
    // Safety
    void emergencyStop();
    void clearEmergency();
    
    std::string getStatusString() const;
};
```

## Integration Flow

### Main Thread Structure
```cpp
int main() {
    // 1. Initialize OCS2 Robot Interface
    // 2. Initialize MPC Solver
    // 3. Initialize MuJoCo GUI
    // 3.5. Initialize Xbox Controller & FSM
    
    // 4. Start threads:
    //    - mpcThread: MPC optimization
    //    - physicsThread: MuJoCo simulation
    //    - controlThread: WBC + PD control
    //    - xboxFsmThread: Xbox input + FSM update
    
    // 5. Run GUI loop
    // 6. Cleanup
}
```

### xboxFsmThread Logic
```cpp
void xboxFsmThread() {
    while (!exitRequest) {
        // 1. Get Xbox input
        XboxInput input = xboxController->getInput();
        
        // 2. Check for mode change requests
        int modeReq = xboxController->getModeChangeRequest();
        if (modeReq == 0) fsmController->requestStateChange(FSMState::PASSIVE);
        if (modeReq == 1) fsmController->requestStateChange(FSMState::STANDUP);
        
        // 3. Check for MPC toggle
        if (xboxController->wasStartPressed()) {
            fsmController->toggleMpc();
        }
        
        // 4. Update FSM with Xbox input
        fsmController->updateFromXbox(input.lx, input.ly, input.rx, input.ry, dt);
        fsmController->update(dt);
        
        // 5. Generate target state for MPC
        if (fsmController->isMpcEnabled()) {
            g_targetState = createTargetStateFromFSM(
                fsmController->getTarget(),
                robotInterface->getInitialState());
        }
        
        sleep(5ms);
    }
}
```

### Target State Generation
```cpp
vector_t createTargetStateFromFSM(const StandingTarget& target, 
                                   const vector_t& initialState) {
    vector_t targetState = initialState;
    
    // Set target position
    targetState(6) = target.x;   // Base X
    targetState(7) = target.y;   // Base Y
    targetState(8) = target.z;   // Base height
    
    // Set target orientation (ZYX Euler -> normalized momentum)
    double yaw = target.yaw;
    double pitch = target.pitch;
    double roll = target.roll;
    
    // Convert to centroidal momentum representation
    targetState(0) = roll;   // Approximate as linear for small angles
    targetState(1) = pitch;
    targetState(2) = yaw;
    
    // Zero velocities
    targetState.segment(3, 3).setZero();  // Angular momentum
    targetState.segment(9, 3).setZero();  // Linear velocity
    
    return targetState;
}
```

## Xbox Controller Detection

The `XboxController` automatically detects connected controllers by searching for these device names:
- Microsoft Xbox Controller
- Xbox One Wireless Controller
- Microsoft Xbox Series S|X Controller
- Xbox Wireless Controller
- Microsoft X-Box One pad
- Xbox 360 Wireless Receiver

Detection method:
1. Read `/proc/bus/input/devices`
2. Find matching "Name" entry
3. Extract event handler (e.g., `eventX`)
4. Open `/dev/input/eventX`

## Usage

### Starting Simulation
```bash
cd /home/quangvd7/ros2_ws_ocs2
source install/setup.bash
./install/vm_anymal_standing_online/lib/vm_anymal_standing_online/anymal_standing_mujoco
```

### Controls Display
```
========================================
  Simulation Running!
========================================
  Space     - Play/Pause
  Backspace - Reset
  ESC       - Exit
----------------------------------------
  Xbox Controller:
    A       - Start standing (STAND_UP)
    B       - Go passive (SIT_DOWN)
    Left Y  - Change height (0.35-0.65m)
    Right Y - Pitch control
    Right X - Yaw control
========================================
```

### Status Output
```
[FSM] State: STANDING | MPC: ON | Target: [0.000, 0.000, 0.500] | Yaw: 0.100
[DEBUG] t=5.500 target_z=0.500 actual_z=0.505 optimal_z=0.502
```

## Safety Features

1. **MPC Delayed Start**: MPC waits 3 seconds for PD controller to stabilize robot
2. **Emergency Stop**: Back button immediately sets all torques to zero
3. **Height Limits**: Clamped to 0.30 - 0.70m range
4. **Torque Limits**: All joint torques clamped to ±40 Nm
5. **Graceful Degradation**: If Xbox not connected, simulation still runs with default pose

## Future Enhancements

1. **Walking Mode**: Add FSM states for locomotion
2. **Recovery Behavior**: Automatic stand-up from fallen position
3. **Keyboard Fallback**: WASD keys when Xbox not available
4. **Velocity Commands**: Add linear/angular velocity targets for walking
5. **Terrain Adaptation**: Adjust stance based on terrain slope

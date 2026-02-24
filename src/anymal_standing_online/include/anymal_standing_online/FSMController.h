#ifndef ANYMAL_STANDING_FSM_H
#define ANYMAL_STANDING_FSM_H

#include <string>
#include <memory>
#include <functional>

namespace anymal_standing_online {

/**
 * @brief FSM State enumeration for ANYmal standing control
 * 
 * Control modes:
 * - PASSIVE: No torques applied, robot free-falling
 * - PD_CONTROL: Pure PD joint control to default stance
 * - STANDING: Full MPC + WBC control
 */
enum class FSMState {
    PASSIVE,      // Motors off, no control (A button)
    PD_CONTROL,   // Pure PD control to default stance (B button)
    STANDING,     // Standing with MPC control (X button)
    RECOVERY,     // Recovery mode after fall
    EMERGENCY     // Emergency stop
};

/**
 * @brief Convert FSM state to string for display
 */
inline std::string fsmStateToString(FSMState state) {
    switch (state) {
        case FSMState::PASSIVE:    return "PASSIVE";
        case FSMState::PD_CONTROL: return "PD_CONTROL";
        case FSMState::STANDING:   return "STANDING";
        case FSMState::RECOVERY:   return "RECOVERY";
        case FSMState::EMERGENCY:  return "EMERGENCY";
        default:                   return "UNKNOWN";
    }
}

/**
 * @brief Target command structure for standing control
 */
struct StandingTarget {
    // Base pose target
    double x = 0.0;           // Target x position [m]
    double y = 0.0;           // Target y position [m]
    double z = 0.575;         // Target height [m]
    double yaw = 0.0;         // Target yaw angle [rad]
    double pitch = 0.0;       // Target pitch angle [rad]
    double roll = 0.0;        // Target roll angle [rad]
    
    // Height presets
    static constexpr double HEIGHT_LOW = 0.35;     // Low stance
    static constexpr double HEIGHT_NORMAL = 0.575; // Normal stance
    static constexpr double HEIGHT_HIGH = 0.65;    // High stance
    static constexpr double HEIGHT_MIN = 0.30;     // Minimum allowed
    static constexpr double HEIGHT_MAX = 0.70;     // Maximum allowed
    
    // Velocity limits for smooth transitions
    static constexpr double MAX_XY_VEL = 0.3;      // Max XY velocity [m/s]
    static constexpr double MAX_Z_VEL = 0.15;      // Max height change rate [m/s]
    static constexpr double MAX_YAW_VEL = 0.5;     // Max yaw rate [rad/s]
    
    void reset() {
        x = y = 0.0;
        z = HEIGHT_NORMAL;
        yaw = pitch = roll = 0.0;
    }
    
    void clampHeight() {
        if (z < HEIGHT_MIN) z = HEIGHT_MIN;
        if (z > HEIGHT_MAX) z = HEIGHT_MAX;
    }
};

/**
 * @brief Finite State Machine for ANYmal standing control
 * 
 * States:
 * - PASSIVE: No control, motors relaxed
 * - STANDUP: Transition from ground to standing
 * - STANDING: Active MPC standing control
 * - RECOVERY: Recover from fall/disturbance
 * - EMERGENCY: Emergency stop (all torques to zero)
 */
class FSMController {
public:
    FSMController();
    ~FSMController() = default;
    
    // State management
    void requestStateChange(FSMState newState);
    void update(double dt);
    FSMState getCurrentState() const { return currentState_; }
    FSMState getPreviousState() const { return previousState_; }
    bool isTransitioning() const { return isTransitioning_; }
    
    // Target management
    const StandingTarget& getTarget() const { return target_; }
    StandingTarget& getTargetMutable() { return target_; }
    void setTarget(const StandingTarget& target) { target_ = target; }
    void resetTarget() { target_.reset(); }
    
    // Height adjustment
    void adjustHeight(double delta);
    void setHeightPreset(int preset);  // 0=low, 1=normal, 2=high
    
    // Xbox controller integration
    void updateFromXbox(float lx, float ly, float rx, float ry, double dt);
    
    // Callbacks for state transitions
    using StateCallback = std::function<void(FSMState, FSMState)>;
    void setStateChangeCallback(StateCallback callback) { stateChangeCallback_ = callback; }
    
    // MPC control
    bool isMpcEnabled() const { return mpcEnabled_; }
    void setMpcEnabled(bool enabled) { mpcEnabled_ = enabled; }
    void toggleMpc() { mpcEnabled_ = !mpcEnabled_; }
    
    // Safety
    void emergencyStop();
    bool isEmergencyStopped() const { return currentState_ == FSMState::EMERGENCY; }
    void clearEmergency();
    
    // Status string for display
    std::string getStatusString() const;
    
private:
    // State transition logic
    bool canTransitionTo(FSMState newState) const;
    void executeTransition(FSMState newState);
    void onStateEnter(FSMState state);
    void onStateExit(FSMState state);
    
    // Current state
    FSMState currentState_ = FSMState::PASSIVE;
    FSMState previousState_ = FSMState::PASSIVE;
    FSMState requestedState_ = FSMState::PASSIVE;
    bool isTransitioning_ = false;
    double transitionProgress_ = 0.0;
    double stateTime_ = 0.0;
    
    // Target
    StandingTarget target_;
    StandingTarget initialTarget_;  // For interpolation during standup
    
    // Control flags
    bool mpcEnabled_ = false;
    
    // Callback
    StateCallback stateChangeCallback_;
};

}  // namespace anymal_standing_online

#endif  // ANYMAL_STANDING_FSM_H

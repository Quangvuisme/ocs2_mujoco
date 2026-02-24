#ifndef ANYMAL_WALKING_FSM_H
#define ANYMAL_WALKING_FSM_H

#include <string>
#include <memory>
#include <functional>

namespace quadruped_nmpc {

/**
 * @brief Gait types for walking control
 */
enum class GaitType {
    STANCE = 0,         // All feet on ground
    TROT = 1,           // Diagonal pairs (LF_RH / RF_LH)
    STANDING_TROT = 2,  // Trot with stance phases
    FLYING_TROT = 3,    // Trot with flight phases
    PACE = 4,           // Same side pairs (LF_LH / RF_RH)
    STANDING_PACE = 5,  // Pace with stance phases
    DYNAMIC_WALK = 6,   // 3-leg contact walk
    STATIC_WALK = 7,    // Conservative 3-leg walk
    AMBLE = 8,          // Lateral sequence
};

/**
 * @brief Convert GaitType to string
 */
inline std::string gaitTypeToString(GaitType gait) {
    switch (gait) {
        case GaitType::STANCE:        return "stance";
        case GaitType::TROT:          return "trot";
        case GaitType::STANDING_TROT: return "standing_trot";
        case GaitType::FLYING_TROT:   return "flying_trot";
        case GaitType::PACE:          return "pace";
        case GaitType::STANDING_PACE: return "standing_pace";
        case GaitType::DYNAMIC_WALK:  return "dynamic_walk";
        case GaitType::STATIC_WALK:   return "static_walk";
        case GaitType::AMBLE:         return "amble";
        default:                      return "stance";
    }
}

/**
 * @brief FSM State enumeration for ANYmal walking control
 * 
 * Control modes:
 * - PASSIVE: No torques applied, robot free-falling
 * - PD_CONTROL: Pure PD joint control to default stance
 * - STANDING: Standing with MPC control (4-leg stance)
 * - WALKING: Walking with MPC control (gait switching)
 */
enum class FSMState {
    PASSIVE,      // Motors off, no control (A button)
    PD_CONTROL,   // Pure PD control to default stance (B button)
    STANDING,     // Standing with MPC control (X button)
    WALKING,      // Walking with MPC control (Y button)
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
        case FSMState::WALKING:    return "WALKING";
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
 * @brief Target command structure for walking control
 */
struct WalkingTarget {
    // Velocity commands
    double vx = 0.0;          // Forward velocity [m/s]
    double vy = 0.0;          // Lateral velocity [m/s]
    double yaw_rate = 0.0;    // Yaw rate [rad/s]
    
    // Height target
    double z = 0.50;          // Target height [m]
    
    // Gait type
    GaitType gait = GaitType::STANCE;
    
    // Velocity limits
    static constexpr double MAX_VX = 0.5;          // Max forward velocity [m/s]
    static constexpr double MAX_VY = 0.3;          // Max lateral velocity [m/s]
    static constexpr double MAX_YAW_RATE = 0.5;    // Max yaw rate [rad/s]
    static constexpr double HEIGHT_MIN = 0.35;     // Minimum allowed
    static constexpr double HEIGHT_MAX = 0.60;     // Maximum allowed (lower for walking)
    static constexpr double HEIGHT_WALKING = 0.50; // Default walking height
    
    void reset() {
        vx = vy = yaw_rate = 0.0;
        z = HEIGHT_WALKING;
        gait = GaitType::STANCE;
    }
    
    void clamp() {
        // Clamp velocities
        if (vx > MAX_VX) vx = MAX_VX;
        if (vx < -MAX_VX) vx = -MAX_VX;
        if (vy > MAX_VY) vy = MAX_VY;
        if (vy < -MAX_VY) vy = -MAX_VY;
        if (yaw_rate > MAX_YAW_RATE) yaw_rate = MAX_YAW_RATE;
        if (yaw_rate < -MAX_YAW_RATE) yaw_rate = -MAX_YAW_RATE;
        // Clamp height
        if (z < HEIGHT_MIN) z = HEIGHT_MIN;
        if (z > HEIGHT_MAX) z = HEIGHT_MAX;
    }
    
    // Apply deadzone to joystick input
    static double applyDeadzone(double value, double deadzone = 0.15) {
        if (std::abs(value) < deadzone) return 0.0;
        return (value - (value > 0 ? deadzone : -deadzone)) / (1.0 - deadzone);
    }
};

/**
 * @brief Finite State Machine for ANYmal walking control
 * 
 * States:
 * - PASSIVE: No control, motors relaxed
 * - PD_CONTROL: Pure PD joint control to default stance
 * - STANDING: Active MPC standing control (4-leg stance)
 * - WALKING: Active MPC walking control (gait switching)
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
    
    // Standing target management
    const StandingTarget& getTarget() const { return target_; }
    StandingTarget& getTargetMutable() { return target_; }
    void setTarget(const StandingTarget& target) { target_ = target; }
    void resetTarget() { target_.reset(); }
    
    // Walking target management
    const WalkingTarget& getWalkingTarget() const { return walkingTarget_; }
    WalkingTarget& getWalkingTargetMutable() { return walkingTarget_; }
    void setWalkingTarget(const WalkingTarget& target) { walkingTarget_ = target; }
    void resetWalkingTarget() { walkingTarget_.reset(); }
    
    // Gait management
    void setGait(GaitType gait);
    GaitType getCurrentGait() const { return walkingTarget_.gait; }
    void nextGait();  // Cycle through gaits
    
    // Height adjustment
    void adjustHeight(double delta);
    void setHeightPreset(int preset);  // 0=low, 1=normal, 2=high
    
    // Xbox controller integration
    void updateFromXbox(float lx, float ly, float rx, float ry, double dt);
    void updateWalkingFromXbox(float lx, float ly, float rx, float ry, double dt);
    
    // Callbacks for state transitions
    using StateCallback = std::function<void(FSMState, FSMState)>;
    void setStateChangeCallback(StateCallback callback) { stateChangeCallback_ = callback; }
    
    // Gait change callback (for notifying MPC)
    using GaitCallback = std::function<void(GaitType)>;
    void setGaitChangeCallback(GaitCallback callback) { gaitChangeCallback_ = callback; }
    
    // MPC control
    bool isMpcEnabled() const { return mpcEnabled_; }
    void setMpcEnabled(bool enabled) { mpcEnabled_ = enabled; }
    void toggleMpc() { mpcEnabled_ = !mpcEnabled_; }
    
    // Walking mode
    bool isWalking() const { return currentState_ == FSMState::WALKING; }
    bool isStanding() const { return currentState_ == FSMState::STANDING; }
    
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
    
    // Standing target
    StandingTarget target_;
    StandingTarget initialTarget_;  // For interpolation during standup
    
    // Walking target
    WalkingTarget walkingTarget_;
    
    // Control flags
    bool mpcEnabled_ = false;
    
    // Callbacks
    StateCallback stateChangeCallback_;
    GaitCallback gaitChangeCallback_;
};

}  // namespace quadruped_nmpc

#endif  // ANYMAL_WALKING_FSM_H

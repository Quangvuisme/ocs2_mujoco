#include <anymal_standing_online/FSMController.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace anymal_standing_online {

FSMController::FSMController() {
    target_.reset();
    initialTarget_.reset();
}

void FSMController::requestStateChange(FSMState newState) {
    if (currentState_ == FSMState::EMERGENCY && newState != FSMState::PASSIVE) {
        std::cout << "[FSM] Cannot transition from EMERGENCY to " 
                  << fsmStateToString(newState) << ". Go to PASSIVE first." << std::endl;
        return;
    }
    
    if (canTransitionTo(newState)) {
        requestedState_ = newState;
        std::cout << "[FSM] State change requested: " << fsmStateToString(currentState_) 
                  << " -> " << fsmStateToString(newState) << std::endl;
    } else {
        std::cout << "[FSM] Cannot transition from " << fsmStateToString(currentState_) 
                  << " to " << fsmStateToString(newState) << std::endl;
    }
}

bool FSMController::canTransitionTo(FSMState newState) const {
    // Emergency can go anywhere
    if (newState == FSMState::EMERGENCY) return true;
    
    // From EMERGENCY, can only go to PASSIVE
    if (currentState_ == FSMState::EMERGENCY) {
        return newState == FSMState::PASSIVE;
    }
    
    // Allow direct transitions between PASSIVE, PD_CONTROL, and STANDING
    // This enables Xbox button control:
    // A = PASSIVE, B = PD_CONTROL, X = STANDING
    switch (currentState_) {
        case FSMState::PASSIVE:
            return newState == FSMState::PD_CONTROL || 
                   newState == FSMState::STANDING;
            
        case FSMState::PD_CONTROL:
            return newState == FSMState::PASSIVE || 
                   newState == FSMState::STANDING ||
                   newState == FSMState::RECOVERY;
            
        case FSMState::STANDING:
            return newState == FSMState::PASSIVE || 
                   newState == FSMState::PD_CONTROL ||
                   newState == FSMState::RECOVERY;
            
        case FSMState::RECOVERY:
            return newState == FSMState::STANDING || 
                   newState == FSMState::PD_CONTROL ||
                   newState == FSMState::PASSIVE;
            
        default:
            return false;
    }
}

void FSMController::executeTransition(FSMState newState) {
    onStateExit(currentState_);
    
    previousState_ = currentState_;
    currentState_ = newState;
    stateTime_ = 0.0;
    transitionProgress_ = 0.0;
    isTransitioning_ = false;
    
    onStateEnter(newState);
    
    if (stateChangeCallback_) {
        stateChangeCallback_(previousState_, currentState_);
    }
    
    std::cout << "[FSM] State changed: " << fsmStateToString(previousState_) 
              << " -> " << fsmStateToString(currentState_) << std::endl;
}

void FSMController::onStateEnter(FSMState state) {
    switch (state) {
        case FSMState::PASSIVE:
            mpcEnabled_ = false;
            std::cout << "[FSM] Entered PASSIVE - Motors OFF" << std::endl;
            break;
            
        case FSMState::PD_CONTROL:
            mpcEnabled_ = false;
            target_.reset();  // Reset to default stance
            std::cout << "[FSM] Entered PD_CONTROL - PD to default stance" << std::endl;
            break;
            
        case FSMState::STANDING:
            mpcEnabled_ = true;
            target_.reset();  // Start from default stance
            std::cout << "[FSM] Entered STANDING - MPC control active" << std::endl;
            break;
            
        case FSMState::RECOVERY:
            mpcEnabled_ = false;
            isTransitioning_ = true;
            target_.reset();
            std::cout << "[FSM] Entered RECOVERY mode" << std::endl;
            break;
            
        case FSMState::EMERGENCY:
            mpcEnabled_ = false;
            break;
    }
}

void FSMController::onStateExit(FSMState state) {
    switch (state) {
        case FSMState::PD_CONTROL:
            // Nothing special
            break;
            
        case FSMState::RECOVERY:
            isTransitioning_ = false;
            break;
            
        default:
            break;
    }
}

void FSMController::update(double dt) {
    stateTime_ += dt;
    
    // Check for requested state change
    if (requestedState_ != currentState_) {
        executeTransition(requestedState_);
    }
    
    // State-specific updates
    switch (currentState_) {
        case FSMState::PASSIVE:
            // Nothing to do
            break;
            
        case FSMState::PD_CONTROL:
            // Nothing to do - control thread handles PD
            break;
            
        case FSMState::STANDING:
            // MPC handles control
            break;
            
        case FSMState::RECOVERY:
            // Gradually recover
            transitionProgress_ += dt / 3.0;  // 3 second recovery
            if (transitionProgress_ >= 1.0) {
                requestStateChange(FSMState::PD_CONTROL);
            }
            break;
            
        default:
            break;
    }
    
    // Always clamp height
    target_.clampHeight();
}

void FSMController::adjustHeight(double delta) {
    if (currentState_ != FSMState::STANDING) return;
    
    target_.z += delta;
    target_.clampHeight();
}

void FSMController::setHeightPreset(int preset) {
    if (currentState_ != FSMState::STANDING) return;
    
    switch (preset) {
        case 0: target_.z = StandingTarget::HEIGHT_LOW; break;
        case 1: target_.z = StandingTarget::HEIGHT_NORMAL; break;
        case 2: target_.z = StandingTarget::HEIGHT_HIGH; break;
    }
}

void FSMController::updateFromXbox(float lx, float ly, float rx, float ry, double dt) {
    if (currentState_ != FSMState::STANDING) return;
    
    // Left stick: XY position adjustment
    target_.x += ly * StandingTarget::MAX_XY_VEL * dt;
    target_.y += lx * StandingTarget::MAX_XY_VEL * dt;
    
    // Right stick Y: Height adjustment
    target_.z += ry * StandingTarget::MAX_Z_VEL * dt;
    target_.clampHeight();
    
    // Right stick X: Yaw adjustment
    target_.yaw += rx * StandingTarget::MAX_YAW_VEL * dt;
    
    // Limit yaw to [-pi, pi]
    while (target_.yaw > M_PI) target_.yaw -= 2 * M_PI;
    while (target_.yaw < -M_PI) target_.yaw += 2 * M_PI;
    
    // Limit XY position
    const double MAX_XY = 0.3;  // 30cm max offset
    if (target_.x > MAX_XY) target_.x = MAX_XY;
    if (target_.x < -MAX_XY) target_.x = -MAX_XY;
    if (target_.y > MAX_XY) target_.y = MAX_XY;
    if (target_.y < -MAX_XY) target_.y = -MAX_XY;
}

void FSMController::emergencyStop() {
    requestedState_ = FSMState::EMERGENCY;
    executeTransition(FSMState::EMERGENCY);
    std::cout << "[FSM] !!! EMERGENCY STOP !!!" << std::endl;
}

void FSMController::clearEmergency() {
    if (currentState_ == FSMState::EMERGENCY) {
        requestStateChange(FSMState::PASSIVE);
    }
}

std::string FSMController::getStatusString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "State: " << fsmStateToString(currentState_);
    oss << " | MPC: " << (mpcEnabled_ ? "ON" : "OFF");
    oss << " | Target: [" << target_.x << ", " << target_.y << ", " << target_.z << "]";
    oss << " | Yaw: " << target_.yaw;
    if (isTransitioning_) {
        oss << " | Transition: " << (int)(transitionProgress_ * 100) << "%";
    }
    return oss.str();
}

}  // namespace anymal_standing_online

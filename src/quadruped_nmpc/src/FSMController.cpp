#include <quadruped_nmpc/FSMController.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace quadruped_nmpc {

FSMController::FSMController() {
    target_.reset();
    initialTarget_.reset();
    walkingTarget_.reset();
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
    
    // Allow direct transitions between PASSIVE, PD_CONTROL, STANDING, and WALKING
    // This enables Xbox button control:
    // A = PASSIVE, B = PD_CONTROL, X = STANDING, Y = WALKING
    switch (currentState_) {
        case FSMState::PASSIVE:
            return newState == FSMState::PD_CONTROL || 
                   newState == FSMState::STANDING;
            
        case FSMState::PD_CONTROL:
            return newState == FSMState::PASSIVE || 
                   newState == FSMState::STANDING ||
                   newState == FSMState::WALKING ||
                   newState == FSMState::RECOVERY;
            
        case FSMState::STANDING:
            return newState == FSMState::PASSIVE || 
                   newState == FSMState::PD_CONTROL ||
                   newState == FSMState::WALKING ||
                   newState == FSMState::RECOVERY;
                   
        case FSMState::WALKING:
            return newState == FSMState::PASSIVE || 
                   newState == FSMState::PD_CONTROL ||
                   newState == FSMState::STANDING ||
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
            // NOTE: Don't reset target here - main.cpp will set proper initial values
            // including continuous yaw from g_continuousYaw
            // target_.reset();  // Commented out to preserve yaw continuity
            walkingTarget_.gait = GaitType::STANCE;  // Use stance gait for standing
            if (gaitChangeCallback_) {
                gaitChangeCallback_(GaitType::STANCE);
            }
            std::cout << "[FSM] Entered STANDING - MPC control active (STANCE)" << std::endl;
            break;
            
        case FSMState::WALKING:
            mpcEnabled_ = true;
            walkingTarget_.reset();
            walkingTarget_.gait = GaitType::TROT;  // Default to trot gait for walking
            if (gaitChangeCallback_) {
                gaitChangeCallback_(GaitType::TROT);
            }
            std::cout << "[FSM] Entered WALKING - MPC control active (TROT)" << std::endl;
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
            
        case FSMState::WALKING:
            // Reset velocity commands when exiting walking
            walkingTarget_.vx = 0.0;
            walkingTarget_.vy = 0.0;
            walkingTarget_.yaw_rate = 0.0;
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
            target_.clampHeight();
            break;
            
        case FSMState::WALKING:
            // MPC handles control, clamp walking targets
            walkingTarget_.clamp();
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

void FSMController::setGait(GaitType gait) {
    if (currentState_ != FSMState::WALKING && currentState_ != FSMState::STANDING) {
        std::cout << "[FSM] Cannot change gait in " << fsmStateToString(currentState_) << " state" << std::endl;
        return;
    }
    
    GaitType oldGait = walkingTarget_.gait;
    walkingTarget_.gait = gait;
    
    if (gaitChangeCallback_) {
        gaitChangeCallback_(gait);
    }
    
    std::cout << "[FSM] Gait changed: " << gaitTypeToString(oldGait) 
              << " -> " << gaitTypeToString(gait) << std::endl;
}

void FSMController::nextGait() {
    if (currentState_ != FSMState::WALKING) return;
    
    // Cycle through walking-friendly gaits: TROT -> STANDING_TROT -> PACE -> DYNAMIC_WALK -> TROT
    GaitType currentGait = walkingTarget_.gait;
    GaitType newGait;
    
    switch (currentGait) {
        case GaitType::TROT:          newGait = GaitType::STANDING_TROT; break;
        case GaitType::STANDING_TROT: newGait = GaitType::PACE; break;
        case GaitType::PACE:          newGait = GaitType::DYNAMIC_WALK; break;
        case GaitType::DYNAMIC_WALK:  newGait = GaitType::TROT; break;
        default:                      newGait = GaitType::TROT; break;
    }
    
    setGait(newGait);
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
    // NOTE: Don't wrap yaw to [-pi, pi] because MPC uses continuous yaw
    // This allows smooth transitions when yaw exceeds ±pi during walking
    target_.yaw += rx * StandingTarget::MAX_YAW_VEL * dt;
    
    // Limit XY position
    const double MAX_XY = 0.3;  // 30cm max offset
    if (target_.x > MAX_XY) target_.x = MAX_XY;
    if (target_.x < -MAX_XY) target_.x = -MAX_XY;
    if (target_.y > MAX_XY) target_.y = MAX_XY;
    if (target_.y < -MAX_XY) target_.y = -MAX_XY;
}

void FSMController::updateWalkingFromXbox(float lx, float ly, float rx, float ry, double dt) {
    if (currentState_ != FSMState::WALKING) return;
    
    // Apply deadzone
    float lx_filtered = WalkingTarget::applyDeadzone(lx);
    float ly_filtered = WalkingTarget::applyDeadzone(ly);
    float rx_filtered = WalkingTarget::applyDeadzone(rx);
    float ry_filtered = WalkingTarget::applyDeadzone(ry);
    
    // Left stick: Velocity commands
    // ly (forward/back): vx (forward velocity)
    // lx (left/right): vy (lateral velocity)
    walkingTarget_.vx = ly_filtered * WalkingTarget::MAX_VX;
    walkingTarget_.vy = lx_filtered * WalkingTarget::MAX_VY;
    
    // Right stick X: Yaw rate
    walkingTarget_.yaw_rate = rx_filtered * WalkingTarget::MAX_YAW_RATE;
    
    // Right stick Y: Height adjustment (slower for walking)
    walkingTarget_.z += ry_filtered * 0.05 * dt;  // Slow height change
    
    // Clamp all values
    walkingTarget_.clamp();
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
    
    if (currentState_ == FSMState::STANDING) {
        oss << " | Target: [" << target_.x << ", " << target_.y << ", " << target_.z << "]";
        oss << " | Yaw: " << target_.yaw;
    } else if (currentState_ == FSMState::WALKING) {
        oss << " | Gait: " << gaitTypeToString(walkingTarget_.gait);
        oss << " | Vel: [" << walkingTarget_.vx << ", " << walkingTarget_.vy << "]";
        oss << " | YawRate: " << walkingTarget_.yaw_rate;
        oss << " | H: " << walkingTarget_.z;
    }
    
    if (isTransitioning_) {
        oss << " | Transition: " << (int)(transitionProgress_ * 100) << "%";
    }
    return oss.str();
}

}  // namespace quadruped_nmpc

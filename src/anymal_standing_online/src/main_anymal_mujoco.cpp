/**
 * @file main_anymal_mujoco.cpp
 * @brief ANYmal C Standing Control with MuJoCo + OCS2 Centroidal MPC
 *        With Xbox Controller and FSM mode management
 * 
 */

#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <cmath>
#include <fstream>

// OCS2
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

// Interface
#include <anymal_standing_online/LeggedRobotInterface.h>
#include <anymal_standing_online/package_path.h>
#include <anymal_standing_online/common/utils.h>

// Xbox Controller and FSM
#include <anymal_standing_online/XboxController.h>
#include <anymal_standing_online/KeyboardController.h>
#include <anymal_standing_online/FSMController.h>

// MuJoCo
#include <mujoco/mujoco.h>
#include "mujoco_gui/glfw_adapter.h"
#include "mujoco_gui/simulate.h"

using namespace ocs2;
using namespace anymal_standing_online;

// =============================================================================
// Global Variables
// =============================================================================
std::atomic<bool> exitRequest{false};
std::atomic<bool> mpcReady{false};
std::atomic<bool> g_mpcInitialized{false};

// MPC objects
std::unique_ptr<LeggedRobotInterface> robotInterface;
std::unique_ptr<GaussNewtonDDP_MPC> mpcSolver;
std::unique_ptr<MPC_MRT_Interface> mpcMrt;
std::unique_ptr<CentroidalModelRbdConversions> rbdConversions;

// Xbox Controller and FSM
std::unique_ptr<XboxController> xboxController;
std::unique_ptr<KeyboardController> keyboardController;
std::unique_ptr<FSMController> fsmController;
std::mutex g_fsmMutex;

// Current optimal trajectory
vector_t g_optimalState;
vector_t g_optimalInput;
std::mutex g_mpcMutex;

// Current target state (controlled by FSM/Xbox)
vector_t g_targetState;
std::mutex g_targetMutex;

// Control torques
Eigen::Matrix<double, 12, 1> g_torques;
std::mutex g_torqueMutex;

constexpr double KP = 150.0;  // Increased for better tracking
constexpr double KD = 5.0;
constexpr double MAX_TORQUE = 80.0;

// Control modes
constexpr bool USE_MPC_TRAJECTORY = true;   // Use MPC optimal trajectory
constexpr bool USE_WBC_TORQUE = true;       // Use Whole Body Control (τ = RNEA with contact forces)

// Config paths
std::string g_taskFile;
std::string g_urdfFile;
std::string g_referenceFile;
std::string g_mujocoModel;

// =============================================================================
// Helper: Quaternion to ZYX Euler
// =============================================================================
void quatToZYXEuler(double qw, double qx, double qy, double qz,
                    double& yaw, double& pitch, double& roll) {
    // ZYX convention (yaw, pitch, roll)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2.0 * (qw * qy - qz * qx);
    pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// =============================================================================
// Convert MuJoCo state to RBD state (for CentroidalModelRbdConversions)
// RBD state format:
// [0:3]  base orientation (ZYX Euler: yaw, pitch, roll)
// [3:6]  base position (x, y, z)
// [6:18] joint positions (LF, LH, RF, RH) - OCS2 order
// [18:21] base angular velocity (world frame: wx, wy, wz)
// [21:24] base linear velocity (world frame: vx, vy, vz)
// [24:36] joint velocities (LF, LH, RF, RH) - OCS2 order
// 
// MuJoCo/URDF order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
// OCS2 order:        LF(0-2), LH(3-5), RF(6-8), RH(9-11)
// =============================================================================
vector_t mujocoToRbdState(const double* qpos, const double* qvel, 
                          const CentroidalModelInfo& info) {
    const int nJoints = 12;
    vector_t rbdState = vector_t::Zero(2 * (6 + nJoints));
    
    // Base orientation (MuJoCo: quaternion w,x,y,z -> ZYX Euler)
    double qw = qpos[3], qx = qpos[4], qy = qpos[5], qz = qpos[6];
    double yaw, pitch, roll;
    quatToZYXEuler(qw, qx, qy, qz, yaw, pitch, roll);
    
    rbdState(0) = yaw;
    rbdState(1) = pitch;
    rbdState(2) = roll;
    
    // Base position
    rbdState(3) = qpos[0];
    rbdState(4) = qpos[1];
    rbdState(5) = qpos[2];
    
    // Joint positions: Map MuJoCo (LF,RF,LH,RH) -> OCS2 (LF,LH,RF,RH)
    // MuJoCo: qpos[7+0..2]=LF, qpos[7+3..5]=RF, qpos[7+6..8]=LH, qpos[7+9..11]=RH
    // OCS2:   rbdState[6+0..2]=LF, rbdState[6+3..5]=LH, rbdState[6+6..8]=RF, rbdState[6+9..11]=RH
    // LF -> LF (0-2)
    rbdState(6 + 0) = qpos[7 + 0];
    rbdState(6 + 1) = qpos[7 + 1];
    rbdState(6 + 2) = qpos[7 + 2];
    // LH -> index 3-5 in OCS2, but index 6-8 in MuJoCo
    rbdState(6 + 3) = qpos[7 + 6];
    rbdState(6 + 4) = qpos[7 + 7];
    rbdState(6 + 5) = qpos[7 + 8];
    // RF -> index 6-8 in OCS2, but index 3-5 in MuJoCo
    rbdState(6 + 6) = qpos[7 + 3];
    rbdState(6 + 7) = qpos[7 + 4];
    rbdState(6 + 8) = qpos[7 + 5];
    // RH -> index 9-11 in OCS2, same as MuJoCo
    rbdState(6 + 9) = qpos[7 + 9];
    rbdState(6 + 10) = qpos[7 + 10];
    rbdState(6 + 11) = qpos[7 + 11];
    
    // Base angular velocity (world frame)
    rbdState(18) = qvel[3];  // wx
    rbdState(19) = qvel[4];  // wy
    rbdState(20) = qvel[5];  // wz
    
    // Base linear velocity (world frame)
    rbdState(21) = qvel[0];  // vx
    rbdState(22) = qvel[1];  // vy
    rbdState(23) = qvel[2];  // vz
    
    // Joint velocities: same mapping as positions
    // LF
    rbdState(24 + 0) = qvel[6 + 0];
    rbdState(24 + 1) = qvel[6 + 1];
    rbdState(24 + 2) = qvel[6 + 2];
    // LH
    rbdState(24 + 3) = qvel[6 + 6];
    rbdState(24 + 4) = qvel[6 + 7];
    rbdState(24 + 5) = qvel[6 + 8];
    // RF
    rbdState(24 + 6) = qvel[6 + 3];
    rbdState(24 + 7) = qvel[6 + 4];
    rbdState(24 + 8) = qvel[6 + 5];
    // RH
    rbdState(24 + 9) = qvel[6 + 9];
    rbdState(24 + 10) = qvel[6 + 10];
    rbdState(24 + 11) = qvel[6 + 11];
    
    return rbdState;
}

// =============================================================================
// Convert MuJoCo state to OCS2 Centroidal state
// =============================================================================
vector_t mujocoToOcs2State(const double* qpos, const double* qvel, 
                           const CentroidalModelInfo& info) {
    if (!rbdConversions) {
        throw std::runtime_error("rbdConversions not initialized!");
    }
    
    vector_t rbdState = mujocoToRbdState(qpos, qvel, info);
    return rbdConversions->computeCentroidalStateFromRbdModel(rbdState);
}

// =============================================================================
// Extract target joint positions from OCS2 state and map to MuJoCo order
// OCS2 order: LF(0-2), LH(3-5), RF(6-8), RH(9-11)
// MuJoCo order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
// =============================================================================
void extractJointTargets(const vector_t& ocs2State, double* targetJoints) {
    // OCS2 centroidal state joint positions start at index 12
    // Map OCS2 (LF,LH,RF,RH) -> MuJoCo (LF,RF,LH,RH)
    
    // LF: OCS2[12-14] -> MuJoCo[0-2]
    targetJoints[0] = ocs2State(12 + 0);
    targetJoints[1] = ocs2State(12 + 1);
    targetJoints[2] = ocs2State(12 + 2);
    
    // RF: OCS2[18-20] -> MuJoCo[3-5]
    targetJoints[3] = ocs2State(12 + 6);
    targetJoints[4] = ocs2State(12 + 7);
    targetJoints[5] = ocs2State(12 + 8);
    
    // LH: OCS2[15-17] -> MuJoCo[6-8]
    targetJoints[6] = ocs2State(12 + 3);
    targetJoints[7] = ocs2State(12 + 4);
    targetJoints[8] = ocs2State(12 + 5);
    
    // RH: OCS2[21-23] -> MuJoCo[9-11]
    targetJoints[9] = ocs2State(12 + 9);
    targetJoints[10] = ocs2State(12 + 10);
    targetJoints[11] = ocs2State(12 + 11);
}

// =============================================================================
// Compute WBC torques using CentroidalModelRbdConversions with PD feedback
// This converts MPC contact forces to joint torques via RNEA + PD
// Output format: OCS2 order (LF, LH, RF, RH)
// =============================================================================
vector_t computeWbcTorques(const vector_t& desiredState, const vector_t& desiredInput,
                           const vector_t& measuredRbdState) {
    if (!rbdConversions) {
        return vector_t::Zero(18);  // 6 base wrench + 12 joint torques
    }
    
    // Zero joint accelerations for standing (quasi-static assumption)
    const auto& info = robotInterface->getCentroidalModelInfo();
    vector_t jointAccelerations = vector_t::Zero(info.actuatedDofNum);
    
    // PD gains: [base(6), joints(12)]
    // Base gains are zero since we don't control base directly
    // Joint gains provide position/velocity tracking
    vector_t pGains = vector_t::Zero(info.generalizedCoordinatesNum);
    vector_t dGains = vector_t::Zero(info.generalizedCoordinatesNum);
    
    // Set joint PD gains (indices 6-17 for joints)
    for (int i = 0; i < 12; i++) {
        pGains(6 + i) = 100.0;   // Position gain
        dGains(6 + i) = 5.0;     // Velocity gain
    }
    
    // Compute torques using RNEA with contact forces + PD feedback
    // τ = RNEA(q, v, a + Kp*(q_des - q) + Kd*(v_des - v), f_ext)
    return rbdConversions->computeRbdTorqueFromCentroidalModelPD(
        desiredState, desiredInput, jointAccelerations,
        measuredRbdState, pGains, dGains);
}

// =============================================================================
// Extract joint torques from WBC output and convert to MuJoCo order
// WBC output: [base_wrench(6), joint_torques_OCS2_order(12)]
// MuJoCo order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
// OCS2 order:   LF(0-2), LH(3-5), RF(6-8), RH(9-11)
// =============================================================================
void extractJointTorques(const vector_t& wbcTorques, Eigen::Matrix<double, 12, 1>& mujocoTorques) {
    // Skip base wrench (first 6 elements), get joint torques
    // LF: OCS2[6-8] -> MuJoCo[0-2]
    mujocoTorques(0) = wbcTorques(6 + 0);
    mujocoTorques(1) = wbcTorques(6 + 1);
    mujocoTorques(2) = wbcTorques(6 + 2);
    
    // RF: OCS2[12-14] -> MuJoCo[3-5]
    mujocoTorques(3) = wbcTorques(6 + 6);
    mujocoTorques(4) = wbcTorques(6 + 7);
    mujocoTorques(5) = wbcTorques(6 + 8);
    
    // LH: OCS2[9-11] -> MuJoCo[6-8]
    mujocoTorques(6) = wbcTorques(6 + 3);
    mujocoTorques(7) = wbcTorques(6 + 4);
    mujocoTorques(8) = wbcTorques(6 + 5);
    
    // RH: OCS2[15-17] -> MuJoCo[9-11]
    mujocoTorques(9) = wbcTorques(6 + 9);
    mujocoTorques(10) = wbcTorques(6 + 10);
    mujocoTorques(11) = wbcTorques(6 + 11);
}

// =============================================================================
// Create target state from FSM target
// =============================================================================
vector_t createTargetStateFromFSM(const StandingTarget& target, const vector_t& defaultState) {
    vector_t targetState = defaultState;
    
    // Update base position (indices 6-8 in OCS2 centroidal state)
    targetState(6) = target.x;      // x position
    targetState(7) = target.y;      // y position  
    targetState(8) = target.z;      // z height
    
    // Update base orientation (indices 9-11: yaw, pitch, roll)
    targetState(9) = target.yaw;    // yaw (around z)
    targetState(10) = target.pitch; // pitch (around y)
    targetState(11) = target.roll;  // roll (around x)
    
    return targetState;
}

// =============================================================================
// Xbox Controller and FSM Thread
// =============================================================================
void xboxFsmThread() {
    std::cout << "[Xbox/FSM Thread] Starting..." << std::endl;
    
    auto lastTime = std::chrono::steady_clock::now();
    double dt = 0.01;  // 100 Hz update rate
    
    // Initialize target state
    if (robotInterface) {
        std::lock_guard<std::mutex> lock(g_targetMutex);
        g_targetState = robotInterface->getInitialState();
    }
    
    while (!exitRequest) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastTime).count();
        
        if (elapsed >= dt) {
            lastTime = now;
            
            // Read Xbox input
            XboxInput input;
            if (xboxController && xboxController->isConnected()) {
                input = xboxController->getInput();
                
                // Check mode change request from Xbox buttons
                // A = PASSIVE (0), B = PD_CONTROL (1), X = STANDING (2)
                int modeReq = xboxController->getModeChangeRequest();
                if (modeReq >= 0) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    switch (modeReq) {
                        case 0:  // A button - PASSIVE
                            fsmController->requestStateChange(FSMState::PASSIVE);
                            break;
                        case 1:  // B button - PD_CONTROL
                            fsmController->requestStateChange(FSMState::PD_CONTROL);
                            break;
                        case 2:  // X button - STANDING (MPC)
                            fsmController->requestStateChange(FSMState::STANDING);
                            break;
                    }
                }
                
                // Check start button (toggle MPC)
                if (xboxController->wasStartPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->toggleMpc();
                    std::cout << "[Xbox] MPC toggled: " 
                              << (fsmController->isMpcEnabled() ? "ON" : "OFF") << std::endl;
                }
                
                // Check back button (emergency stop)
                if (xboxController->wasBackPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->emergencyStop();
                }
                
                // Check reset button (Y)
                if (xboxController->wasResetPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->resetTarget();
                    std::cout << "[Xbox] Target reset to default" << std::endl;
                }
                
                // Check height adjustment (LB/RB)
                int heightAdj = xboxController->getHeightAdjustRequest();
                if (heightAdj != 0) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->adjustHeight(heightAdj * 0.05);  // 5cm per press
                    std::cout << "[Xbox] Height adjusted: " 
                              << fsmController->getTarget().z << " m" << std::endl;
                }
                
                // Update FSM with Xbox input (joystick control)
                {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->updateFromXbox(input.lx, input.ly, input.rx, input.ry, dt);
                }
            }
            
            // Also check keyboard input
            KeyboardInput kbInput;
            if (keyboardController && keyboardController->isConnected()) {
                kbInput = keyboardController->getInput();
                
                // Merge keyboard movement with Xbox
                if (std::abs(kbInput.lx) > 0.01f || std::abs(kbInput.ly) > 0.01f ||
                    std::abs(kbInput.rx) > 0.01f || std::abs(kbInput.ry) > 0.01f) {
                    input.lx = kbInput.lx;
                    input.ly = kbInput.ly;
                    input.rx = kbInput.rx;
                    input.ry = kbInput.ry;
                }
                
                // Keyboard mode change
                int kbModeReq = keyboardController->getModeChangeRequest();
                if (kbModeReq >= 0) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    switch (kbModeReq) {
                        case 0: fsmController->requestStateChange(FSMState::PASSIVE); break;
                        case 1: fsmController->requestStateChange(FSMState::STANDING); break;
                        case 2: fsmController->requestStateChange(FSMState::PD_CONTROL); break;
                    }
                }
                
                // Keyboard start (Enter)
                if (keyboardController->wasStartPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->toggleMpc();
                    std::cout << "[Keyboard] MPC: " << (fsmController->isMpcEnabled() ? "ON" : "OFF") << std::endl;
                }
                
                // Keyboard back (Q = emergency stop)
                if (keyboardController->wasBackPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->emergencyStop();
                }
                
                // Keyboard reset (R or 4)
                if (keyboardController->wasResetPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->resetTarget();
                    std::cout << "[Keyboard] Target reset" << std::endl;
                }
                
                // Keyboard height adjustment (-/=)
                int kbHeightAdj = keyboardController->getHeightAdjustRequest();
                if (kbHeightAdj != 0) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->adjustHeight(kbHeightAdj * 0.05);
                }
            }
            
            // Update FSM state machine
            {
                std::lock_guard<std::mutex> lock(g_fsmMutex);
                fsmController->update(dt);
                
                // Create target state from FSM
                if (robotInterface) {
                    std::lock_guard<std::mutex> targetLock(g_targetMutex);
                    g_targetState = createTargetStateFromFSM(
                        fsmController->getTarget(),
                        robotInterface->getInitialState());
                }
            }
            
            // Print status periodically
            static int printCount = 0;
            if (++printCount % 100 == 0) {  // Every 1 second
                std::lock_guard<std::mutex> lock(g_fsmMutex);
                std::cout << "[FSM] " << fsmController->getStatusString() << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    std::cout << "[Xbox/FSM Thread] Stopped" << std::endl;
}

// =============================================================================
// MPC Thread 
// =============================================================================
void mpcThread() {
    std::cout << "[MPC Thread] Starting..." << std::endl;
    
    auto lastTime = std::chrono::steady_clock::now();
    const double mpcPeriod = 1.0 / robotInterface->mpcSettings().mpcDesiredFrequency_;
    
    while (!exitRequest) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastTime).count();
        
        if (elapsed >= mpcPeriod && mpcReady.load()) {
            lastTime = now;
            
            try {
                std::lock_guard<std::mutex> lock(g_mpcMutex);
                mpcMrt->advanceMpc();
            } catch (const std::exception& e) {
                std::cerr << "[MPC Thread] Error: " << e.what() << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    std::cout << "[MPC Thread] Stopped" << std::endl;
}

// =============================================================================
// Control Thread (WBC + PD tracking MPC trajectory)
// =============================================================================
void controlThread(mujoco::Simulate* sim) {
    std::cout << "[Control Thread] Started" << std::endl;
    
    g_torques.setZero();
    
    // Default joint targets (ANYmal C standing)
    double defaultJoints[12] = {
        -0.25,  0.60, -0.85,   // LF
         0.25,  0.60, -0.85,   // RF
        -0.25, -0.60,  0.85,   // LH
         0.25, -0.60,  0.85    // RH
    };
    double targetJoints[12];
    std::copy(defaultJoints, defaultJoints + 12, targetJoints);
    
    // Smooth PD interpolation variables
    double pdStartJoints[12] = {0};      // Starting joint positions for interpolation
    double pdCurrentTarget[12] = {0};    // Current interpolated target
    double pdInterpolationStartTime = -1.0;
    const double pdInterpolationDuration = 1.5;  // 1.5 seconds to reach target
    bool pdInterpolationActive = false;
    FSMState lastFsmState = FSMState::PASSIVE;
    
    // Continuous yaw tracking
    static double g_continuousYaw = 0.0;
    static bool g_yawInitialized = false;
    
    // Standing initial state tracking
    bool standingInitialized = false;
    
    static int debugCount = 0;
    
    while (!exitRequest) {
        if (sim->m_ && sim->d_) {
            Eigen::Matrix<double, 12, 1> torques;
            torques.setZero();
            
            double currentTime = sim->d_->time;
            
            // Get current FSM state
            FSMState currentFsmState;
            {
                std::lock_guard<std::mutex> fsmLock(g_fsmMutex);
                currentFsmState = fsmController->getCurrentState();
            }
            
            {
                std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
                
                // Get current yaw
                double yaw, pitch, roll;
                quatToZYXEuler(sim->d_->qpos[3], sim->d_->qpos[4], 
                               sim->d_->qpos[5], sim->d_->qpos[6], yaw, pitch, roll);
                
                // Track continuous yaw
                if (!g_yawInitialized) {
                    g_continuousYaw = yaw;
                    g_yawInitialized = true;
                } else {
                    // Unwrap yaw
                    double diff = yaw - g_continuousYaw;
                    while (diff > M_PI) { yaw -= 2.0 * M_PI; diff = yaw - g_continuousYaw; }
                    while (diff < -M_PI) { yaw += 2.0 * M_PI; diff = yaw - g_continuousYaw; }
                    g_continuousYaw = yaw;
                }
                
                // Detect state transitions for smooth interpolation
                bool enteredPD = (currentFsmState == FSMState::PD_CONTROL || currentFsmState == FSMState::RECOVERY) 
                                 && (lastFsmState != FSMState::PD_CONTROL && lastFsmState != FSMState::RECOVERY);
                bool enteredStanding = (currentFsmState == FSMState::STANDING) && (lastFsmState != FSMState::STANDING);
                bool leftStanding = (lastFsmState == FSMState::STANDING) && (currentFsmState != FSMState::STANDING);
                
                // Reset when leaving STANDING
                if (leftStanding) {
                    standingInitialized = false;
                    g_mpcInitialized.store(false);
                    mpcReady.store(false);
                    std::cout << "[Control] Left STANDING - MPC reset" << std::endl;
                }
                
                // Reset MPC when entering STANDING to reinitialize from current position
                if (enteredStanding) {
                    standingInitialized = false;
                    g_mpcInitialized.store(false);
                    mpcReady.store(false);
                    std::cout << "[Control] Entering STANDING - MPC will reinitialize from current position" << std::endl;
                }
                
                // Start PD interpolation when entering PD_CONTROL
                if (enteredPD) {
                    for (int i = 0; i < 12; i++) {
                        pdStartJoints[i] = sim->d_->qpos[7 + i];
                        pdCurrentTarget[i] = sim->d_->qpos[7 + i];
                    }
                    pdInterpolationStartTime = currentTime;
                    pdInterpolationActive = true;
                    std::cout << "[Control] Starting smooth PD interpolation (1.5s)" << std::endl;
                }
                
                // Initialize STANDING target from current base position
                if (enteredStanding && !standingInitialized) {
                    std::lock_guard<std::mutex> targetLock(g_targetMutex);
                    if (robotInterface) {
                        g_targetState = robotInterface->getInitialState();
                        // Override with current base position and yaw
                        g_targetState(6) = sim->d_->qpos[0];   // x
                        g_targetState(7) = sim->d_->qpos[1];   // y
                        g_targetState(8) = sim->d_->qpos[2];   // z
                        g_targetState(9) = g_continuousYaw;    // yaw
                        g_targetState(10) = 0.0;               // pitch
                        g_targetState(11) = 0.0;               // roll
                        
                        // Also update FSM target
                        {
                            std::lock_guard<std::mutex> fsmLock(g_fsmMutex);
                            StandingTarget target = fsmController->getTarget();
                            target.x = sim->d_->qpos[0];
                            target.y = sim->d_->qpos[1];
                            target.z = 0.575;  // Default standing height
                            target.yaw = g_continuousYaw;
                            target.pitch = 0.0;
                            target.roll = 0.0;
                            fsmController->setTarget(target);
                        }
                        standingInitialized = true;
                        std::cout << "[Control] STANDING initialized from current: x=" << sim->d_->qpos[0] 
                                  << " y=" << sim->d_->qpos[1] << " z=" << sim->d_->qpos[2] 
                                  << " yaw=" << g_continuousYaw << std::endl;
                    }
                }
                
                lastFsmState = currentFsmState;
                
                // Control based on FSM state
                switch (currentFsmState) {
                    case FSMState::PASSIVE:
                    case FSMState::EMERGENCY:
                        // No torques - robot free-falling
                        torques.setZero();
                        pdInterpolationActive = false;  // Reset PD interpolation
                        if (debugCount++ % 1000 == 0) {
                            std::cout << "[Control] PASSIVE - No torques" << std::endl;
                        }
                        break;
                        
                    case FSMState::PD_CONTROL:
                    case FSMState::RECOVERY: {
                        // Smooth PD interpolation
                        if (pdInterpolationActive) {
                            double elapsed = currentTime - pdInterpolationStartTime;
                            double alpha = std::min(1.0, elapsed / pdInterpolationDuration);
                            // Smooth interpolation using cosine easing
                            double smoothAlpha = 0.5 * (1.0 - std::cos(alpha * M_PI));
                            
                            for (int i = 0; i < 12; i++) {
                                pdCurrentTarget[i] = pdStartJoints[i] + smoothAlpha * (defaultJoints[i] - pdStartJoints[i]);
                            }
                            
                            if (alpha >= 1.0) {
                                pdInterpolationActive = false;
                                std::cout << "[Control] PD interpolation complete" << std::endl;
                            }
                        } else {
                            // After interpolation complete, use default joints directly
                            for (int i = 0; i < 12; i++) {
                                pdCurrentTarget[i] = defaultJoints[i];
                            }
                        }
                        
                        for (int i = 0; i < 12; i++) {
                            double pos = sim->d_->qpos[7 + i];
                            double vel = sim->d_->qvel[6 + i];
                            torques(i) = KP * (pdCurrentTarget[i] - pos) - KD * vel;
                        }
                        if (debugCount++ % 1000 == 0) {
                            std::cout << "[Control] PD_CONTROL" << (pdInterpolationActive ? " (interpolating)" : "") << std::endl;
                        }
                        break;
                    }
                        
                    case FSMState::STANDING:
                        // MPC + WBC control
                        if (mpcReady.load()) {
                            // Get current measured RBD state
                            vector_t measuredRbdState = mujocoToRbdState(sim->d_->qpos, sim->d_->qvel,
                                                                         robotInterface->getCentroidalModelInfo());
                            
                            // Get MPC optimal state and input
                            vector_t desiredState, desiredInput;
                            bool useMpc = false;
                            
                            {
                                std::lock_guard<std::mutex> mpcLock(g_mpcMutex);
                                if (g_optimalState.size() >= 24 && g_optimalInput.size() >= 24) {
                                    desiredState = g_optimalState;
                                    desiredInput = g_optimalInput;
                                    useMpc = true;
                                    
                                    // Extract joint targets for PD fallback
                                    extractJointTargets(g_optimalState, targetJoints);
                                }
                            }
                            
                            if (useMpc && USE_WBC_TORQUE && rbdConversions) {
                                // WBC: τ = RNEA(q, v, a, f_ext) using contact forces from MPC
                                try {
                                    vector_t wbcTorques = computeWbcTorques(desiredState, desiredInput, measuredRbdState);
                                    extractJointTorques(wbcTorques, torques);
                                    
                                    if (debugCount++ % 1000 == 0) {
                                        std::cout << "[Control] STANDING (WBC) - Contact Fz: ["
                                                  << desiredInput(2) << ", " << desiredInput(5) << ", "
                                                  << desiredInput(8) << ", " << desiredInput(11) << "]" << std::endl;
                                    }
                                } catch (const std::exception& e) {
                                    std::cerr << "[WBC] Error: " << e.what() << std::endl;
                                    // Fallback to PD
                                    for (int i = 0; i < 12; i++) {
                                        double pos = sim->d_->qpos[7 + i];
                                        double vel = sim->d_->qvel[6 + i];
                                        torques(i) = KP * (targetJoints[i] - pos) - KD * vel;
                                    }
                                }
                            } else if (useMpc) {
                                // MPC without WBC - use PD tracking
                                for (int i = 0; i < 12; i++) {
                                    double pos = sim->d_->qpos[7 + i];
                                    double vel = sim->d_->qvel[6 + i];
                                    torques(i) = KP * (targetJoints[i] - pos) - KD * vel;
                                }
                                if (debugCount++ % 1000 == 0) {
                                    std::cout << "[Control] STANDING (PD tracking MPC)" << std::endl;
                                }
                            } else {
                                // MPC not ready yet - fallback to default PD
                                for (int i = 0; i < 12; i++) {
                                    double pos = sim->d_->qpos[7 + i];
                                    double vel = sim->d_->qvel[6 + i];
                                    torques(i) = KP * (defaultJoints[i] - pos) - KD * vel;
                                }
                                if (debugCount++ % 1000 == 0) {
                                    std::cout << "[Control] STANDING - Waiting for MPC..." << std::endl;
                                }
                            }
                        } else {
                            // MPC not ready - PD to default stance
                            for (int i = 0; i < 12; i++) {
                                double pos = sim->d_->qpos[7 + i];
                                double vel = sim->d_->qvel[6 + i];
                                torques(i) = KP * (defaultJoints[i] - pos) - KD * vel;
                            }
                            if (debugCount++ % 1000 == 0) {
                                std::cout << "[Control] STANDING - MPC initializing..." << std::endl;
                            }
                        }
                        break;
                }
                
                // Clamp torques
                for (int i = 0; i < 12; i++) {
                    torques(i) = std::clamp(torques(i), -MAX_TORQUE, MAX_TORQUE);
                }
            }
            
            {
                std::lock_guard<std::mutex> lock(g_torqueMutex);
                g_torques = torques;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(1000));  // 1kHz
    }
    
    std::cout << "[Control Thread] Stopped" << std::endl;
}

// =============================================================================
// Physics Thread
// =============================================================================
void physicsThread(mujoco::Simulate* sim, const std::string& modelPath) {
    std::cout << "[Physics Thread] Started" << std::endl;
    std::cout << "[Physics Thread] Loading: " << modelPath << std::endl;
    
    char error[1000] = "Could not load model";
    mjModel* m = mj_loadXML(modelPath.c_str(), nullptr, error, 1000);
    if (!m) {
        std::cerr << "[Physics Thread] Load error: " << error << std::endl;
        exitRequest = true;
        return;
    }
    
    mjData* d = mj_makeData(m);
    if (!d) {
        std::cerr << "[Physics Thread] Could not create mjData" << std::endl;
        mj_deleteModel(m);
        exitRequest = true;
        return;
    }
    
    // Set initial pose
    d->qpos[0] = 0.0;   // x
    d->qpos[1] = 0.0;   // y
    d->qpos[2] = 0.57;  // z (standing height)
    d->qpos[3] = 1.0;   // qw
    d->qpos[4] = 0.0;   // qx
    d->qpos[5] = 0.0;   // qy
    d->qpos[6] = 0.0;   // qz
    
    // Joint positions (LF, RF, LH, RH)
    double initJoints[12] = {
        -0.25,  0.60, -0.85,   // LF
         0.25,  0.60, -0.85,   // RF
        -0.25, -0.60,  0.85,   // LH
         0.25, -0.60,  0.85    // RH
    };
    for (int i = 0; i < 12; i++) {
        d->qpos[7 + i] = initJoints[i];
    }
    
    mj_forward(m, d);
    sim->Load(m, d, modelPath.c_str());
    std::cout << "[Physics Thread] Model loaded!" << std::endl;
    
    // Initialize MPC with current state
    if (robotInterface && mpcMrt) {
        std::cout << "[Physics Thread] MPC ready - waiting for STANDING mode (press X)" << std::endl;
    }
    
    using Clock = std::chrono::steady_clock;
    auto syncCPU = Clock::now();
    mjtNum syncSim = 0;
    
    // Note: using g_mpcInitialized (global atomic) instead of local
    
    while (!exitRequest && !sim->exitrequest.load()) {
        if (sim->run && sim->busywait) {
            std::this_thread::yield();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        {
            std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
            
            if (sim->m_ && sim->d_) {
                sim->Sync();
                
                // Apply torques
                {
                    std::lock_guard<std::mutex> torqueLock(g_torqueMutex);
                    for (int i = 0; i < 12; i++) {
                        sim->d_->ctrl[i] = g_torques(i);
                    }
                }
                
                // Update MPC observation (only when FSM is in STANDING mode)
                FSMState currentFsmState;
                {
                    std::lock_guard<std::mutex> fsmLock(g_fsmMutex);
                    currentFsmState = fsmController->getCurrentState();
                }
                
                // Initialize MPC when entering STANDING mode
                if (!g_mpcInitialized.load() && currentFsmState == FSMState::STANDING && sim->run) {
                    vector_t initState = mujocoToOcs2State(sim->d_->qpos, sim->d_->qvel, 
                                                           robotInterface->getCentroidalModelInfo());
                    
                    // Compute gravity-compensating input
                    contact_flag_t contactFlags = {true, true, true, true};
                    vector_t gravCompInput = weightCompensatingInput(
                        robotInterface->getCentroidalModelInfo(), contactFlags);
                    
                    // Get target state (already set from current position in control thread)
                    vector_t initTargetState;
                    {
                        std::lock_guard<std::mutex> targetLock(g_targetMutex);
                        initTargetState = g_targetState;
                    }
                    
                    // Fallback: if g_targetState not initialized, use current position
                    if (initTargetState.size() == 0) {
                        initTargetState = robotInterface->getInitialState();
                        initTargetState(6) = sim->d_->qpos[0];  // x
                        initTargetState(7) = sim->d_->qpos[1];  // y
                        initTargetState(8) = 0.575;             // target height
                        // Extract yaw from quaternion
                        Eigen::Quaterniond quat(sim->d_->qpos[6], sim->d_->qpos[3], 
                                               sim->d_->qpos[4], sim->d_->qpos[5]);
                        Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);
                        initTargetState(9) = euler(0);          // yaw
                    }
                    
                    // Set initial target trajectory with current position
                    TargetTrajectories target;
                    target.timeTrajectory = {sim->d_->time, sim->d_->time + 10.0};
                    target.stateTrajectory = {initTargetState, initTargetState};
                    target.inputTrajectory = {gravCompInput, gravCompInput};
                    mpcSolver->getSolverPtr()->getReferenceManager().setTargetTrajectories(target);
                    
                    SystemObservation initObs;
                    initObs.time = sim->d_->time;
                    initObs.state = initState;
                    initObs.input = gravCompInput;
                    
                    mpcMrt->setCurrentObservation(initObs);
                    mpcReady.store(true);
                    g_mpcInitialized.store(true);
                    
                    std::cout << "\n========================================" << std::endl;
                    std::cout << "[MPC] INITIALIZED at t=" << sim->d_->time << "s" << std::endl;
                    std::cout << "[MPC] Target from current: x=" << initTargetState(6) 
                              << " y=" << initTargetState(7) << " yaw=" << initTargetState(9) << std::endl;
                    std::cout << "[MPC] Robot z=" << sim->d_->qpos[2] << std::endl;
                    std::cout << "========================================\n" << std::endl;
                }
                
                // Update MPC when in STANDING mode
                if (mpcReady.load() && currentFsmState == FSMState::STANDING && sim->run) {
                    double currentTime = sim->d_->time;
                    vector_t state = mujocoToOcs2State(sim->d_->qpos, sim->d_->qvel,
                                                       robotInterface->getCentroidalModelInfo());
                    
                    // Compute gravity-compensating input
                    contact_flag_t contactFlags = {true, true, true, true};
                    vector_t gravCompInput = weightCompensatingInput(
                        robotInterface->getCentroidalModelInfo(), contactFlags);
                    
                    // Get target state from FSM
                    vector_t fsmTargetState;
                    {
                        std::lock_guard<std::mutex> targetLock(g_targetMutex);
                        fsmTargetState = g_targetState;
                    }
                    
                    // Update target trajectories
                    TargetTrajectories target;
                    target.timeTrajectory = {currentTime, currentTime + 10.0};
                    target.stateTrajectory = {fsmTargetState, fsmTargetState};
                    target.inputTrajectory = {gravCompInput, gravCompInput};
                    mpcSolver->getSolverPtr()->getReferenceManager().setTargetTrajectories(target);
                    
                    SystemObservation obs;
                    obs.time = currentTime;
                    obs.state = state;
                    obs.input = gravCompInput;
                    
                    {
                        std::lock_guard<std::mutex> mpcLock(g_mpcMutex);
                        mpcMrt->setCurrentObservation(obs);
                        
                        // Get optimal policy
                        if (mpcMrt->initialPolicyReceived()) {
                            mpcMrt->updatePolicy();
                            size_t mode;
                            mpcMrt->evaluatePolicy(obs.time, state, 
                                                  g_optimalState, g_optimalInput, mode);
                            
                            // Debug output every 500ms
                            static double lastPrintTime = 0;
                            if (currentTime - lastPrintTime > 0.5) {
                                lastPrintTime = currentTime;
                                std::cout << "[MPC] t=" << std::fixed << std::setprecision(2) << currentTime 
                                          << " target_z=" << fsmTargetState(8)
                                          << " actual_z=" << sim->d_->qpos[2]
                                          << std::endl;
                            }
                        }
                    }
                }
                
                // Step physics
                if (sim->run) {
                    auto startCPU = Clock::now();
                    double slowdown = 100.0 / sim->percentRealTime[sim->real_time_index];
                    double elapsedSim = sim->d_->time - syncSim;
                    
                    bool misaligned = std::abs(std::chrono::duration<double>(startCPU - syncCPU).count() / slowdown - elapsedSim) > 0.1;
                    
                    if (elapsedSim < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned || sim->speed_changed) {
                        syncCPU = startCPU;
                        syncSim = sim->d_->time;
                        sim->speed_changed = false;
                        mj_step(sim->m_, sim->d_);
                    } else {
                        double refreshTime = 0.7 / sim->refresh_rate;
                        mjtNum prevSim = sim->d_->time;
                        
                        while (std::chrono::duration<double>((sim->d_->time - syncSim) * slowdown) < Clock::now() - syncCPU &&
                               Clock::now() - startCPU < std::chrono::duration<double>(refreshTime)) {
                            mj_step(sim->m_, sim->d_);
                            if (sim->d_->time < prevSim) break;
                        }
                    }
                    
                    sim->AddToHistory();
                } else {
                    mj_forward(sim->m_, sim->d_);
                    sim->speed_changed = true;
                }
            }
        }
    }
    
    mj_deleteData(d);
    mj_deleteModel(m);
    std::cout << "[Physics Thread] Stopped" << std::endl;
}

// =============================================================================
// Main
// =============================================================================
int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "  ANYmal C Standing Control" << std::endl;
    std::cout << "  OCS2 Centroidal MPC + MuJoCo" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Setup paths
    std::string basePath = getPath();
    g_taskFile = basePath + "/config/mpc/task.info";
    g_urdfFile = basePath + "/robots/anymal_c/urdf/anymal.urdf";
    g_referenceFile = basePath + "/config/command/reference.info";
    
    // Default MuJoCo model - can be overridden by command line
    g_mujocoModel = basePath + "/robots/anymal_c/scene.xml";
    
    if (argc > 1) g_mujocoModel = argv[1];
    
    std::cout << "\n[Paths]" << std::endl;
    std::cout << "  Task: " << g_taskFile << std::endl;
    std::cout << "  URDF: " << g_urdfFile << std::endl;
    std::cout << "  Reference: " << g_referenceFile << std::endl;
    std::cout << "  MuJoCo: " << g_mujocoModel << std::endl;
    
    // Check files exist
    std::ifstream taskCheck(g_taskFile);
    std::ifstream urdfCheck(g_urdfFile);
    std::ifstream refCheck(g_referenceFile);
    std::ifstream modelCheck(g_mujocoModel);
    
    if (!taskCheck.good() || !urdfCheck.good() || !refCheck.good() || !modelCheck.good()) {
        std::cerr << "\n[ERROR] One or more files not found!" << std::endl;
        if (!taskCheck.good()) std::cerr << "  Missing: " << g_taskFile << std::endl;
        if (!urdfCheck.good()) std::cerr << "  Missing: " << g_urdfFile << std::endl;
        if (!refCheck.good()) std::cerr << "  Missing: " << g_referenceFile << std::endl;
        if (!modelCheck.good()) std::cerr << "  Missing: " << g_mujocoModel << std::endl;
        return 1;
    }
    
    // Initialize OCS2 interface 
    std::cout << "\n[1] Initializing OCS2 Interface..." << std::endl;
    try {
        bool useHardFrictionConeConstraint = true;
        robotInterface = std::make_unique<LeggedRobotInterface>(
            g_taskFile, g_urdfFile, g_referenceFile, useHardFrictionConeConstraint);
        
        // Initialize RBD conversions
        rbdConversions = std::make_unique<CentroidalModelRbdConversions>(
            robotInterface->getPinocchioInterface(),
            robotInterface->getCentroidalModelInfo());
        
        std::cout << "[SUCCESS] Interface initialized!" << std::endl;
        std::cout << "  State dim: " << robotInterface->getCentroidalModelInfo().stateDim << std::endl;
        std::cout << "  Input dim: " << robotInterface->getCentroidalModelInfo().inputDim << std::endl;
        std::cout << "  Robot mass: " << robotInterface->getCentroidalModelInfo().robotMass << " kg" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Interface initialization failed: " << e.what() << std::endl;
        return 1;
    }
    
    // Initialize MPC 
    std::cout << "\n[2] Initializing MPC..." << std::endl;
    try {
        mpcSolver = std::make_unique<GaussNewtonDDP_MPC>(
            robotInterface->mpcSettings(),
            robotInterface->ddpSettings(),
            robotInterface->getRollout(),
            robotInterface->getOptimalControlProblem(),
            robotInterface->getInitializer());
        
        mpcSolver->getSolverPtr()->setReferenceManager(robotInterface->getReferenceManagerPtr());
        
        // Set initial target
        TargetTrajectories target;
        target.timeTrajectory = {0.0};
        target.stateTrajectory = {robotInterface->getInitialState()};
        target.inputTrajectory = {vector_t::Zero(robotInterface->getCentroidalModelInfo().inputDim)};
        
        mpcSolver->getSolverPtr()->getReferenceManager().setTargetTrajectories(target);
        
        mpcMrt = std::make_unique<MPC_MRT_Interface>(*mpcSolver);
        mpcMrt->initRollout(&robotInterface->getRollout());
        
        // Initialize optimal state/input
        g_optimalState = robotInterface->getInitialState();
        g_optimalInput = vector_t::Zero(robotInterface->getCentroidalModelInfo().inputDim);
        
        std::cout << "[SUCCESS] MPC initialized!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] MPC initialization failed: " << e.what() << std::endl;
        return 1;
    }
    
    // Initialize MuJoCo GUI
    std::cout << "\n[3] Initializing MuJoCo GUI..." << std::endl;
    
    mjvCamera cam;
    mjvOption opt;
    mjvPerturb pert;
    
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);
    
    cam.azimuth = 135;
    cam.elevation = -20;
    cam.distance = 3.0;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.4;
    
    auto sim = std::make_unique<mujoco::Simulate>(
        std::make_unique<mujoco::GlfwAdapter>(),
        &cam, &opt, &pert, false);
    
    std::cout << "[SUCCESS] GUI initialized!" << std::endl;
    
    // Initialize Xbox Controller
    std::cout << "\n[3.5] Initializing Xbox Controller and FSM..." << std::endl;
    xboxController = std::make_unique<XboxController>();
    keyboardController = std::make_unique<KeyboardController>();
    fsmController = std::make_unique<FSMController>();
    
    // Initialize target state
    g_targetState = robotInterface->getInitialState();
    
    if (xboxController->isConnected()) {
        std::cout << "[SUCCESS] Xbox controller connected!" << std::endl;
    } else {
        std::cout << "[WARNING] Xbox controller not found. Using keyboard/default." << std::endl;
    }
    std::cout << "[SUCCESS] FSM initialized (PASSIVE mode)" << std::endl;
    
    // Start threads
    std::cout << "\n[4] Starting threads..." << std::endl;
    
    std::thread mpcWorker(mpcThread);
    std::thread physicsWorker(physicsThread, sim.get(), g_mujocoModel);
    std::thread controlWorker(controlThread, sim.get());
    std::thread xboxFsmWorker(xboxFsmThread);  // Xbox/FSM thread
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Simulation Running!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  MuJoCo GUI:" << std::endl;
    std::cout << "    Space     - Play/Pause" << std::endl;
    std::cout << "    Backspace - Reset" << std::endl;
    std::cout << "    ESC       - Exit" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "  Xbox Controller Modes:" << std::endl;
    std::cout << "    A - PASSIVE  (no torques)" << std::endl;
    std::cout << "    B - PD       (PD to default stance)" << std::endl;
    std::cout << "    X - STANDING (MPC + WBC control)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "  In STANDING mode:" << std::endl;
    std::cout << "    Left Y  - Move forward/backward" << std::endl;
    std::cout << "    Right Y - Change height (0.35-0.65m)" << std::endl;
    std::cout << "    Right X - Yaw control" << std::endl;
    std::cout << "    LB/RB   - Height presets" << std::endl;
    std::cout << "    Back    - Emergency stop" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Run GUI
    sim->RenderLoop();
    
    // Cleanup
    std::cout << "\n[5] Cleaning up..." << std::endl;
    exitRequest = true;
    sim->exitrequest.store(1);
    
    if (mpcWorker.joinable()) mpcWorker.join();
    if (physicsWorker.joinable()) physicsWorker.join();
    if (controlWorker.joinable()) controlWorker.join();
    if (xboxFsmWorker.joinable()) xboxFsmWorker.join();
    
    std::cout << "\n[DONE] Simulation Complete!" << std::endl;
    
    return 0;
}

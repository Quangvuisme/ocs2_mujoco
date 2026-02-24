/**
 * @file main_anymal_mujoco.cpp
 * @brief ANYmal C Walking Control with MuJoCo + OCS2 Centroidal MPC
 *        With Xbox Controller and FSM mode management
 * 
 * Modes:
 * - PASSIVE: No torques, robot free-falling
 * - PD_CONTROL: Pure PD control to default stance
 * - STANDING: MPC control with 4-leg stance
 * - WALKING: MPC control with gait switching (trot, pace, etc.)
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
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

// Interface
#include <anymal_walking_online/LeggedRobotInterface.h>
#include <anymal_walking_online/package_path.h>
#include <anymal_walking_online/common/utils.h>
#include <anymal_walking_online/common/DebugUtils.h>
#include <anymal_walking_online/reference_manager/SwitchedModelReferenceManager.h>

// Xbox Controller and FSM
#include <anymal_walking_online/XboxController.h>
#include <anymal_walking_online/KeyboardController.h>
#include <anymal_walking_online/FSMController.h>

// MuJoCo
#include <mujoco/mujoco.h>
#include "mujoco_gui/glfw_adapter.h"
#include "mujoco_gui/simulate.h"

using namespace ocs2;
using namespace anymal_walking_online;

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

// Simulation time (for gait change callback)
std::atomic<double> g_simTime{0.0};

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
std::string g_gaitFile;
std::string g_mujocoModel;

// Debug utilities
DebugUtils& g_debug = getDebugUtils();

// =============================================================================
// Helper: Normalize angle to [-pi, pi]
// =============================================================================
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// =============================================================================
// Helper: Unwrap yaw to make it continuous (avoid ±π discontinuity)
// =============================================================================
double unwrapYaw(double newYaw, double prevYaw) {
    double diff = newYaw - prevYaw;
    // If the difference is > π, we crossed from +π to -π
    while (diff > M_PI) {
        newYaw -= 2.0 * M_PI;
        diff = newYaw - prevYaw;
    }
    // If the difference is < -π, we crossed from -π to +π
    while (diff < -M_PI) {
        newYaw += 2.0 * M_PI;
        diff = newYaw - prevYaw;
    }
    return newYaw;
}

// Global continuous yaw tracker
static double g_continuousYaw = 0.0;
static bool g_yawInitialized = false;

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
    
    // IMPORTANT: Use continuous yaw to avoid discontinuity at ±π
    // This allows the robot to rotate freely without yaw jumps
    if (!g_yawInitialized) {
        g_continuousYaw = yaw;
        g_yawInitialized = true;
    } else {
        g_continuousYaw = unwrapYaw(yaw, g_continuousYaw);
    }
    
    rbdState(0) = g_continuousYaw;  // Use continuous yaw
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
    
    // IMPORTANT: RBD state needs WORLD frame angular velocity!
    // OCS2's computeRbdTorqueFromCentroidalModelPD will convert it to Euler derivatives internally
    // MuJoCo qvel[3:6] is in BODY (local) frame, need to convert to WORLD frame
    
    // Get rotation matrix from body to world using quaternion (qw,qx,qy,qz already defined above)
    Eigen::Quaterniond quat(qw, qx, qy, qz);
    Eigen::Matrix3d R_body_to_world = quat.toRotationMatrix();
    
    // Angular velocity in body frame from MuJoCo
    Eigen::Vector3d angularVelocityBody(qvel[3], qvel[4], qvel[5]);
    
    // Convert to world frame: ω_world = R * ω_body
    Eigen::Vector3d angularVelocityWorld = R_body_to_world * angularVelocityBody;
    
    // Store WORLD frame angular velocity (OCS2 will convert to Euler derivatives)
    rbdState(18) = angularVelocityWorld(0);  // wx_world
    rbdState(19) = angularVelocityWorld(1);  // wy_world
    rbdState(20) = angularVelocityWorld(2);  // wz_world
    
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
        pGains(6 + i) = 500.0;   // Position gain
        dGains(6 + i) = 50.0;     // Velocity gain
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
// Create walking target state with velocity commands
// For walking, we extrapolate the target based on velocity commands
// =============================================================================
vector_t createWalkingTargetState(const WalkingTarget& walkTarget, 
                                   const vector_t& currentState,
                                   const vector_t& defaultState,
                                   double horizonTime) {
    vector_t targetState = defaultState;
    
    // Get current position and yaw
    double currentX = currentState(6);
    double currentY = currentState(7);
    double currentYaw = currentState(9);  // Don't normalize here - keep continuous
    
    // Extrapolate target position based on velocity commands
    // For walking, we set the target position ahead by horizonTime
    double cosYaw = std::cos(currentYaw);
    double sinYaw = std::sin(currentYaw);
    
    double targetX = currentX + walkTarget.vx * horizonTime * cosYaw 
                              - walkTarget.vy * horizonTime * sinYaw;
    double targetY = currentY + walkTarget.vx * horizonTime * sinYaw 
                              + walkTarget.vy * horizonTime * cosYaw;
    
    // IMPORTANT: Don't normalize target yaw! Keep it continuous to avoid discontinuity
    // If current yaw is -2.9 and we want to go to -3.1, that's fine (no wrap)
    // MPC will track the smooth trajectory
    double targetYaw = currentYaw + walkTarget.yaw_rate * horizonTime;
    
    // Set target state
    targetState(6) = targetX;
    targetState(7) = targetY;
    targetState(8) = walkTarget.z;   // Walking height
    targetState(9) = targetYaw;      // Continuous yaw (may exceed ±π)
    targetState(10) = 0.0;  // No pitch during walking
    targetState(11) = 0.0;  // No roll during walking
    
    // Debug: Check for abnormal values
    if (g_debug.getFlags().enabled && g_debug.getFlags().printStateValidity) {
        if (!DebugUtils::isValidVector(targetState)) {
            std::cerr << "[createWalkingTargetState] Generated invalid target state!" << std::endl;
            std::cerr << "  currentYaw=" << currentYaw << " targetYaw=" << targetYaw << std::endl;
            std::cerr << "  vx=" << walkTarget.vx << " vy=" << walkTarget.vy << " yaw_rate=" << walkTarget.yaw_rate << std::endl;
        }
    }
    
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
                // A = PASSIVE (0), B = PD_CONTROL (1), X = STANDING (2), Y = WALKING (3)
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
                        case 3:  // Y button - WALKING (MPC + Gait)
                            fsmController->requestStateChange(FSMState::WALKING);
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
                
                // Check RB button for gait switching (only in WALKING mode)
                if (xboxController->wasGaitSwitchPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    if (fsmController->getCurrentState() == FSMState::WALKING) {
                        fsmController->nextGait();
                    } else {
                        // In STANDING mode, RB adjusts height
                        fsmController->adjustHeight(0.05);
                        std::cout << "[Xbox] Height adjusted: " 
                                  << fsmController->getTarget().z << " m" << std::endl;
                    }
                }
                
                // Check height adjustment (LB)
                int heightAdj = xboxController->getHeightAdjustRequest();
                if (heightAdj != 0) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    if (fsmController->getCurrentState() == FSMState::STANDING) {
                        fsmController->adjustHeight(heightAdj * 0.05);  // 5cm per press
                        std::cout << "[Xbox] Height adjusted: " 
                                  << fsmController->getTarget().z << " m" << std::endl;
                    }
                }
                
                // Update FSM with Xbox input (joystick control)
                {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    FSMState currentState = fsmController->getCurrentState();
                    
                    if (currentState == FSMState::STANDING) {
                        fsmController->updateFromXbox(input.lx, input.ly, input.rx, input.ry, dt);
                    } else if (currentState == FSMState::WALKING) {
                        fsmController->updateWalkingFromXbox(input.lx, input.ly, input.rx, input.ry, dt);
                    }
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
                
                // Keyboard mode change (1=PASSIVE, 2=PD_CONTROL, 3=STANDING, 4=WALKING)
                int kbModeReq = keyboardController->getModeChangeRequest();
                if (kbModeReq >= 0) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    switch (kbModeReq) {
                        case 0: fsmController->requestStateChange(FSMState::PASSIVE); break;
                        case 1: fsmController->requestStateChange(FSMState::PD_CONTROL); break;
                        case 2: fsmController->requestStateChange(FSMState::STANDING); break;
                        case 3: fsmController->requestStateChange(FSMState::WALKING); break;
                    }
                }
                
                // Keyboard start (Enter = toggle MPC)
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
                
                // Keyboard gait switch (G key)
                if (keyboardController->wasGaitSwitchPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    if (fsmController->getCurrentState() == FSMState::WALKING) {
                        fsmController->nextGait();
                    } else {
                        fsmController->adjustHeight(0.05);
                    }
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
                    
                    FSMState currentState = fsmController->getCurrentState();
                    if (currentState == FSMState::WALKING) {
                        // For walking, we'll update target state in physics thread
                        // based on current state and velocity commands
                    } else {
                        g_targetState = createTargetStateFromFSM(
                            fsmController->getTarget(),
                            robotInterface->getInitialState());
                    }
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
    
    // Standing initial state tracking
    bool standingInitialized = false;
    
    static int debugCount = 0;
    static double lastDebugPrintTime = 0.0;
    
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
                
                // Detect state transitions for smooth interpolation
                bool enteredPD = (currentFsmState == FSMState::PD_CONTROL || currentFsmState == FSMState::RECOVERY) 
                                 && (lastFsmState != FSMState::PD_CONTROL && lastFsmState != FSMState::RECOVERY);
                bool enteredStanding = (currentFsmState == FSMState::STANDING) && (lastFsmState != FSMState::STANDING);
                bool enteredWalking = (currentFsmState == FSMState::WALKING) && (lastFsmState != FSMState::WALKING);
                bool leftStandingOrWalking = (lastFsmState == FSMState::STANDING || lastFsmState == FSMState::WALKING) 
                                              && (currentFsmState != FSMState::STANDING && currentFsmState != FSMState::WALKING);
                
                // Reset MPC when leaving STANDING/WALKING
                if (leftStandingOrWalking) {
                    standingInitialized = false;
                    g_mpcInitialized.store(false);
                    mpcReady.store(false);
                    std::cout << "[Control] Left STANDING/WALKING - MPC reset" << std::endl;
                }
                
                // Reset MPC when entering STANDING or WALKING to reinitialize from current position
                if (enteredStanding || enteredWalking) {
                    standingInitialized = false;
                    g_mpcInitialized.store(false);
                    mpcReady.store(false);
                    std::cout << "[Control] Entering " << (enteredStanding ? "STANDING" : "WALKING") 
                              << " - MPC will reinitialize from current position" << std::endl;
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
                
                // NOTE: STANDING/WALKING initialization is now done in MPC control block below
                
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
                    case FSMState::WALKING:
                        // MPC + WBC control for both STANDING and WALKING
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
                                    
                                    // Debug: Check state validity
                                    g_debug.checkAbnormalValues(currentTime, desiredState, desiredInput, "MPC_OUTPUT");
                                }
                            }
                            
                            if (useMpc && USE_WBC_TORQUE && rbdConversions) {
                                // WBC: τ = RNEA(q, v, a, f_ext) using contact forces from MPC
                                try {
                                    vector_t wbcTorques = computeWbcTorques(desiredState, desiredInput, measuredRbdState);
                                    
                                    // Debug: Check WBC output validity
                                    g_debug.checkAbnormalValues(currentTime, wbcTorques, desiredInput, "WBC_OUTPUT");
                                    
                                    extractJointTorques(wbcTorques, torques);
                                    
                                    // Debug: Print detailed info periodically
                                    bool shouldPrintDebug = g_debug.getFlags().enabled && 
                                                            (currentTime - lastDebugPrintTime > g_debug.getFlags().printInterval);
                                    
                                    if (shouldPrintDebug) {
                                        lastDebugPrintTime = currentTime;
                                        
                                        // Print base state from MuJoCo (only if enabled)
                                        if (g_debug.getFlags().printBaseState) {
                                            double yaw, pitch, roll;
                                            quatToZYXEuler(sim->d_->qpos[3], sim->d_->qpos[4], 
                                                           sim->d_->qpos[5], sim->d_->qpos[6], 
                                                           yaw, pitch, roll);
                                            Eigen::Vector3d basePos(sim->d_->qpos[0], sim->d_->qpos[1], sim->d_->qpos[2]);
                                            Eigen::Vector3d linVel(sim->d_->qvel[0], sim->d_->qvel[1], sim->d_->qvel[2]);
                                            Eigen::Vector3d angVel(sim->d_->qvel[3], sim->d_->qvel[4], sim->d_->qvel[5]);
                                            
                                            g_debug.printBaseState(currentTime, basePos, yaw, pitch, roll, linVel, angVel);
                                        }
                                        
                                        // Print contact forces (only if enabled)
                                        if (g_debug.getFlags().printContactForce) {
                                            g_debug.printContactForces(currentTime, desiredInput, 
                                                                       robotInterface->getCentroidalModelInfo());
                                        }
                                        
                                        // Print foot positions using Pinocchio FK (only if enabled)
                                        if (g_debug.getFlags().printFootPosition) {
                                            vector_t rbdState = mujocoToRbdState(sim->d_->qpos, sim->d_->qvel, 
                                                                                 robotInterface->getCentroidalModelInfo());
                                            // RBD state format: [yaw, pitch, roll, x, y, z, joints(12), ...]
                                            // OCS2 Pinocchio uses SphericalZYX base: q = [x, y, z, yaw, pitch, roll, joints(12)] = 18D
                                            
                                            // Build OCS2 Pinocchio configuration vector (18D)
                                            vector_t q_ocs2(18);
                                            // Position
                                            q_ocs2(0) = rbdState(3);  // x
                                            q_ocs2(1) = rbdState(4);  // y
                                            q_ocs2(2) = rbdState(5);  // z
                                            // Orientation (ZYX Euler)
                                            q_ocs2(3) = rbdState(0);  // yaw
                                            q_ocs2(4) = rbdState(1);  // pitch
                                            q_ocs2(5) = rbdState(2);  // roll
                                            // Joint positions (OCS2 order: LF, LH, RF, RH)
                                            for (int i = 0; i < 12; ++i) {
                                                q_ocs2(6 + i) = rbdState(6 + i);
                                            }
                                            
                                            std::array<std::string, 4> footNames = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};
                                            auto footPositions = DebugUtils::computeFootPositions(
                                                robotInterface->getPinocchioInterface(), q_ocs2, footNames);
                                            
                                            std::array<std::string, 4> footLabels = {"LF", "LH", "RF", "RH"};
                                            g_debug.printFootPositions(currentTime, footPositions, footLabels);
                                        }
                                        
                                        // Print joint states (only if enabled - checked inside function)
                                        g_debug.printJointStates(currentTime, targetJoints, 
                                                                 &sim->d_->qpos[7], &sim->d_->qvel[6]);
                                        
                                        // Print WBC torques (only if enabled - checked inside function)
                                        g_debug.printWbcTorques(currentTime, wbcTorques);
                                        
                                        // Print MPC target state (only if enabled - checked inside function)
                                        if (g_debug.getFlags().printMpcState) {
                                            vector_t fsmTargetState;
                                            {
                                                std::lock_guard<std::mutex> targetLock(g_targetMutex);
                                                fsmTargetState = g_targetState;
                                            }
                                            g_debug.printMpcState(currentTime, desiredState, fsmTargetState);
                                        }
                                    }
                                    
                                    if (debugCount++ % 1000 == 0) {
                                        std::string modeStr = (currentFsmState == FSMState::WALKING) ? "WALKING" : "STANDING";
                                        std::cout << "[Control] " << modeStr << " (WBC) - Contact Fz: ["
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
                                    std::string modeStr = (currentFsmState == FSMState::WALKING) ? "WALKING" : "STANDING";
                                    std::cout << "[Control] " << modeStr << " (PD tracking MPC)" << std::endl;
                                }
                            } else {
                                // MPC not ready yet - fallback to default PD
                                for (int i = 0; i < 12; i++) {
                                    double pos = sim->d_->qpos[7 + i];
                                    double vel = sim->d_->qvel[6 + i];
                                    torques(i) = KP * (defaultJoints[i] - pos) - KD * vel;
                                }
                                if (debugCount++ % 1000 == 0) {
                                    std::cout << "[Control] Waiting for MPC..." << std::endl;
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
                                std::cout << "[Control] MPC initializing..." << std::endl;
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
                
                // Initialize MPC when entering STANDING or WALKING mode
                if (!g_mpcInitialized.load() && (currentFsmState == FSMState::STANDING || currentFsmState == FSMState::WALKING) && sim->run) {
                    vector_t initState = mujocoToOcs2State(sim->d_->qpos, sim->d_->qvel, 
                                                           robotInterface->getCentroidalModelInfo());
                    
                    // Compute gravity-compensating input
                    contact_flag_t contactFlags = {true, true, true, true};
                    vector_t gravCompInput = weightCompensatingInput(
                        robotInterface->getCentroidalModelInfo(), contactFlags);
                    
                    // Create target state from CURRENT position (x, y, yaw from robot)
                    // Extract current x, y from qpos and yaw from continuous tracker
                    double currentX = sim->d_->qpos[0];
                    double currentY = sim->d_->qpos[1];
                    double currentYaw = g_continuousYaw;
                    
                    vector_t initTargetState = robotInterface->getInitialState();
                    initTargetState(6) = currentX;    // x position
                    initTargetState(7) = currentY;    // y position
                    initTargetState(8) = 0.575;       // target height (default standing)
                    initTargetState(9) = currentYaw;  // yaw orientation
                    
                    // Set initial target trajectory with current position
                    TargetTrajectories target;
                    target.timeTrajectory = {sim->d_->time, sim->d_->time + 10.0};
                    target.stateTrajectory = {initTargetState, initTargetState};
                    target.inputTrajectory = {gravCompInput, gravCompInput};
                    mpcSolver->getSolverPtr()->getReferenceManager().setTargetTrajectories(target);
                    
                    // Also update g_targetState for FSM consistency
                    {
                        std::lock_guard<std::mutex> targetLock(g_targetMutex);
                        g_targetState = initTargetState;
                    }
                    
                    SystemObservation initObs;
                    initObs.time = sim->d_->time;
                    initObs.state = initState;
                    initObs.input = gravCompInput;
                    
                    mpcMrt->setCurrentObservation(initObs);
                    mpcReady.store(true);
                    g_mpcInitialized.store(true);
                    
                    std::string modeStr = (currentFsmState == FSMState::WALKING) ? "WALKING" : "STANDING";
                    std::cout << "\n========================================" << std::endl;
                    std::cout << "[MPC] INITIALIZED (" << modeStr << ") at t=" << sim->d_->time << "s" << std::endl;
                    std::cout << "[MPC] Target from current: x=" << currentX << " y=" << currentY << " yaw=" << currentYaw << std::endl;
                    std::cout << "[MPC] Robot z=" << sim->d_->qpos[2] << std::endl;
                    std::cout << "========================================\n" << std::endl;
                }
                
                // Update MPC when in STANDING or WALKING mode
                if (mpcReady.load() && (currentFsmState == FSMState::STANDING || currentFsmState == FSMState::WALKING) && sim->run) {
                    double currentTime = sim->d_->time;
                    g_simTime.store(currentTime);  // Update global sim time for callbacks
                    vector_t state = mujocoToOcs2State(sim->d_->qpos, sim->d_->qvel,
                                                       robotInterface->getCentroidalModelInfo());
                    
                    // Compute gravity-compensating input
                    contact_flag_t contactFlags = {true, true, true, true};
                    vector_t gravCompInput = weightCompensatingInput(
                        robotInterface->getCentroidalModelInfo(), contactFlags);
                    
                    // Get target state from FSM
                    vector_t fsmTargetState;
                    WalkingTarget walkTarget;
                    
                    {
                        std::lock_guard<std::mutex> fsmLock(g_fsmMutex);
                        
                        if (currentFsmState == FSMState::WALKING) {
                            walkTarget = fsmController->getWalkingTarget();
                            // Create walking target with velocity extrapolation
                            const double horizonTime = 1.0;  // Look ahead 1 second
                            fsmTargetState = createWalkingTargetState(walkTarget, state, 
                                                                      robotInterface->getInitialState(), 
                                                                      horizonTime);
                        } else {
                            std::lock_guard<std::mutex> targetLock(g_targetMutex);
                            fsmTargetState = g_targetState;
                        }
                    }
                    SystemObservation obs;
                    obs.time = currentTime;
                    obs.state = state;
                    obs.input = gravCompInput;
                    // Update target trajectories
                    TargetTrajectories target;
                    target.timeTrajectory = {currentTime, currentTime + 10.0};
                    target.stateTrajectory = {fsmTargetState};
                    target.inputTrajectory = {gravCompInput, gravCompInput};
                    mpcSolver->getSolverPtr()->getReferenceManager().setTargetTrajectories(target);
                    

                    
                    // Debug: check state validity before sending to MPC
                    g_debug.checkAbnormalValues(currentTime, state, gravCompInput, "PHYSICS_OBS");
                    
                    {
                        std::lock_guard<std::mutex> mpcLock(g_mpcMutex);
                        mpcMrt->setCurrentObservation(obs);
                        
                        // Get optimal policy
                        if (mpcMrt->initialPolicyReceived()) {
                            mpcMrt->updatePolicy();
                            size_t mode;
                            mpcMrt->evaluatePolicy(obs.time, state, 
                                                  g_optimalState, g_optimalInput, mode);
                            
                            // Debug: Check optimal state validity
                            g_debug.checkAbnormalValues(currentTime, g_optimalState, g_optimalInput, "POLICY_EVAL");
                            
                            // Debug: Print contact status from gait
                            if (g_debug.getFlags().enabled && g_debug.getFlags().printContactStatus) {
                                auto refManager = std::dynamic_pointer_cast<SwitchedModelReferenceManager>(
                                    robotInterface->getReferenceManagerPtr());
                                if (refManager) {
                                    auto contacts = refManager->getContactFlags(currentTime);
                                    std::array<bool, 4> contactArray = {contacts[0], contacts[1], contacts[2], contacts[3]};
                                    static double lastContactPrint = 0;
                                    if (currentTime - lastContactPrint > g_debug.getFlags().printInterval) {
                                        lastContactPrint = currentTime;
                                        g_debug.printContactStatus(currentTime, contactArray);
                                    }
                                }
                            }
                            
                            // Debug output every 500ms
                            static double lastPrintTime = 0;
                            if (currentTime - lastPrintTime > 0.5) {
                                lastPrintTime = currentTime;
                                
                                if (currentFsmState == FSMState::WALKING) {
                                    // Also print current yaw for debugging
                                    double yaw, pitch, roll;
                                    quatToZYXEuler(sim->d_->qpos[3], sim->d_->qpos[4], 
                                                   sim->d_->qpos[5], sim->d_->qpos[6], 
                                                   yaw, pitch, roll);
                                    
                                    std::cout << "[MPC] t=" << std::fixed << std::setprecision(2) << currentTime 
                                              << " vel=[" << walkTarget.vx << ", " << walkTarget.vy << "]"
                                              << " z=" << sim->d_->qpos[2]
                                              << " yaw=" << std::setprecision(3) << yaw
                                              << " gait=" << gaitTypeToString(walkTarget.gait)
                                              << std::endl;
                                } else {
                                    std::cout << "[MPC] t=" << std::fixed << std::setprecision(2) << currentTime 
                                              << " target_z=" << fsmTargetState(8)
                                              << " actual_z=" << sim->d_->qpos[2]
                                              << std::endl;
                                }
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
    std::cout << "  ANYmal C Walking Control" << std::endl;
    std::cout << "  OCS2 Centroidal MPC + MuJoCo" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // ==========================================================================
    // Debug Configuration - Set debug flags here
    // ==========================================================================
    // To enable full debug output, use: g_debug.getFlags().enableAll();
    // To enable minimal debug (recommended), use: g_debug.getFlags().setMinimal();
    // To disable all debug, use: g_debug.getFlags().disableAll();
    
    // Default: Disable all debug output
    g_debug.getFlags().disableAll();
    g_debug.getFlags().printInterval = 0.5;  // Print every 0.5 seconds
    
    // Enable master switch and specific debug outputs:
    g_debug.getFlags().enabled = true;  // MUST enable master switch!
    // g_debug.getFlags().printFK = true;
    // g_debug.getFlags().printJacobian = true;
    // g_debug.getFlags().printContactForce = true;
    // g_debug.getFlags().printFootPosition = true;
    // g_debug.getFlags().printContactStatus = true;
    // g_debug.getFlags().printJointTarget = true;
    // g_debug.getFlags().printJointCurrent = true;
    g_debug.getFlags().printBaseState = true;
    // g_debug.getFlags().printMpcState = true;
    g_debug.getFlags().printWbcTorques = true;
    // g_debug.getFlags().printStateValidity = true;
    
    // For full verbose debug during development:
    // g_debug.getFlags().enableAll();
    
    // To disable debug output entirely:
    // g_debug.getFlags().disableAll();
    
    std::cout << "\n[Debug Configuration]" << std::endl;
    std::cout << "  Debug enabled: " << (g_debug.getFlags().enabled ? "YES" : "NO") << std::endl;
    if (g_debug.getFlags().enabled) {
        std::cout << "  Print interval: " << g_debug.getFlags().printInterval << "s" << std::endl;
        std::cout << "  FK: " << (g_debug.getFlags().printFK ? "ON" : "OFF") << std::endl;
        std::cout << "  Jacobian: " << (g_debug.getFlags().printJacobian ? "ON" : "OFF") << std::endl;
        std::cout << "  ContactForce: " << (g_debug.getFlags().printContactForce ? "ON" : "OFF") << std::endl;
        std::cout << "  FootPosition: " << (g_debug.getFlags().printFootPosition ? "ON" : "OFF") << std::endl;
        std::cout << "  ContactStatus: " << (g_debug.getFlags().printContactStatus ? "ON" : "OFF") << std::endl;
        std::cout << "  JointTarget: " << (g_debug.getFlags().printJointTarget ? "ON" : "OFF") << std::endl;
        std::cout << "  JointCurrent: " << (g_debug.getFlags().printJointCurrent ? "ON" : "OFF") << std::endl;
        std::cout << "  BaseState: " << (g_debug.getFlags().printBaseState ? "ON" : "OFF") << std::endl;
        std::cout << "  MpcState: " << (g_debug.getFlags().printMpcState ? "ON" : "OFF") << std::endl;
        std::cout << "  WbcTorques: " << (g_debug.getFlags().printWbcTorques ? "ON" : "OFF") << std::endl;
        std::cout << "  StateValidity: " << (g_debug.getFlags().printStateValidity ? "ON" : "OFF") << std::endl;
    }
    // ==========================================================================
    
    // Parse command line arguments first (before setting paths)
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--debug" || arg == "-d") {
            g_debug.getFlags().enableAll();
            std::cout << "[DEBUG] Full debug mode enabled via command line" << std::endl;
        } else if (arg == "--debug-minimal" || arg == "-dm") {
            g_debug.getFlags().setMinimal();
            std::cout << "[DEBUG] Minimal debug mode enabled via command line" << std::endl;
        } else if (arg == "--no-debug" || arg == "-nd") {
            g_debug.getFlags().disableAll();
            std::cout << "[DEBUG] Debug disabled via command line" << std::endl;
        }
    }
    
    // Setup paths
    std::string basePath = getPath();
    g_taskFile = basePath + "/config/mpc/task.info";
    g_urdfFile = basePath + "/robots/anymal_c/urdf/anymal.urdf";
    g_referenceFile = basePath + "/config/command/reference.info";
    g_gaitFile = basePath + "/config/command/gait.info";
    
    // Default MuJoCo model - can be overridden by command line (non-flag argument)
    g_mujocoModel = basePath + "/robots/anymal_c/scene.xml";
    
    // Check for MuJoCo model path in command line (first non-flag argument)
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        // Skip debug flags
        if (arg[0] != '-') {
            g_mujocoModel = arg;
            break;
        }
    }
    
    std::cout << "\n[Paths]" << std::endl;
    std::cout << "  Task: " << g_taskFile << std::endl;
    std::cout << "  URDF: " << g_urdfFile << std::endl;
    std::cout << "  Reference: " << g_referenceFile << std::endl;
    std::cout << "  Gait: " << g_gaitFile << std::endl;
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
    
    // Setup gait change callback to update reference manager
    auto refManagerPtr = std::dynamic_pointer_cast<SwitchedModelReferenceManager>(
        robotInterface->getReferenceManagerPtr());
    if (refManagerPtr) {
        // Load gait definitions
        refManagerPtr->loadGaitDefinitions(g_gaitFile);
        
        // Set callback from FSM to reference manager
        fsmController->setGaitChangeCallback([refManagerPtr](GaitType gait) {
            std::string gaitName = gaitTypeToString(gait);
            double currentTime = g_simTime.load();  // Get current simulation time
            
            std::cout << "[Main] FSM requested gait change to: " << gaitName 
                      << " at sim time " << currentTime << std::endl;
            refManagerPtr->setGait(gaitName, currentTime);
        });
        std::cout << "[SUCCESS] Gait callback connected to reference manager" << std::endl;
    }
    
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
    std::cout << "    X - STANDING (MPC + WBC, 4-leg stance)" << std::endl;
    std::cout << "    Y - WALKING  (MPC + Gait switching)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "  In STANDING mode:" << std::endl;
    std::cout << "    Left Y  - Move forward/backward" << std::endl;
    std::cout << "    Left X  - Move left/right" << std::endl;
    std::cout << "    Right Y - Change height (0.35-0.65m)" << std::endl;
    std::cout << "    Right X - Yaw control" << std::endl;
    std::cout << "    LB/RB   - Height presets" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "  In WALKING mode:" << std::endl;
    std::cout << "    Left Y  - Forward/backward velocity" << std::endl;
    std::cout << "    Left X  - Left/right velocity" << std::endl;
    std::cout << "    Right X - Yaw rate" << std::endl;
    std::cout << "    RB      - Switch gait (trot/pace/walk)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "    Back    - Emergency stop" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "  Debug Options (command line):" << std::endl;
    std::cout << "    --debug, -d       Full debug output" << std::endl;
    std::cout << "    --debug-minimal, -dm  Minimal debug output" << std::endl;
    std::cout << "    --no-debug, -nd   Disable debug output" << std::endl;
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

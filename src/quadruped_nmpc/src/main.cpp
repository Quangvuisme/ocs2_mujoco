/**
 * @file main.cpp
 * @brief Quadruped NMPC Controller via UDP
 *        OCS2 Centroidal MPC + WBC with MuJoCo via UDP bridge
 * 
 * Control Modes:
 * - PASSIVE: No torques, robot free-falling
 * - PD_CONTROL: Pure PD control to default stance
 * - STANDING: MPC control with 4-leg stance
 * - WALKING: MPC control with gait switching
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
#include <csignal>

// OCS2
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

// Local includes
#include "quadruped_nmpc/LeggedRobotInterface.h"
#include "quadruped_nmpc/package_path.h"
#include "quadruped_nmpc/common/utils.h"
#include "quadruped_nmpc/reference_manager/SwitchedModelReferenceManager.h"
#include "quadruped_nmpc/FSMController.h"
#include "quadruped_nmpc/QuadrupedUDP.h"
#include "quadruped_nmpc/XboxController.h"
#include "quadruped_nmpc/KeyboardController.h"

using namespace ocs2;
using namespace quadruped_nmpc;

// =============================================================================
// Global Variables
// =============================================================================
std::atomic<bool> exitRequest{false};
std::atomic<bool> mpcReady{false};

// MPC objects
std::unique_ptr<LeggedRobotInterface> robotInterface;
std::unique_ptr<GaussNewtonDDP_MPC> mpcSolver;
std::unique_ptr<MPC_MRT_Interface> mpcMrt;
std::unique_ptr<CentroidalModelRbdConversions> rbdConversions;

// UDP Communication
std::unique_ptr<QuadrupedUDP> udpComm;

// Xbox Controller and FSM
std::unique_ptr<XboxController> xboxController;
std::unique_ptr<KeyboardController> keyboardController;
std::unique_ptr<FSMController> fsmController;
std::mutex g_fsmMutex;

// Current optimal trajectory
vector_t g_optimalState;
vector_t g_optimalInput;
std::mutex g_mpcMutex;

// Current target state
vector_t g_targetState;
std::mutex g_targetMutex;

// Simulation time
std::atomic<double> g_simTime{0.0};

// Control torques
Eigen::Matrix<double, 12, 1> g_torques;
std::mutex g_torqueMutex;

// Note: KP, KD, MAX_TORQUE are now in g_robotConfig for robot-specific values
constexpr bool USE_WBC_TORQUE = true;

// Config paths
std::string g_taskFile;
std::string g_urdfFile;
std::string g_referenceFile;
std::string g_gaitFile;

// Continuous yaw tracker
static double g_continuousYaw = 0.0;
static bool g_yawInitialized = false;

// Robot selection
std::string g_selectedRobot = "anymal_c";  // Default robot

// =============================================================================
// Robot Selection Helper
// =============================================================================
struct RobotConfig {
    std::string name;
    std::string urdfPath;
    std::string description;
    
    // Robot-specific parameters
    double defaultJoints[12];      // Default joint positions (MuJoCo order: LF, RF, LH, RH)
    double standingHeight;         // Target standing height (m)
    double walkingHeight;          // Target walking height (m)
    double kp;                     // PD proportional gain
    double kd;                     // PD derivative gain
    double maxTorque;              // Maximum joint torque (Nm)
};

// Global robot config (selected at startup)
RobotConfig g_robotConfig;

std::vector<RobotConfig> getAvailableRobots(const std::string& basePath) {
    std::vector<RobotConfig> robots;
    std::string robotsPath = basePath + "/../../robots";  // Go up from package to workspace
    
    // [0] ANYmal C
    robots.push_back({
        "anymal_c",
        robotsPath + "/anymal_c/anymal_c_urdf/urdf/anymal.urdf",
        "ANYmal C quadruped (default)",
        {-0.25,  0.60, -0.85,   // LF: HAA, HFE, KFE
          0.25,  0.60, -0.85,   // RF
         -0.25, -0.60,  0.85,   // LH
          0.25, -0.60,  0.85},  // RH
        0.575,    // standing height
        0.50,     // walking height  
        450.0,    // kp
        15.0,     // kd
        80.0      // max torque
    });
    
    // [1] Unitree Go1
    robots.push_back({
        "go1",
        robotsPath + "/go1/go1_urdf/urdf/go1_anymal_naming.urdf",
        "Unitree Go1 quadruped",
        { 0.01,  0.70, -1.37,   // LF: HAA, HFE, KFE
         -0.01,  0.70, -1.37,   // RF
          0.01,  0.70, -1.37,   // LH
         -0.01,  0.70, -1.37},  // RH
        0.32,     // standing height
        0.28,     // walking height
        350.0,    // kp
        10.0,     // kd
        23.7      // max torque
    });
    
    // [2] Unitree Go2
    robots.push_back({
        "go2",
        robotsPath + "/go2/go2_urdf/urdf/go2_anymal_naming.urdf",
        "Unitree Go2 quadruped",
        { 0.01,  0.70, -1.37,   // LF: HAA, HFE, KFE
         -0.01,  0.70, -1.37,   // RF
          0.01,  0.70, -1.37,   // LH
         -0.01,  0.70, -1.37},  // RH
        0.32,     // standing height
        0.28,     // walking height
        350.0,    // kp
        10.0,     // kd
        23.7      // max torque
    });
    
    return robots;
}

void printRobotMenu(const std::vector<RobotConfig>& robots) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Available Robots:" << std::endl;
    std::cout << "========================================" << std::endl;
    for (size_t i = 0; i < robots.size(); i++) {
        std::cout << "  [" << i << "] " << robots[i].name << std::endl;
        std::cout << "      " << robots[i].description << std::endl;
    }
    std::cout << "========================================" << std::endl;
    std::cout << "Select robot (0-" << robots.size()-1 << ") [default: 0]: ";
}

// =============================================================================
// Signal Handler
// =============================================================================
void signalHandler(int signal) {
    std::cout << "\n[Signal] Received signal " << signal << ", shutting down..." << std::endl;
    exitRequest = true;
}

// =============================================================================
// Helper: Unwrap yaw to make it continuous
// =============================================================================
double unwrapYaw(double newYaw, double prevYaw) {
    double diff = newYaw - prevYaw;
    while (diff > M_PI) {
        newYaw -= 2.0 * M_PI;
        diff = newYaw - prevYaw;
    }
    while (diff < -M_PI) {
        newYaw += 2.0 * M_PI;
        diff = newYaw - prevYaw;
    }
    return newYaw;
}

// =============================================================================
// Convert StatePacket to RBD state (similar to anymal_walking_online/mujocoToRbdState)
// 
// RBD state format:
// [0:3]  base orientation (ZYX Euler: yaw, pitch, roll)
// [3:6]  base position (x, y, z)
// [6:18] joint positions - OCS2 order: LF(0-2), LH(3-5), RF(6-8), RH(9-11)
// [18:21] base angular velocity (world frame: wx, wy, wz)
// [21:24] base linear velocity (world frame: vx, vy, vz)
// [24:36] joint velocities - OCS2 order
// 
// Bridge sends MuJoCo order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
// OCS2 order:                LF(0-2), LH(3-5), RF(6-8), RH(9-11)
// =============================================================================
vector_t statePacketToRbdState(const StatePacket& pkt, const CentroidalModelInfo& info) {
    const int nJoints = 12;
    vector_t rbdState = vector_t::Zero(2 * (6 + nJoints));
    
    // Use continuous yaw (yaw is pkt.yaw from bridge)
    double yaw = pkt.yaw;
    if (!g_yawInitialized) {
        g_continuousYaw = yaw;
        g_yawInitialized = true;
    } else {
        g_continuousYaw = unwrapYaw(yaw, g_continuousYaw);
    }
    
    // Base orientation (RBD state uses ZYX Euler: yaw, pitch, roll at indices 0,1,2)
    rbdState(0) = g_continuousYaw;  // YAW at index 0 (ZYX order)
    rbdState(1) = pkt.pitch;        // PITCH at index 1
    rbdState(2) = pkt.roll;         // ROLL at index 2
    
    // Base position
    rbdState(3) = pkt.x;
    rbdState(4) = pkt.y;
    rbdState(5) = pkt.z;
    
    // Joint positions: Map MuJoCo (LF,RF,LH,RH) -> OCS2 (LF,LH,RF,RH)
    // MuJoCo: pkt.jointPos[0..2]=LF, [3..5]=RF, [6..8]=LH, [9..11]=RH
    // OCS2:   rbdState[6+0..2]=LF, [6+3..5]=LH, [6+6..8]=RF, [6+9..11]=RH
    // LF -> LF (0-2)
    rbdState(6 + 0) = pkt.jointPos[0];
    rbdState(6 + 1) = pkt.jointPos[1];
    rbdState(6 + 2) = pkt.jointPos[2];
    // LH -> index 3-5 in OCS2, but index 6-8 in MuJoCo
    rbdState(6 + 3) = pkt.jointPos[6];
    rbdState(6 + 4) = pkt.jointPos[7];
    rbdState(6 + 5) = pkt.jointPos[8];
    // RF -> index 6-8 in OCS2, but index 3-5 in MuJoCo
    rbdState(6 + 6) = pkt.jointPos[3];
    rbdState(6 + 7) = pkt.jointPos[4];
    rbdState(6 + 8) = pkt.jointPos[5];
    // RH -> index 9-11 in OCS2, same as MuJoCo
    rbdState(6 + 9) = pkt.jointPos[9];
    rbdState(6 + 10) = pkt.jointPos[10];
    rbdState(6 + 11) = pkt.jointPos[11];
    
    // Angular velocity (already world frame from bridge)
    rbdState(18) = pkt.angVelWorld[0];
    rbdState(19) = pkt.angVelWorld[1];
    rbdState(20) = pkt.angVelWorld[2];
    
    // Linear velocity (world frame)
    rbdState(21) = pkt.linVel[0];
    rbdState(22) = pkt.linVel[1];
    rbdState(23) = pkt.linVel[2];
    
    // Joint velocities: same mapping as positions
    // LF
    rbdState(24 + 0) = pkt.jointVel[0];
    rbdState(24 + 1) = pkt.jointVel[1];
    rbdState(24 + 2) = pkt.jointVel[2];
    // LH
    rbdState(24 + 3) = pkt.jointVel[6];
    rbdState(24 + 4) = pkt.jointVel[7];
    rbdState(24 + 5) = pkt.jointVel[8];
    // RF
    rbdState(24 + 6) = pkt.jointVel[3];
    rbdState(24 + 7) = pkt.jointVel[4];
    rbdState(24 + 8) = pkt.jointVel[5];
    // RH
    rbdState(24 + 9) = pkt.jointVel[9];
    rbdState(24 + 10) = pkt.jointVel[10];
    rbdState(24 + 11) = pkt.jointVel[11];
    
    return rbdState;
}

// =============================================================================
// Convert StatePacket to OCS2 Centroidal state
// =============================================================================
vector_t statePacketToOcs2State(const StatePacket& pkt, const CentroidalModelInfo& info) {
    if (!rbdConversions) {
        throw std::runtime_error("rbdConversions not initialized!");
    }
    vector_t rbdState = statePacketToRbdState(pkt, info);
    return rbdConversions->computeCentroidalStateFromRbdModel(rbdState);
}

// =============================================================================
// Extract joint targets from OCS2 state and convert to MuJoCo order
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
// Compute WBC torques
// =============================================================================
vector_t computeWbcTorques(const vector_t& desiredState, const vector_t& desiredInput,
                           const vector_t& measuredRbdState) {
    if (!rbdConversions) {
        return vector_t::Zero(18);
    }
    
    const auto& info = robotInterface->getCentroidalModelInfo();
    vector_t jointAccelerations = vector_t::Zero(info.actuatedDofNum);
    
    vector_t pGains = vector_t::Zero(info.generalizedCoordinatesNum);
    vector_t dGains = vector_t::Zero(info.generalizedCoordinatesNum);
    
    for (int i = 0; i < 12; i++) {
        pGains(6 + i) = 350.0;
        dGains(6 + i) = 35.0;
    }
    
    return rbdConversions->computeRbdTorqueFromCentroidalModelPD(
        desiredState, desiredInput, jointAccelerations,
        measuredRbdState, pGains, dGains);
}

// =============================================================================
// Extract joint torques from WBC output and convert to MuJoCo order
// (similar to anymal_walking_online/extractJointTorques)
// 
// WBC output: [base_wrench(6), joint_torques_OCS2_order(12)]
// OCS2 order:   LF(0-2), LH(3-5), RF(6-8), RH(9-11)
// MuJoCo order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
// =============================================================================
void extractJointTorques(const vector_t& wbcTorques, Eigen::Matrix<double, 12, 1>& mujocoTorques) {
    // Skip base wrench (first 6 elements), get joint torques
    // Map OCS2 order -> MuJoCo order
    
    // LF: OCS2[6-8] -> MuJoCo[0-2] (same)
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
    
    // RH: OCS2[15-17] -> MuJoCo[9-11] (same)
    mujocoTorques(9) = wbcTorques(6 + 9);
    mujocoTorques(10) = wbcTorques(6 + 10);
    mujocoTorques(11) = wbcTorques(6 + 11);
}

// =============================================================================
// Create target state from FSM
// =============================================================================
vector_t createTargetStateFromFSM(const StandingTarget& target, const vector_t& defaultState) {
    vector_t targetState = defaultState;
    targetState(6) = target.x;
    targetState(7) = target.y;
    targetState(8) = target.z;
    targetState(9) = target.yaw;
    targetState(10) = target.pitch;
    targetState(11) = target.roll;
    return targetState;
}

// =============================================================================
// Create walking target state
// =============================================================================
vector_t createWalkingTargetState(const WalkingTarget& walkTarget,
                                   const vector_t& currentState,
                                   const vector_t& defaultState,
                                   double horizonTime) {
    vector_t targetState = defaultState;
    
    double currentX = currentState(6);
    double currentY = currentState(7);
    double currentYaw = currentState(9);
    
    double cosYaw = std::cos(currentYaw);
    double sinYaw = std::sin(currentYaw);
    
    double targetX = currentX + walkTarget.vx * horizonTime * cosYaw 
                              - walkTarget.vy * horizonTime * sinYaw;
    double targetY = currentY + walkTarget.vx * horizonTime * sinYaw 
                              + walkTarget.vy * horizonTime * cosYaw;
    double targetYaw = currentYaw + walkTarget.yaw_rate * horizonTime;
    
    targetState(6) = targetX;
    targetState(7) = targetY;
    targetState(8) = walkTarget.z;
    targetState(9) = targetYaw;
    targetState(10) = 0.0;
    targetState(11) = 0.0;
    
    return targetState;
}

// =============================================================================
// Xbox/FSM Thread
// =============================================================================
void xboxFsmThread() {
    std::cout << "[Xbox/FSM] Starting..." << std::endl;
    
    auto lastTime = std::chrono::steady_clock::now();
    double dt = 0.01;
    
    if (robotInterface) {
        std::lock_guard<std::mutex> lock(g_targetMutex);
        g_targetState = robotInterface->getInitialState();
    }
    
    while (!exitRequest) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastTime).count();
        
        if (elapsed >= dt) {
            lastTime = now;
            
            XboxInput input;
            if (xboxController && xboxController->isConnected()) {
                input = xboxController->getInput();
                
                // Mode change from Xbox (A,B,X,Y)
                int modeReq = xboxController->getTargetChangeRequest();
                if (modeReq >= 0) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    switch (modeReq) {
                        case 0: fsmController->requestStateChange(FSMState::PASSIVE); break;
                        case 1: fsmController->requestStateChange(FSMState::PD_CONTROL); break;
                        case 2: fsmController->requestStateChange(FSMState::STANDING); break;
                        case 3: fsmController->requestStateChange(FSMState::WALKING); break;
                    }
                }
                
                // Start button - toggle MPC
                if (xboxController->wasStartPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->toggleMpc();
                    std::cout << "[Xbox] MPC: " << (fsmController->isMpcEnabled() ? "ON" : "OFF") << std::endl;
                }
            }
            
            // Also check keyboard input
            KeyboardInput kbInput;
            if (keyboardController && keyboardController->isConnected()) {
                kbInput = keyboardController->getInput();
                
                // Merge keyboard input with Xbox (keyboard overrides if active)
                if (std::abs(kbInput.lx) > 0.01f || std::abs(kbInput.ly) > 0.01f ||
                    std::abs(kbInput.rx) > 0.01f || std::abs(kbInput.ry) > 0.01f) {
                    input.lx = kbInput.lx;
                    input.ly = kbInput.ly;
                    input.rx = kbInput.rx;
                    input.ry = kbInput.ry;
                }
                
                // Keyboard mode change (1-4 keys)
                int kbModeReq = keyboardController->getTargetChangeRequest();
                if (kbModeReq >= 0) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    switch (kbModeReq) {
                        case 0: fsmController->requestStateChange(FSMState::PASSIVE); break;
                        case 1: fsmController->requestStateChange(FSMState::PD_CONTROL); break;
                        case 2: fsmController->requestStateChange(FSMState::STANDING); break;
                        case 3: fsmController->requestStateChange(FSMState::WALKING); break;
                    }
                }
                
                // Keyboard start button (Enter key)
                if (keyboardController->wasStartPressed()) {
                    std::lock_guard<std::mutex> lock(g_fsmMutex);
                    fsmController->toggleMpc();
                    std::cout << "[Keyboard] MPC: " << (fsmController->isMpcEnabled() ? "ON" : "OFF") << std::endl;
                }
            }
            
            {
                // Update FSM with joystick input
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
            
            // Update FSM state machine
            {
                std::lock_guard<std::mutex> lock(g_fsmMutex);
                fsmController->update(dt);
                
                if (robotInterface && fsmController->getCurrentState() != FSMState::WALKING) {
                    std::lock_guard<std::mutex> targetLock(g_targetMutex);
                    g_targetState = createTargetStateFromFSM(
                        fsmController->getTarget(),
                        robotInterface->getInitialState());
                }
            }
            
            // Print status
            static int printCount = 0;
            if (++printCount % 100 == 0) {
                std::lock_guard<std::mutex> lock(g_fsmMutex);
                std::cout << "[FSM] " << fsmController->getStatusString() << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    std::cout << "[Xbox/FSM] Stopped" << std::endl;
}

// =============================================================================
// MPC Thread
// =============================================================================
void mpcThread() {
    std::cout << "[MPC] Starting..." << std::endl;
    
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
                std::cerr << "[MPC] Error: " << e.what() << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    std::cout << "[MPC] Stopped" << std::endl;
}

// =============================================================================
// Control Thread (UDP communication + WBC)
// =============================================================================
void controlThread() {
    std::cout << "[Control] Starting..." << std::endl;
    std::cout << "[Control] Robot: " << g_robotConfig.name << std::endl;
    std::cout << "[Control] Standing height: " << g_robotConfig.standingHeight << "m" << std::endl;
    std::cout << "[Control] KP=" << g_robotConfig.kp << " KD=" << g_robotConfig.kd 
              << " MaxTorque=" << g_robotConfig.maxTorque << std::endl;
    
    g_torques.setZero();
    
    // Use robot-specific joint positions from config
    double defaultJoints[12];
    std::copy(g_robotConfig.defaultJoints, g_robotConfig.defaultJoints + 12, defaultJoints);
    
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
    
    bool mpcInitialized = false;
    int debugCount = 0;
    
    while (!exitRequest) {
        // Receive state from MuJoCo
        StatePacket statePkt;
        if (!udpComm->receiveState(statePkt)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        
        double currentTime = statePkt.timestamp;
        g_simTime.store(currentTime);
        
        // Get current FSM state
        FSMState currentFsmState;
        {
            std::lock_guard<std::mutex> fsmLock(g_fsmMutex);
            currentFsmState = fsmController->getCurrentState();
        }
        
        Eigen::Matrix<double, 12, 1> torques;
        torques.setZero();
        
        // Detect state transitions for smooth interpolation
        bool enteredPD = (currentFsmState == FSMState::PD_CONTROL || currentFsmState == FSMState::RECOVERY) 
                         && (lastFsmState != FSMState::PD_CONTROL && lastFsmState != FSMState::RECOVERY);
        bool enteredStanding = (currentFsmState == FSMState::STANDING) && (lastFsmState != FSMState::STANDING);
        bool enteredWalking = (currentFsmState == FSMState::WALKING) && (lastFsmState != FSMState::WALKING);
        bool leftStandingOrWalking = (lastFsmState == FSMState::STANDING || lastFsmState == FSMState::WALKING) 
                                      && (currentFsmState != FSMState::STANDING && currentFsmState != FSMState::WALKING);
        
        // Reset MPC when leaving STANDING/WALKING
        if (leftStandingOrWalking) {
            mpcInitialized = false;
            standingInitialized = false;
            mpcReady.store(false);
            std::cout << "[Control] Left STANDING/WALKING - MPC reset" << std::endl;
        }
        
        // Reset MPC when entering STANDING or WALKING to reinitialize from current position
        if (enteredStanding || enteredWalking) {
            mpcInitialized = false;
            standingInitialized = false;
            mpcReady.store(false);
            std::cout << "[Control] Entering " << (enteredStanding ? "STANDING" : "WALKING") 
                      << " - MPC will reinitialize from current position" << std::endl;
        }
        
        // Start PD interpolation when entering PD_CONTROL
        if (enteredPD) {
            for (int i = 0; i < 12; i++) {
                pdStartJoints[i] = statePkt.jointPos[i];
                pdCurrentTarget[i] = statePkt.jointPos[i];
            }
            pdInterpolationStartTime = currentTime;
            pdInterpolationActive = true;
            std::cout << "[Control] Starting smooth PD interpolation (1.5s)" << std::endl;
        }
        
        // NOTE: STANDING/WALKING initialization is now done in MPC init block below
        
        lastFsmState = currentFsmState;
        
        // Control based on FSM state
        switch (currentFsmState) {
            case FSMState::PASSIVE:
            case FSMState::EMERGENCY:
                torques.setZero();
                pdInterpolationActive = false;  // Reset PD interpolation
                if (debugCount++ % 1000 == 0) {
                    std::cout << "[Control] PASSIVE" << std::endl;
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
                    torques(i) = g_robotConfig.kp * (pdCurrentTarget[i] - statePkt.jointPos[i]) 
                               - g_robotConfig.kd * statePkt.jointVel[i];
                }
                if (debugCount++ % 1000 == 0) {
                    std::cout << "[Control] PD_CONTROL" << (pdInterpolationActive ? " (interpolating)" : "") << std::endl;
                }
                break;
            }
                
            case FSMState::STANDING:
            case FSMState::WALKING:
                // Initialize MPC from CURRENT position
                if (!mpcInitialized) {
                    try {
                        vector_t initState = statePacketToOcs2State(statePkt, 
                            robotInterface->getCentroidalModelInfo());
                        
                        contact_flag_t contactFlags = {true, true, true, true};
                        vector_t gravCompInput = weightCompensatingInput(
                            robotInterface->getCentroidalModelInfo(), contactFlags);
                        
                        // Create initial target from CURRENT position, not default!
                        vector_t initTargetState = robotInterface->getInitialState();
                        // Override base pose with current measured values
                        initTargetState(6) = statePkt.x;           // x
                        initTargetState(7) = statePkt.y;           // y
                        initTargetState(8) = g_robotConfig.standingHeight;  // z - robot-specific height
                        initTargetState(9) = g_continuousYaw;      // yaw - use continuous
                        initTargetState(10) = 0.0;                 // pitch
                        initTargetState(11) = 0.0;                 // roll
                        
                        // Update FSM target as well
                        {
                            std::lock_guard<std::mutex> fsmLock(g_fsmMutex);
                            if (currentFsmState == FSMState::STANDING) {
                                StandingTarget target = fsmController->getTarget();
                                target.x = statePkt.x;
                                target.y = statePkt.y;
                                target.z = g_robotConfig.standingHeight;
                                target.yaw = g_continuousYaw;
                                target.pitch = 0.0;
                                target.roll = 0.0;
                                fsmController->setTarget(target);
                            } else {
                                // WALKING mode
                                WalkingTarget walkTarget = fsmController->getWalkingTarget();
                                walkTarget.vx = 0.0;
                                walkTarget.vy = 0.0;
                                walkTarget.yaw_rate = 0.0;
                                walkTarget.z = g_robotConfig.walkingHeight;
                                fsmController->setWalkingTarget(walkTarget);
                            }
                        }
                        
                        // Store in g_targetState
                        {
                            std::lock_guard<std::mutex> targetLock(g_targetMutex);
                            g_targetState = initTargetState;
                        }
                        standingInitialized = true;
                        
                        TargetTrajectories target;
                        target.timeTrajectory = {currentTime, currentTime + 10.0};
                        target.stateTrajectory = {initTargetState, initTargetState};
                        target.inputTrajectory = {gravCompInput, gravCompInput};
                        mpcSolver->getSolverPtr()->getReferenceManager().setTargetTrajectories(target);
                        
                        SystemObservation initObs;
                        initObs.time = currentTime;
                        initObs.state = initState;
                        initObs.input = gravCompInput;
                        
                        mpcMrt->setCurrentObservation(initObs);
                        mpcReady.store(true);
                        mpcInitialized = true;
                        
                        std::cout << "\n========================================" << std::endl;
                        std::cout << "[MPC] INITIALIZED at t=" << currentTime << "s" << std::endl;
                        std::cout << "  Current pos: x=" << statePkt.x << " y=" << statePkt.y 
                                  << " z=" << statePkt.z << " yaw=" << g_continuousYaw << std::endl;
                        std::cout << "  Target: x=" << initTargetState(6) << " y=" << initTargetState(7) 
                                  << " z=" << initTargetState(8) << " yaw=" << initTargetState(9) << std::endl;
                        std::cout << "========================================\n" << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << "[MPC Init] Error: " << e.what() << std::endl;
                    }
                }
                
                // MPC control
                if (mpcReady.load()) {
                    try {
                        vector_t measuredRbdState = statePacketToRbdState(statePkt,
                            robotInterface->getCentroidalModelInfo());
                        vector_t state = statePacketToOcs2State(statePkt,
                            robotInterface->getCentroidalModelInfo());
                        
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
                                const double horizonTime = 1.0;
                                fsmTargetState = createWalkingTargetState(walkTarget, state,
                                    robotInterface->getInitialState(), horizonTime);
                            } else {
                                std::lock_guard<std::mutex> targetLock(g_targetMutex);
                                fsmTargetState = g_targetState;
                            }
                        }
                        
                        // Update observation
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
                        
                        {
                            std::lock_guard<std::mutex> mpcLock(g_mpcMutex);
                            mpcMrt->setCurrentObservation(obs);
                            
                            if (mpcMrt->initialPolicyReceived()) {
                                mpcMrt->updatePolicy();
                                size_t mode;
                                mpcMrt->evaluatePolicy(obs.time, state,
                                    g_optimalState, g_optimalInput, mode);
                                
                                extractJointTargets(g_optimalState, targetJoints);
                            }
                        }
                        
                        // Compute WBC torques
                        if (USE_WBC_TORQUE && rbdConversions && g_optimalState.size() >= 24) {
                            vector_t wbcTorques = computeWbcTorques(
                                g_optimalState, g_optimalInput, measuredRbdState);
                            extractJointTorques(wbcTorques, torques);
                        } else {
                            // Fallback to PD
                            for (int i = 0; i < 12; i++) {
                                torques(i) = g_robotConfig.kp * (targetJoints[i] - statePkt.jointPos[i])
                                           - g_robotConfig.kd * statePkt.jointVel[i];
                            }
                        }
                        
                        if (debugCount++ % 500 == 0) {
                            std::string modeStr = (currentFsmState == FSMState::WALKING) ? "WALKING" : "STANDING";
                            std::cout << "[Control] " << modeStr << " t=" << std::fixed << std::setprecision(2) 
                                      << currentTime << " z=" << statePkt.z << std::endl;
                        }
                        
                    } catch (const std::exception& e) {
                        std::cerr << "[Control] Error: " << e.what() << std::endl;
                        for (int i = 0; i < 12; i++) {
                            torques(i) = g_robotConfig.kp * (defaultJoints[i] - statePkt.jointPos[i])
                                       - g_robotConfig.kd * statePkt.jointVel[i];
                        }
                    }
                } else {
                    for (int i = 0; i < 12; i++) {
                        torques(i) = g_robotConfig.kp * (defaultJoints[i] - statePkt.jointPos[i])
                                   - g_robotConfig.kd * statePkt.jointVel[i];
                    }
                    if (debugCount++ % 1000 == 0) {
                        std::cout << "[Control] Waiting for MPC..." << std::endl;
                    }
                }
                break;
        }
        
        // Clamp torques with robot-specific limits
        for (int i = 0; i < 12; i++) {
            torques(i) = std::clamp(torques(i), -g_robotConfig.maxTorque, g_robotConfig.maxTorque);
        }
        
        // Send torques to MuJoCo (OCS2 order, QuadrupedUDP handles mapping to bridge order)
        CommandPacket cmdPkt;
        cmdPkt.timestamp = currentTime;
        for (int i = 0; i < 12; i++) {
            cmdPkt.jointTorques[i] = torques(i);  // Now double, not float
        }
        udpComm->sendCommand(cmdPkt);
        
        {
            std::lock_guard<std::mutex> lock(g_torqueMutex);
            g_torques = torques;
        }
    }
    
    std::cout << "[Control] Stopped" << std::endl;
}

// =============================================================================
// Main
// =============================================================================
int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "  Quadruped NMPC via UDP" << std::endl;
    std::cout << "  OCS2 Centroidal MPC + WBC" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Signal handler
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // Get available robots
    std::string basePath = quadruped_nmpc::getPath();
    std::vector<RobotConfig> availableRobots = getAvailableRobots(basePath);
    
    // Robot selection
    int robotChoice = 0;
    if (argc > 1) {
        // Command line argument: robot index
        robotChoice = std::atoi(argv[1]);
        if (robotChoice < 0 || robotChoice >= (int)availableRobots.size()) {
            std::cerr << "[ERROR] Invalid robot index: " << robotChoice << std::endl;
            std::cerr << "Valid range: 0-" << availableRobots.size()-1 << std::endl;
            return 1;
        }
    } else {
        // Interactive menu
        printRobotMenu(availableRobots);
        std::string input;
        std::getline(std::cin, input);
        if (!input.empty()) {
            robotChoice = std::atoi(input.c_str());
            if (robotChoice < 0 || robotChoice >= (int)availableRobots.size()) {
                std::cerr << "[ERROR] Invalid choice, using default (0)" << std::endl;
                robotChoice = 0;
            }
        }
    }
    
    RobotConfig selectedRobot = availableRobots[robotChoice];
    g_selectedRobot = selectedRobot.name;
    g_robotConfig = selectedRobot;  // Store full config globally
    
    std::cout << "\n[Selected Robot]" << std::endl;
    std::cout << "  Name: " << selectedRobot.name << std::endl;
    std::cout << "  Description: " << selectedRobot.description << std::endl;
    std::cout << "  Standing height: " << g_robotConfig.standingHeight << "m" << std::endl;
    std::cout << "  Walking height: " << g_robotConfig.walkingHeight << "m" << std::endl;
    std::cout << "  PD gains: Kp=" << g_robotConfig.kp << ", Kd=" << g_robotConfig.kd << std::endl;
    std::cout << "  Max torque: " << g_robotConfig.maxTorque << "Nm" << std::endl;
    
    // Setup paths - use robot-specific config if available
    std::string robotConfigPath = basePath + "/config/robots/" + selectedRobot.name;
    std::string robotTaskFile = robotConfigPath + "/task.info";
    std::string robotRefFile = robotConfigPath + "/reference.info";
    
    std::ifstream robotTaskCheck(robotTaskFile);
    std::ifstream robotRefCheck(robotRefFile);
    
    if (robotTaskCheck.good()) {
        g_taskFile = robotTaskFile;
    } else {
        g_taskFile = basePath + "/config/mpc/task.info";  // Fallback to default
    }
    
    if (robotRefCheck.good()) {
        g_referenceFile = robotRefFile;
    } else {
        g_referenceFile = basePath + "/config/command/reference.info";  // Fallback
    }
    
    g_urdfFile = selectedRobot.urdfPath;
    g_gaitFile = basePath + "/config/command/gait.info";
    
    std::cout << "\n[Paths]" << std::endl;
    std::cout << "  Base: " << basePath << std::endl;
    std::cout << "  Task: " << g_taskFile << std::endl;
    std::cout << "  URDF: " << g_urdfFile << std::endl;
    std::cout << "  Reference: " << g_referenceFile << std::endl;
    std::cout << "  Gait: " << g_gaitFile << std::endl;
    
    // Check files
    std::ifstream taskCheck(g_taskFile);
    std::ifstream urdfCheck(g_urdfFile);
    std::ifstream refCheck(g_referenceFile);
    
    if (!taskCheck.good() || !urdfCheck.good() || !refCheck.good()) {
        std::cerr << "\n[ERROR] Config files not found!" << std::endl;
        if (!taskCheck.good()) std::cerr << "  Missing: " << g_taskFile << std::endl;
        if (!urdfCheck.good()) std::cerr << "  Missing: " << g_urdfFile << std::endl;
        if (!refCheck.good()) std::cerr << "  Missing: " << g_referenceFile << std::endl;
        return 1;
    }
    
    // Initialize OCS2 interface
    std::cout << "\n[1] Initializing OCS2 Interface..." << std::endl;
    try {
        bool useHardFrictionConeConstraint = true;
        robotInterface = std::make_unique<LeggedRobotInterface>(
            g_taskFile, g_urdfFile, g_referenceFile, useHardFrictionConeConstraint);
        
        rbdConversions = std::make_unique<CentroidalModelRbdConversions>(
            robotInterface->getPinocchioInterface(),
            robotInterface->getCentroidalModelInfo());
        
        std::cout << "[SUCCESS] Interface initialized!" << std::endl;
        std::cout << "  State dim: " << robotInterface->getCentroidalModelInfo().stateDim << std::endl;
        std::cout << "  Input dim: " << robotInterface->getCentroidalModelInfo().inputDim << std::endl;
        std::cout << "  Robot mass: " << robotInterface->getCentroidalModelInfo().robotMass << " kg" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Interface init failed: " << e.what() << std::endl;
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
        
        TargetTrajectories target;
        target.timeTrajectory = {0.0};
        target.stateTrajectory = {robotInterface->getInitialState()};
        target.inputTrajectory = {vector_t::Zero(robotInterface->getCentroidalModelInfo().inputDim)};
        mpcSolver->getSolverPtr()->getReferenceManager().setTargetTrajectories(target);
        
        mpcMrt = std::make_unique<MPC_MRT_Interface>(*mpcSolver);
        mpcMrt->initRollout(&robotInterface->getRollout());
        
        g_optimalState = robotInterface->getInitialState();
        g_optimalInput = vector_t::Zero(robotInterface->getCentroidalModelInfo().inputDim);
        
        std::cout << "[SUCCESS] MPC initialized!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] MPC init failed: " << e.what() << std::endl;
        return 1;
    }
    
    // Initialize UDP
    std::cout << "\n[3] Initializing UDP Communication..." << std::endl;
    udpComm = std::make_unique<QuadrupedUDP>();
    if (!udpComm->initialize()) {
        std::cerr << "[ERROR] UDP init failed!" << std::endl;
        return 1;
    }
    std::cout << "[SUCCESS] UDP initialized!" << std::endl;
    
    // Initialize Xbox Controller and FSM
    std::cout << "\n[4] Initializing Xbox Controller and FSM..." << std::endl;
    xboxController = std::make_unique<XboxController>();
    keyboardController = std::make_unique<KeyboardController>();
    fsmController = std::make_unique<FSMController>();
    
    g_targetState = robotInterface->getInitialState();
    
    // Setup gait change callback
    auto refManagerPtr = std::dynamic_pointer_cast<SwitchedModelReferenceManager>(
        robotInterface->getReferenceManagerPtr());
    if (refManagerPtr) {
        refManagerPtr->loadGaitDefinitions(g_gaitFile);
        fsmController->setGaitChangeCallback([refManagerPtr](GaitType gait) {
            std::string gaitName = gaitTypeToString(gait);
            double currentTime = g_simTime.load();
            std::cout << "[Main] Gait change: " << gaitName << " at t=" << currentTime << std::endl;
            refManagerPtr->setGait(gaitName, currentTime);
        });
        std::cout << "[SUCCESS] Gait callback connected" << std::endl;
    }
    
    if (xboxController->isConnected()) {
        std::cout << "[SUCCESS] Xbox controller connected!" << std::endl;
    } else {
        std::cout << "[WARNING] Xbox controller not found" << std::endl;
    }
    
    // Start threads
    std::cout << "\n[5] Starting threads..." << std::endl;
    
    std::thread mpcWorker(mpcThread);
    std::thread controlWorker(controlThread);
    std::thread xboxFsmWorker(xboxFsmThread);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Controller Running!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Xbox Controller:" << std::endl;
    std::cout << "    A - PASSIVE  (no torques)" << std::endl;
    std::cout << "    B - PD       (PD to default stance)" << std::endl;
    std::cout << "    X - STANDING (MPC + WBC)" << std::endl;
    std::cout << "    Y - WALKING  (MPC + Gait)" << std::endl;
    std::cout << "  Press Ctrl+C to exit" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Wait for exit
    while (!exitRequest) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Cleanup
    std::cout << "\n[6] Cleaning up..." << std::endl;
    exitRequest = true;
    
    if (mpcWorker.joinable()) mpcWorker.join();
    if (controlWorker.joinable()) controlWorker.join();
    if (xboxFsmWorker.joinable()) xboxFsmWorker.join();
    
    udpComm->shutdown();
    
    std::cout << "\n[DONE] Controller Complete!" << std::endl;
    
    return 0;
}

/**
 * @file main_udp_nmpc.cpp
 * @brief Quadrotor NMPC Controller via UDP
 * 
 * Refactored to match quadrotor_online architecture:
 * - Uses MPC_MRT_Interface (standard OCS2 pattern)
 * - Analytical dynamics (no AutoDiff)
 * - Proper ReferenceManager
 * - Proper initialization before first MPC call
 * - Xbox controller support for target selection
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include <atomic>
#include <mutex>
#include <signal.h>
#include <vector>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include "ocs2_quadrotor_nmpc/QuadrotorInterface.h"
#include "ocs2_quadrotor_nmpc/QuadrotorUDP.h"
#include "ocs2_quadrotor_nmpc/XboxController.h"
#include "ocs2_quadrotor_nmpc/KeyboardController.h"

using namespace ocs2;
using namespace ocs2_cartpole_quadrotor;

// ============================================================================
// Helper: Unwrap yaw to make it continuous (avoid ±π discontinuity)
// ============================================================================
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

// ============================================================================
// Helper: Convert angular velocity from body frame to world frame
// MuJoCo qvel[3:6] is in BODY frame, OCS2 expects WORLD frame
// ============================================================================
void convertAngularVelocityBodyToWorld(double roll, double pitch, double yaw,
                                        double wx_body, double wy_body, double wz_body,
                                        double& wx_world, double& wy_world, double& wz_world) {
    // Compute rotation matrix from Euler angles (ZYX convention)
    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    
    // Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
    // This transforms vectors from body frame to world frame
    double R00 = cy * cp;
    double R01 = cy * sp * sr - sy * cr;
    double R02 = cy * sp * cr + sy * sr;
    double R10 = sy * cp;
    double R11 = sy * sp * sr + cy * cr;
    double R12 = sy * sp * cr - cy * sr;
    double R20 = -sp;
    double R21 = cp * sr;
    double R22 = cp * cr;
    
    // omega_world = R * omega_body
    wx_world = R00 * wx_body + R01 * wy_body + R02 * wz_body;
    wy_world = R10 * wx_body + R11 * wy_body + R12 * wz_body;
    wz_world = R20 * wx_body + R21 * wy_body + R22 * wz_body;
}

// ============================================================================
// Global Variables
// ============================================================================
std::atomic<bool> exitRequest{false};
std::atomic<bool> mpcThreadRunning{false};
std::atomic<bool> mpcInitialized{false};

// Continuous yaw tracker
static double g_continuousYaw = 0.0;
static bool g_yawInitialized = false;

// MPC objects - need raw pointer for mpc since MPC_MRT_Interface takes reference
std::unique_ptr<QuadrotorInterface> quadrotorInterface;
std::unique_ptr<GaussNewtonDDP_MPC> mpcSolver;
std::unique_ptr<MPC_MRT_Interface> mpcMrtInterface;
QuadrotorParameters quadrotorParams;

// Target management
std::vector<vector_t> targets;
std::atomic<int> currentTargetIndex{0};
vector_t currentTarget;
vector_t dynamicTarget;
std::mutex targetMutex;
std::atomic<bool> useJoystickTarget{false};

// Xbox controller
std::unique_ptr<XboxController> xboxController;

// Keyboard controller
std::unique_ptr<KeyboardController> keyboardController;

// Control storage for MRT thread
vector_t g_currentControl;
std::mutex g_controlMutex;

// Current state (for joystick relative control)
vector_t g_currentState;
std::mutex g_stateMutex;

// ============================================================================
// Signal handler
// ============================================================================
void signalHandler(int signal) {
    std::cout << "\n[Main] Received signal " << signal << ", shutting down...\n";
    exitRequest = true;
}

// ============================================================================
// MPC Thread Function - Computes MPC at fixed rate
// ============================================================================
void mpcThread(QuadrotorUDP* udp) {
    std::cout << "[MPC Thread] Started" << std::endl;
    
    const double mpcDt = 0.02;  // MPC update rate (50 Hz)
    double lastMpcTime = -1.0;
    
    double hoverThrust = quadrotorParams.quadrotorMass_ * quadrotorParams.gravity_;
    g_currentControl = vector_t::Zero(INPUT_DIM);
    g_currentControl(0) = hoverThrust;
    
    // Joystick velocity scaling (same as online version)
    const double maxVelXY = 1.0;    // m/s max velocity in X/Y
    const double maxVelZ = 0.5;     // m/s max velocity in Z
    const double maxYawRate = 0.5;  // rad/s max yaw rate
    
    mpcThreadRunning = true;
    
    // Wait for first valid state from simulator
    std::cout << "[MPC Thread] Waiting for simulator state..." << std::endl;
    double currentTime;
    vector_t currentState(STATE_DIM);
    bool gotFirstState = false;
    
    while (!exitRequest && !gotFirstState) {
        if (udp->receiveState(currentTime, currentState)) {
            // Check if state is valid (reasonable values)
            if (std::abs(currentState(2)) < 10.0 &&  // z < 10m
                std::abs(currentState(3)) < M_PI &&   // roll < 180deg
                std::abs(currentState(4)) < M_PI &&   // pitch < 180deg
                !std::isnan(currentState.sum())) {
                gotFirstState = true;
                std::cout << "[MPC Thread] Got first valid state at t=" << currentTime << std::endl;
                std::cout << "[MPC Thread] State: " << currentState.transpose() << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (!gotFirstState) {
        std::cout << "[MPC Thread] No valid state received, exiting" << std::endl;
        mpcThreadRunning = false;
        return;
    }
    
    // Initialize MPC with the first state
    std::cout << "[MPC Thread] Initializing MPC with first observation..." << std::endl;
    {
        // Set initial observation
        SystemObservation initObs;
        initObs.time = currentTime;
        initObs.state = currentState;
        initObs.input = g_currentControl;
        mpcMrtInterface->setCurrentObservation(initObs);
    }
    
    mpcInitialized = true;
    std::cout << "[MPC Thread] MPC initialized, starting control loop" << std::endl;
    
    // Main MPC loop
    uint64_t stepCount = 0;
    while (!exitRequest) {
        // ================================================================
        // Handle Xbox controller input (same as online version)
        // ================================================================
        if (xboxController && xboxController->isConnected()) {
            // Check for preset target selection (buttons A, B, X, Y)
            int targetRequest = xboxController->getTargetChangeRequest();
            if (targetRequest >= 0 && targetRequest < static_cast<int>(targets.size())) {
                std::lock_guard<std::mutex> lock(targetMutex);
                dynamicTarget = targets[targetRequest];
                currentTarget = dynamicTarget;
                useJoystickTarget.store(false);
                currentTargetIndex.store(targetRequest);
                
                std::cout << "[Xbox] Target " << (targetRequest + 1) << " selected: ["
                          << dynamicTarget(0) << ", " << dynamicTarget(1) << ", " 
                          << dynamicTarget(2) << "]" << std::endl;
            }
            
            // Get joystick input for velocity control
            XboxInput input = xboxController->getInput();
            
            // If joystick is active, update dynamic target based on velocity
            if (std::abs(input.lx) > 0.01 || std::abs(input.ly) > 0.01 || 
                std::abs(input.ry) > 0.01 || std::abs(input.rx) > 0.01) {
                
                std::lock_guard<std::mutex> stateLock(g_stateMutex);
                std::lock_guard<std::mutex> lock(targetMutex);
                
                // On first joystick movement, initialize dynamic target from current position
                if (!useJoystickTarget.load()) {
                    dynamicTarget(0) = g_currentState(0);
                    dynamicTarget(1) = g_currentState(1);
                    dynamicTarget(2) = g_currentState(2);
                    dynamicTarget(5) = 0;  // Reset yaw
                    for (int i = 3; i < 5; i++) dynamicTarget(i) = 0;
                    for (int i = 6; i < STATE_DIM; i++) dynamicTarget(i) = 0;
                }
                
                useJoystickTarget.store(true);
                
                // Update target based on joystick input
                // Left stick: X/Y position offset (only if stick is moved)
                if (std::abs(input.ly) > 0.01) {
                    dynamicTarget(0) = g_currentState(0) + input.ly * maxVelXY * 2.0;  // Forward/back
                }
                if (std::abs(input.lx) > 0.01) {
                    dynamicTarget(1) = g_currentState(1) + input.lx * maxVelXY * 2.0;  // Left/right
                }
                
                // Right stick Y: Z position offset (only if stick is moved)
                if (std::abs(input.ry) > 0.01) {
                    dynamicTarget(2) = std::clamp(g_currentState(2) + input.ry * maxVelZ * 2.0, 0.3, 5.0);
                }
                // If ry is not moved, keep the previous target Z (don't follow current Z)
                
                // Yaw target from right stick X (only if stick is moved)
                if (std::abs(input.rx) > 0.01) {
                    dynamicTarget(5) = input.rx * M_PI;  // Yaw angle
                }
                
                currentTarget = dynamicTarget;
            }
        }
        
        // Also check keyboard input
        KeyboardInput kbInput;
        if (keyboardController && keyboardController->isConnected()) {
            kbInput = keyboardController->getInput();
            
            // Keyboard target selection (1-4 keys)
            int kbTargetReq = keyboardController->getTargetChangeRequest();
            if (kbTargetReq >= 0 && kbTargetReq < static_cast<int>(targets.size())) {
                std::lock_guard<std::mutex> lock(targetMutex);
                dynamicTarget = targets[kbTargetReq];
                currentTarget = dynamicTarget;
                useJoystickTarget.store(false);
                currentTargetIndex.store(kbTargetReq);
                std::cout << "[Keyboard] Target " << (kbTargetReq + 1) << " selected" << std::endl;
            }
            
            // If keyboard has velocity input
            if (std::abs(kbInput.lx) > 0.01 || std::abs(kbInput.ly) > 0.01 || 
                std::abs(kbInput.ry) > 0.01 || std::abs(kbInput.rx) > 0.01) {
                
                std::lock_guard<std::mutex> stateLock(g_stateMutex);
                std::lock_guard<std::mutex> lock(targetMutex);
                
                if (!useJoystickTarget.load()) {
                    dynamicTarget(0) = g_currentState(0);
                    dynamicTarget(1) = g_currentState(1);
                    dynamicTarget(2) = g_currentState(2);
                    dynamicTarget(5) = 0;
                    for (int i = 3; i < 5; i++) dynamicTarget(i) = 0;
                    for (int i = 6; i < STATE_DIM; i++) dynamicTarget(i) = 0;
                }
                
                useJoystickTarget.store(true);
                
                if (std::abs(kbInput.ly) > 0.01) {
                    dynamicTarget(0) = g_currentState(0) + kbInput.ly * maxVelXY * 2.0;
                }
                if (std::abs(kbInput.lx) > 0.01) {
                    dynamicTarget(1) = g_currentState(1) + kbInput.lx * maxVelXY * 2.0;
                }
                if (std::abs(kbInput.ry) > 0.01) {
                    dynamicTarget(2) = std::clamp(g_currentState(2) + kbInput.ry * maxVelZ * 2.0, 0.3, 5.0);
                }
                if (std::abs(kbInput.rx) > 0.01) {
                    dynamicTarget(5) = kbInput.rx * M_PI;
                }
                
                currentTarget = dynamicTarget;
            }
        }
        
        // Receive state from simulator
        if (!udp->receiveState(currentTime, currentState)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        
        // Apply continuous yaw tracking to avoid ±π discontinuity
        double rawYaw = currentState(5);
        if (!g_yawInitialized) {
            g_continuousYaw = rawYaw;
            g_yawInitialized = true;
        } else {
            g_continuousYaw = unwrapYaw(rawYaw, g_continuousYaw);
        }
        currentState(5) = g_continuousYaw;
        
        // Convert angular velocity from body frame to world frame
        // MuJoCo (and most simulators) report angular velocity in body frame
        // OCS2 dynamics expects world frame angular velocity
        double roll = currentState(3);
        double pitch = currentState(4);
        double yaw = rawYaw;  // Use raw yaw for rotation matrix (not continuous)
        double wx_body = currentState(9);
        double wy_body = currentState(10);
        double wz_body = currentState(11);
        double wx_world, wy_world, wz_world;
        convertAngularVelocityBodyToWorld(roll, pitch, yaw, 
                                          wx_body, wy_body, wz_body,
                                          wx_world, wy_world, wz_world);
        currentState(9) = wx_world;
        currentState(10) = wy_world;
        currentState(11) = wz_world;
        
        // Update global state for joystick control
        {
            std::lock_guard<std::mutex> stateLock(g_stateMutex);
            g_currentState = currentState;
        }
        
        // Skip invalid states
        if (std::isnan(currentState.sum()) || 
            std::abs(currentState(3)) > M_PI || 
            std::abs(currentState(4)) > M_PI) {
            std::cout << "[MPC Thread] WARNING: Invalid state received, skipping" << std::endl;
            continue;
        }
        
        // Update MPC target (same format as online version)
        {
            std::lock_guard<std::mutex> lock(targetMutex);
            
            TargetTrajectories targetTrajectories;
            targetTrajectories.timeTrajectory = {0.0};
            targetTrajectories.stateTrajectory = {currentTarget};
            targetTrajectories.inputTrajectory = {vector_t::Zero(INPUT_DIM)};
            mpcMrtInterface->getReferenceManager().setTargetTrajectories(targetTrajectories);
        }
        
        // MPC update at fixed rate
        if (currentTime - lastMpcTime >= mpcDt || lastMpcTime < 0) {
            try {
                std::lock_guard<std::mutex> ctrlLock(g_controlMutex);
                
                SystemObservation observation;
                observation.time = currentTime;
                observation.state = currentState;
                observation.input = g_currentControl;
                mpcMrtInterface->setCurrentObservation(observation);
                
                mpcMrtInterface->advanceMpc();
                mpcMrtInterface->updatePolicy();
                
                vector_t optimalState;
                vector_t localControl;
                size_t mode;
                mpcMrtInterface->evaluatePolicy(currentTime, currentState, optimalState, localControl, mode);
                
                // Clamp control
                localControl(0) = std::clamp(localControl(0), 0.2 * hoverThrust, 2.0 * hoverThrust);
                localControl(1) = std::clamp(localControl(1), -1.0, 1.0);
                localControl(2) = std::clamp(localControl(2), -1.0, 1.0);
                localControl(3) = std::clamp(localControl(3), -0.5, 0.5);
                
                g_currentControl = localControl;
                
            } catch (const std::exception& e) {
                std::cerr << "[MPC Thread] MPC error: " << e.what() << std::endl;
                // Use hover thrust on error
                g_currentControl = vector_t::Zero(INPUT_DIM);
                g_currentControl(0) = hoverThrust;
            }
            
            lastMpcTime = currentTime;
        }
        
        // Send command to simulator
        {
            std::lock_guard<std::mutex> ctrlLock(g_controlMutex);
            udp->sendCommand(currentTime, g_currentControl);
        }
        
        // Print status periodically
        stepCount++;
        if (stepCount % 50 == 0) {
            std::lock_guard<std::mutex> ctrlLock(g_controlMutex);
            std::lock_guard<std::mutex> tgtLock(targetMutex);
            std::cout << std::fixed << std::setprecision(3)
                      << "[" << stepCount << "] t=" << currentTime
                      << " pos=[" << currentState(0) << "," << currentState(1) << "," << currentState(2) << "]"
                      << " tgt=[" << currentTarget(0) << "," << currentTarget(1) << "," << currentTarget(2) << "]"
                      << " | Fz=" << g_currentControl(0)
                      << " M=[" << g_currentControl(1) << "," << g_currentControl(2) << "," << g_currentControl(3) << "]"
                      << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    mpcThreadRunning = false;
    std::cout << "[MPC Thread] Stopped" << std::endl;
}

// ============================================================================
// Main function
// ============================================================================
int main(int argc, char** argv) {
    std::cout << "=" << std::string(58, '=') << "=\n";
    std::cout << "Quadrotor NMPC Controller via UDP (with Xbox Controller)\n";
    std::cout << "=" << std::string(58, '=') << "=\n";
    
    // Config paths
    std::string configFile = "src/ocs2_quadrotor_nmpc/config/mpc/task.info";
    std::string libraryFolder = "/tmp/ocs2_quadrotor";
    
    if (argc > 1) {
        configFile = std::string(argv[1]) + "/task.info";
    }
    
    std::cout << "Config: " << configFile << "\n";
    std::cout << "Run simulator: ./simulate/build/mujoco_ocs quadrotor\n";
    std::cout << "=" << std::string(58, '=') << "=\n\n";
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // ========================================
        // 1. Initialize UDP interface
        // ========================================
        std::cout << "[*] Initializing UDP interface...\n";
        QuadrotorUDP udp;
        udp.initialize();
        
        // ========================================
        // 2. Initialize Xbox Controller
        // ========================================
        std::cout << "[*] Initializing Xbox Controller...\n";
        xboxController = std::make_unique<XboxController>();
        if (xboxController->isConnected()) {
            std::cout << "[SUCCESS] Xbox controller connected!\n";
        } else {
            std::cout << "[INFO] No Xbox controller found. Using preset targets only.\n";
        }
        
        // ========================================
        // 2b. Initialize Keyboard Controller
        // ========================================
        std::cout << "[*] Initializing Keyboard Controller...\n";
        keyboardController = std::make_unique<KeyboardController>();
        if (keyboardController->isConnected()) {
            std::cout << "[SUCCESS] Keyboard controller initialized!\n";
        }
        
        // ========================================
        // 3. Build Quadrotor Interface
        // ========================================
        std::cout << "[*] Building QuadrotorInterface...\n";
        quadrotorInterface = std::make_unique<QuadrotorInterface>(configFile, libraryFolder, true);
        quadrotorParams = quadrotorInterface->getQuadrotorParameters();
        
        // ========================================
        // 4. Create GaussNewtonDDP_MPC
        // ========================================
        std::cout << "[*] Creating GaussNewtonDDP_MPC solver...\n";
        mpcSolver = std::make_unique<GaussNewtonDDP_MPC>(
            quadrotorInterface->mpcSettings(),
            quadrotorInterface->ddpSettings(),
            quadrotorInterface->getRollout(),
            quadrotorInterface->getOptimalControlProblem(),
            quadrotorInterface->getInitializer());
        
        mpcSolver->getSolverPtr()->setReferenceManager(quadrotorInterface->getReferenceManagerPtr());
        
        // ========================================
        // 5. Create MPC_MRT_Interface
        // ========================================
        std::cout << "[*] Creating MPC_MRT_Interface...\n";
        mpcMrtInterface = std::make_unique<MPC_MRT_Interface>(*mpcSolver);
        mpcMrtInterface->initRollout(&quadrotorInterface->getRollout());
        
        // ========================================
        // 6. Initialize targets (same as online version)
        // ========================================
        vector_t targetState = quadrotorInterface->getInitialTarget();
        
        // Define preset targets
        targets.resize(4);
        targets[0] = targetState;
        
        targets[1] = vector_t::Zero(STATE_DIM);
        targets[1] << 1.0, 0.0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        
        targets[2] = vector_t::Zero(STATE_DIM);
        targets[2] << 0.0, 1.0, 2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        
        targets[3] = vector_t::Zero(STATE_DIM);
        targets[3] << -0.5, -0.5, 0.8, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        
        currentTarget = targets[0];
        dynamicTarget = targets[0];
        g_currentState = quadrotorInterface->getInitialState();
        
        const vector_t& initialState = quadrotorInterface->getInitialState();
        double hoverThrust = quadrotorParams.quadrotorMass_ * quadrotorParams.gravity_;
        
        std::cout << "\n[*] Initial state: [" << initialState.transpose() << "]\n";
        std::cout << "[*] Target state:  [" << currentTarget.transpose() << "]\n";
        std::cout << "[*] Hover thrust:  " << hoverThrust << " N\n";
        
        // Set initial target trajectories (same as online version)
        TargetTrajectories initTargets;
        initTargets.timeTrajectory = {0.0};
        initTargets.stateTrajectory = {currentTarget};
        initTargets.inputTrajectory = {vector_t::Zero(INPUT_DIM)};
        mpcMrtInterface->getReferenceManager().setTargetTrajectories(initTargets);
        
        // Initialize control with hover
        g_currentControl = vector_t::Zero(INPUT_DIM);
        g_currentControl(0) = hoverThrust;
        
        // ========================================
        // 7. Start MPC thread
        // ========================================
        std::thread mpcThreadHandle(mpcThread, &udp);
        
        std::cout << "\n[*] MPC thread started\n";
        std::cout << "    Waiting for simulator to connect...\n\n";
        
        std::cout << "========================================\n";
        std::cout << "  Controls:\n";
        std::cout << "========================================\n";
        std::cout << "  Ctrl+C - Exit\n";
        std::cout << "  Xbox Controller:\n";
        std::cout << "    Left Stick  - Move X/Y target\n";
        std::cout << "    Right Stick Y - Move Z target\n";
        std::cout << "    Right Stick X - Yaw target\n";
        std::cout << "    A/B/X/Y - Select preset target 1/2/3/4\n";
        std::cout << "========================================\n";
        std::cout << "  Preset Targets:\n";
        std::cout << "    1: [" << targets[0](0) << ", " << targets[0](1) << ", " << targets[0](2) << "]\n";
        std::cout << "    2: [" << targets[1](0) << ", " << targets[1](1) << ", " << targets[1](2) << "]\n";
        std::cout << "    3: [" << targets[2](0) << ", " << targets[2](1) << ", " << targets[2](2) << "]\n";
        std::cout << "    4: [" << targets[3](0) << ", " << targets[3](1) << ", " << targets[3](2) << "]\n";
        std::cout << "========================================\n\n";
        
        // ========================================
        // 8. Main loop - monitoring
        // ========================================
        while (!exitRequest) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            if (!mpcInitialized) {
                std::cout << "[Main] Waiting for MPC initialization..." << std::endl;
            }
        }
        
        // ========================================
        // 9. Cleanup
        // ========================================
        if (mpcThreadHandle.joinable()) {
            mpcThreadHandle.join();
        }
        
        xboxController.reset();
        keyboardController.reset();
        udp.shutdown();
        std::cout << "\n[*] Shutdown complete\n";
        std::cout << "=" << std::string(58, '=') << "=\n";
        
        return EXIT_SUCCESS;
        
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}

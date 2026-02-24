/**
 * @file main_interactive_gui.cpp
 * @brief Interactive GUI for Quadrotor Online NMPC with MuJoCo
 * 
 * This program provides a full interactive GUI similar to simulate folder
 * Features:
 * - Full MuJoCo GUI with UI panels
 * - Real-time MPC control
 * - Interactive target selection
 * - Pause, reset, and playback controls
 */

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <quadrotor_online/QuadrotorInterface.h>
#include <quadrotor_online/package_path.h>
#include <quadrotor_online/XboxController.h>
#include <quadrotor_online/KeyboardController.h>

#include <mujoco/mujoco.h>
#include "mujoco_gui/glfw_adapter.h"
#include "mujoco_gui/simulate.h"

using namespace ocs2;
using namespace quadrotor_online;

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
// Global Variables
// ============================================================================
std::atomic<bool> exitRequest{false};
std::atomic<bool> mpcThreadRunning{false};

// Continuous yaw tracker
static double g_continuousYaw = 0.0;
static bool g_yawInitialized = false;

// MPC objects (need to be accessible from both threads)
std::unique_ptr<QuadrotorInterface> quadrotorInterface;
std::unique_ptr<MPC_MRT_Interface> mpcMrtInterface;
QuadrotorParameters quadrotorParams;

// Target management
std::vector<vector_t> targets;
std::atomic<int> currentTargetIndex{0};
std::mutex mpcMutex;

// Control storage for physics thread
vector_t g_currentControl;
std::mutex g_controlMutex;

// Xbox controller for joystick input
std::unique_ptr<XboxController> xboxController;

// Keyboard controller for keyboard input
std::unique_ptr<KeyboardController> keyboardController;

// Dynamic target position (updated by joystick)
vector_t dynamicTarget;
std::mutex targetMutex;
std::atomic<bool> useJoystickTarget{false};

// ============================================================================
// MPC Thread Function - Only computes MPC, does not step physics
// ============================================================================
void mpcThread(mujoco::Simulate* sim) {
    std::cout << "[MPC Thread] Started" << std::endl;
    
    const double mpcDt = 0.02;  // MPC update rate (50 Hz)
    double lastMpcTime = -1.0;
    
    g_currentControl = vector_t::Zero(INPUT_DIM);
    g_currentControl(0) = quadrotorParams.quadrotorMass_ * quadrotorParams.gravity_;
    
    mpcThreadRunning = true;
    
    // Joystick velocity scaling
    const double maxVelXY = 1.0;    // m/s max velocity in X/Y
    const double maxVelZ = 0.5;     // m/s max velocity in Z
    const double maxYawRate = 0.5;  // rad/s max yaw rate
    
    while (!exitRequest) {
        // ================================================================
        // Handle Xbox controller input
        // ================================================================
        if (xboxController && xboxController->isConnected()) {
            // Check for preset target selection (buttons A, B, X, Y)
            int targetRequest = xboxController->getTargetChangeRequest();
            if (targetRequest >= 0 && targetRequest < static_cast<int>(targets.size())) {
                std::lock_guard<std::mutex> lock(targetMutex);
                dynamicTarget = targets[targetRequest];
                useJoystickTarget.store(false);  // Use preset target
                
                // Also update UI selection
                sim->target_index = targetRequest;
                
                std::cout << "[Xbox] Target " << (targetRequest + 1) << " selected: ["
                          << dynamicTarget(0) << ", " << dynamicTarget(1) << ", " 
                          << dynamicTarget(2) << "]" << std::endl;
            }
            
            // Check for start button (toggle run/pause)
            if (xboxController->wasStartPressed()) {
                sim->run = 1 - sim->run;
                std::cout << "[Xbox] " << (sim->run ? "RUNNING" : "PAUSED") << std::endl;
            }
            
            // Get joystick input for velocity control
            XboxInput input = xboxController->getInput();
            
            // If joystick is active, update dynamic target based on velocity
            if (std::abs(input.lx) > 0.01 || std::abs(input.ly) > 0.01 || 
                std::abs(input.ry) > 0.01 || std::abs(input.rx) > 0.01) {
                
                useJoystickTarget.store(true);
                
                // Read current quadrotor position
                if (sim->m_ && sim->d_) {
                    std::lock_guard<mujoco::SimulateMutex> simLock(sim->mtx);
                    std::lock_guard<std::mutex> lock(targetMutex);
                    
                    // Get current position as base
                    double currentX = sim->d_->qpos[0];
                    double currentY = sim->d_->qpos[1];
                    double currentZ = sim->d_->qpos[2];
                    
                    // Set target ahead of current position based on joystick
                    // Left stick: X/Y position offset
                    // Right stick Y: Z position offset
                    dynamicTarget(0) = currentX + input.ly * maxVelXY * 2.0;  // Forward/back
                    dynamicTarget(1) = currentY + input.lx * maxVelXY * 2.0;  // Left/right
                    dynamicTarget(2) = std::clamp(currentZ + input.ry * maxVelZ * 2.0, 0.3, 5.0);  // Up/down
                    
                    // Yaw target from right stick X
                    dynamicTarget(5) = input.rx * M_PI;  // Yaw angle
                    
                    // Zero out other states
                    for (int i = 3; i < 5; i++) dynamicTarget(i) = 0;
                    for (int i = 6; i < STATE_DIM; i++) dynamicTarget(i) = 0;
                }
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
                useJoystickTarget.store(false);
                sim->target_index = kbTargetReq;
                std::cout << "[Keyboard] Target " << (kbTargetReq + 1) << " selected" << std::endl;
            }
            
            // Keyboard start button (Enter)
            if (keyboardController->wasStartPressed()) {
                sim->run = 1 - sim->run;
                std::cout << "[Keyboard] " << (sim->run ? "RUNNING" : "PAUSED") << std::endl;
            }
            
            // If keyboard is active for movement
            if (std::abs(kbInput.lx) > 0.01 || std::abs(kbInput.ly) > 0.01 || 
                std::abs(kbInput.ry) > 0.01 || std::abs(kbInput.rx) > 0.01) {
                
                useJoystickTarget.store(true);
                
                if (sim->m_ && sim->d_) {
                    std::lock_guard<mujoco::SimulateMutex> simLock(sim->mtx);
                    std::lock_guard<std::mutex> lock(targetMutex);
                    
                    double currentX = sim->d_->qpos[0];
                    double currentY = sim->d_->qpos[1];
                    double currentZ = sim->d_->qpos[2];
                    
                    dynamicTarget(0) = currentX + kbInput.ly * maxVelXY * 2.0;
                    dynamicTarget(1) = currentY + kbInput.lx * maxVelXY * 2.0;
                    dynamicTarget(2) = std::clamp(currentZ + kbInput.ry * maxVelZ * 2.0, 0.3, 5.0);
                    dynamicTarget(5) = kbInput.rx * M_PI;
                    
                    for (int i = 3; i < 5; i++) dynamicTarget(i) = 0;
                    for (int i = 6; i < STATE_DIM; i++) dynamicTarget(i) = 0;
                }
            }
        }
        
        // ================================================================
        // Handle UI target change
        // ================================================================
        if (sim->target_changed.exchange(false)) {
            int newTargetIdx = sim->target_index;
            if (newTargetIdx >= 0 && newTargetIdx < static_cast<int>(targets.size())) {
                std::lock_guard<std::mutex> lock(targetMutex);
                dynamicTarget = targets[newTargetIdx];
                useJoystickTarget.store(false);
                currentTargetIndex.store(newTargetIdx);
                
                std::cout << "[UI] Target updated to " << (newTargetIdx + 1) 
                          << ": [" << dynamicTarget(0) << ", " 
                          << dynamicTarget(1) << ", " 
                          << dynamicTarget(2) << "]" << std::endl;
            }
        }
        
        // ================================================================
        // Update MPC target
        // ================================================================
        {
            std::lock_guard<std::mutex> mpcLock(mpcMutex);
            std::lock_guard<std::mutex> lock(targetMutex);
            
            TargetTrajectories targetTrajectories;
            targetTrajectories.timeTrajectory = {0.0};
            targetTrajectories.stateTrajectory = {dynamicTarget};
            targetTrajectories.inputTrajectory = {vector_t::Zero(INPUT_DIM)};
            mpcMrtInterface->getReferenceManager().setTargetTrajectories(targetTrajectories);
        }
        
        if (sim->m_ && sim->d_) {
            // Read state with lock
            vector_t currentState(STATE_DIM);
            double simTime;
            {
                std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
                simTime = sim->d_->time;
                
                currentState(0) = sim->d_->qpos[0];  // x
                currentState(1) = sim->d_->qpos[1];  // y
                currentState(2) = sim->d_->qpos[2];  // z
                
                // Quaternion to Euler
                double qw = sim->d_->qpos[3];
                double qx = sim->d_->qpos[4];
                double qy = sim->d_->qpos[5];
                double qz = sim->d_->qpos[6];
                
                double sinr_cosp = 2.0 * (qw * qx + qy * qz);
                double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
                currentState(3) = std::atan2(sinr_cosp, cosr_cosp);
                
                double sinp = 2.0 * (qw * qy - qz * qx);
                if (std::abs(sinp) >= 1)
                    currentState(4) = std::copysign(M_PI / 2, sinp);
                else
                    currentState(4) = std::asin(sinp);
                
                double siny_cosp = 2.0 * (qw * qz + qx * qy);
                double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                double rawYaw = std::atan2(siny_cosp, cosy_cosp);
                
                // Use continuous yaw to avoid ±π discontinuity
                if (!g_yawInitialized) {
                    g_continuousYaw = rawYaw;
                    g_yawInitialized = true;
                } else {
                    g_continuousYaw = unwrapYaw(rawYaw, g_continuousYaw);
                }
                currentState(5) = g_continuousYaw;
                
                // Linear velocities (world frame) - directly from qvel[0:3]
                currentState(6) = sim->d_->qvel[0];  // vx
                currentState(7) = sim->d_->qvel[1];  // vy
                currentState(8) = sim->d_->qvel[2];  // vz
                
                // Angular velocities: MuJoCo qvel[3:6] is BODY frame!
                // OCS2 expects WORLD frame angular velocity
                // Convert: omega_world = R_body_to_world * omega_body
                double wx_body = sim->d_->qvel[3];
                double wy_body = sim->d_->qvel[4];
                double wz_body = sim->d_->qvel[5];
                
                // Rotation matrix from body to world (using quaternion)
                // R = [R00 R01 R02; R10 R11 R12; R20 R21 R22]
                double R00 = 1.0 - 2.0 * (qy * qy + qz * qz);
                double R01 = 2.0 * (qx * qy - qz * qw);
                double R02 = 2.0 * (qx * qz + qy * qw);
                double R10 = 2.0 * (qx * qy + qz * qw);
                double R11 = 1.0 - 2.0 * (qx * qx + qz * qz);
                double R12 = 2.0 * (qy * qz - qx * qw);
                double R20 = 2.0 * (qx * qz - qy * qw);
                double R21 = 2.0 * (qy * qz + qx * qw);
                double R22 = 1.0 - 2.0 * (qx * qx + qy * qy);
                
                // omega_world = R * omega_body
                currentState(9)  = R00 * wx_body + R01 * wy_body + R02 * wz_body;
                currentState(10) = R10 * wx_body + R11 * wy_body + R12 * wz_body;
                currentState(11) = R20 * wx_body + R21 * wy_body + R22 * wz_body;
            }
            
            // MPC update (without holding sim mutex)
            if (simTime - lastMpcTime >= mpcDt || lastMpcTime < 0) {
                vector_t localControl;
                {
                    std::lock_guard<std::mutex> mpcLock(mpcMutex);
                    std::lock_guard<std::mutex> ctrlLock(g_controlMutex);
                    
                    SystemObservation observation;
                    observation.time = simTime;
                    observation.state = currentState;
                    observation.input = g_currentControl;
                    mpcMrtInterface->setCurrentObservation(observation);
                    
                    mpcMrtInterface->advanceMpc();
                    mpcMrtInterface->updatePolicy();
                    
                    vector_t optimalState;
                    size_t mode;
                    mpcMrtInterface->evaluatePolicy(simTime, currentState, optimalState, localControl, mode);
                    
                    // Clamp control
                    double hoverThrust = quadrotorParams.quadrotorMass_ * quadrotorParams.gravity_;
                    localControl(0) = std::clamp(localControl(0), 0.2 * hoverThrust, 2.0 * hoverThrust);
                    localControl(1) = std::clamp(localControl(1), -1.0, 1.0);
                    localControl(2) = std::clamp(localControl(2), -1.0, 1.0);
                    localControl(3) = std::clamp(localControl(3), -0.5, 0.5);
                    
                    g_currentControl = localControl;
                }
                lastMpcTime = simTime;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    mpcThreadRunning = false;
    std::cout << "[MPC Thread] Stopped" << std::endl;
}

// ============================================================================
// Physics Thread Function - Loads model and runs physics simulation
// ============================================================================
void physicsThread(mujoco::Simulate* sim, const std::string& modelPath, const vector_t& initialState) {
    std::cout << "[Physics Thread] Started" << std::endl;
    
    // Load model first (this must happen after RenderLoop starts)
    std::cout << "[Physics Thread] Loading model: " << modelPath << std::endl;
    char error[1000] = "Could not load model";
    mjModel* m = mj_loadXML(modelPath.c_str(), nullptr, error, 1000);
    if (!m) {
        std::cerr << "[Physics Thread] Load model error: " << error << std::endl;
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
    
    // Set initial state
    d->qpos[0] = initialState(0);
    d->qpos[1] = initialState(1);
    d->qpos[2] = initialState(2);
    d->qpos[3] = 1.0;  // qw
    d->qpos[4] = 0.0;  // qx
    d->qpos[5] = 0.0;  // qy
    d->qpos[6] = 0.0;  // qz
    mj_forward(m, d);
    
    // Load into simulate object - this will wait for render thread to accept it
    sim->Load(m, d, modelPath.c_str());
    std::cout << "[Physics Thread] Model loaded successfully!" << std::endl;
    
    // Initialize control
    {
        std::lock_guard<std::mutex> ctrlLock(g_controlMutex);
        g_currentControl = vector_t::Zero(INPUT_DIM);
        g_currentControl(0) = quadrotorParams.quadrotorMass_ * quadrotorParams.gravity_;
    }
    
    using Clock = std::chrono::steady_clock;
    auto syncCPU = Clock::now();
    mjtNum syncSim = 0;
    
    const double syncMisalign = 0.1;
    const double simRefreshFraction = 0.7;
    
    while (!exitRequest && !sim->exitrequest.load()) {
        // Sleep to let main thread run
        if (sim->run && sim->busywait) {
            std::this_thread::yield();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        {
            std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
            
            if (sim->m_ && sim->d_) {
                // ============================================================
                // CRITICAL: Call Sync() FIRST to process UI inputs
                // This handles: perturbation, reset, selection, etc.
                // Sync() will clear xfrc_applied and apply perturbation forces
                // ============================================================
                sim->Sync();
                
                // ============================================================
                // Apply MPC control AFTER Sync() (add to perturbation forces)
                // ============================================================
                int quadBodyId = mj_name2id(sim->m_, mjOBJ_BODY, "quadrotor");
                if (quadBodyId >= 0) {
                    vector_t ctrl;
                    {
                        std::lock_guard<std::mutex> ctrlLock(g_controlMutex);
                        ctrl = g_currentControl;
                    }
                    
                    double qw = sim->d_->qpos[3];
                    double qx = sim->d_->qpos[4];
                    double qy = sim->d_->qpos[5];
                    double qz = sim->d_->qpos[6];
                    
                    double Fz = ctrl(0);
                    
                    // Body z-axis to world frame
                    double bz_x = 2.0 * (qx * qz + qw * qy);
                    double bz_y = 2.0 * (qy * qz - qw * qx);
                    double bz_z = 1.0 - 2.0 * (qx * qx + qy * qy);
                    
                    // ADD to existing forces (perturbation may have set some)
                    sim->d_->xfrc_applied[6*quadBodyId + 0] += Fz * bz_x;
                    sim->d_->xfrc_applied[6*quadBodyId + 1] += Fz * bz_y;
                    sim->d_->xfrc_applied[6*quadBodyId + 2] += Fz * bz_z;
                    
                    // Torques - rotation matrix
                    double Mx = ctrl(1);
                    double My = ctrl(2);
                    double Mz = ctrl(3);
                    
                    double R00 = 1.0 - 2.0 * (qy * qy + qz * qz);
                    double R01 = 2.0 * (qx * qy - qz * qw);
                    double R02 = 2.0 * (qx * qz + qy * qw);
                    double R10 = 2.0 * (qx * qy + qz * qw);
                    double R11 = 1.0 - 2.0 * (qx * qx + qz * qz);
                    double R12 = 2.0 * (qy * qz - qx * qw);
                    double R20 = 2.0 * (qx * qz - qy * qw);
                    double R21 = 2.0 * (qy * qz + qx * qw);
                    double R22 = 1.0 - 2.0 * (qx * qx + qy * qy);
                    
                    // ADD to existing torques
                    sim->d_->xfrc_applied[6*quadBodyId + 3] += R00 * Mx + R01 * My + R02 * Mz;
                    sim->d_->xfrc_applied[6*quadBodyId + 4] += R10 * Mx + R11 * My + R12 * Mz;
                    sim->d_->xfrc_applied[6*quadBodyId + 5] += R20 * Mx + R21 * My + R22 * Mz;
                }
                
                // ============================================================
                // Step physics simulation
                // ============================================================
                if (sim->run) {
                    auto startCPU = Clock::now();
                    auto elapsedCPU = startCPU - syncCPU;
                    double elapsedSim = sim->d_->time - syncSim;
                    
                    // Slowdown factor
                    double slowdown = 100.0 / sim->percentRealTime[sim->real_time_index];
                    
                    // Check misalignment
                    bool misaligned = std::abs(std::chrono::duration<double>(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;
                    
                    if (elapsedSim < 0 || elapsedCPU.count() < 0 || 
                        syncCPU.time_since_epoch().count() == 0 || misaligned || sim->speed_changed) {
                        // Re-sync
                        syncCPU = startCPU;
                        syncSim = sim->d_->time;
                        sim->speed_changed = false;
                        
                        mj_step(sim->m_, sim->d_);
                    } else {
                        // Step while sim lags behind cpu
                        double refreshTime = simRefreshFraction / sim->refresh_rate;
                        mjtNum prevSim = sim->d_->time;
                        
                        while (std::chrono::duration<double>((sim->d_->time - syncSim) * slowdown) < Clock::now() - syncCPU &&
                               Clock::now() - startCPU < std::chrono::duration<double>(refreshTime)) {
                            mj_step(sim->m_, sim->d_);
                            
                            if (sim->d_->time < prevSim) break;
                        }
                    }
                    
                    sim->AddToHistory();
                } else {
                    // Paused - just forward kinematics
                    mj_forward(sim->m_, sim->d_);
                    sim->speed_changed = true;
                }
            }
        }
    }
    
    // Cleanup - delete model and data since we created them
    mj_deleteData(d);
    mj_deleteModel(m);
    
    std::cout << "[Physics Thread] Stopped" << std::endl;
}

// ============================================================================
// Main Function
// ============================================================================
int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "  Quadrotor Interactive GUI - MPC" << std::endl;
    std::cout << "========================================" << std::endl;

    // Paths
    std::string packagePath = quadrotor_online::getPath();
    std::string taskFile = packagePath + "/config/task.info";
    std::string libFolder = packagePath + "/auto_generated";
    std::string modelPath = packagePath + "/robots/quadrotor.xml";

    // ========================================================================
    // Step 1: Create Quadrotor interface
    // ========================================================================
    std::cout << "\n[1] Creating Quadrotor interface..." << std::endl;
    quadrotorInterface = std::make_unique<QuadrotorInterface>(taskFile, libFolder, false);

    vector_t initialState = quadrotorInterface->getInitialState();
    vector_t targetState = quadrotorInterface->getInitialTarget();
    quadrotorParams = quadrotorInterface->getQuadrotorParameters();

    // Define targets
    targets.resize(4);
    targets[0] = targetState;
    
    // Initialize each target vector with proper size before assigning
    targets[1] = vector_t::Zero(STATE_DIM);
    targets[1] << 1.0, 0.0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    
    targets[2] = vector_t::Zero(STATE_DIM);
    targets[2] << 0.0, 1.0, 2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    
    targets[3] = vector_t::Zero(STATE_DIM);
    targets[3] << -0.5, -0.5, 0.8, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    
    // Initialize dynamic target
    dynamicTarget = targets[0];

    // ========================================================================
    // Step 2: Initialize Xbox Controller
    // ========================================================================
    std::cout << "\n[2] Initializing Xbox Controller..." << std::endl;
    xboxController = std::make_unique<XboxController>();
    if (xboxController->isConnected()) {
        std::cout << "[SUCCESS] Xbox controller connected!" << std::endl;
    } else {
        std::cout << "[INFO] No Xbox controller found. Using UI/keyboard only." << std::endl;
    }

    // ========================================================================
    // Step 2b: Initialize Keyboard Controller
    // ========================================================================
    std::cout << "\n[2b] Initializing Keyboard Controller..." << std::endl;
    keyboardController = std::make_unique<KeyboardController>();
    if (keyboardController->isConnected()) {
        std::cout << "[SUCCESS] Keyboard controller initialized!" << std::endl;
    } else {
        std::cout << "[INFO] Keyboard controller not available." << std::endl;
    }

    // ========================================================================
    // Step 3: Setup MPC
    // ========================================================================
    std::cout << "\n[3] Creating MPC controller..." << std::endl;
    
    auto mpcSettings = quadrotorInterface->mpcSettings();
    auto ddpSettings = quadrotorInterface->ddpSettings();
    
    auto mpc = std::make_unique<GaussNewtonDDP_MPC>(
        mpcSettings,
        ddpSettings,
        quadrotorInterface->getRollout(),
        quadrotorInterface->getOptimalControlProblem(),
        quadrotorInterface->getInitializer());
    
    mpc->getSolverPtr()->setReferenceManager(quadrotorInterface->getReferenceManagerPtr());
    
    // Set initial target
    TargetTrajectories targetTrajectories;
    targetTrajectories.timeTrajectory = {0.0};
    targetTrajectories.stateTrajectory = {targets[0]};
    targetTrajectories.inputTrajectory = {vector_t::Zero(INPUT_DIM)};
    mpc->getSolverPtr()->getReferenceManager().setTargetTrajectories(targetTrajectories);

    mpcMrtInterface = std::make_unique<MPC_MRT_Interface>(*mpc);
    mpcMrtInterface->initRollout(&quadrotorInterface->getRollout());

    // ========================================================================
    // Step 4: Initialize MuJoCo GUI
    // ========================================================================
    std::cout << "\n[4] Initializing MuJoCo GUI..." << std::endl;
    
    // Create camera, option, perturb - these must persist for entire program
    mjvCamera cam;
    mjvOption opt;
    mjvPerturb pert;
    
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);
    
    // Set initial camera
    cam.azimuth = 135;
    cam.elevation = -25;
    cam.distance = 6.0;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 1.0;

    // Create simulate object (GlfwAdapter will initialize GLFW internally)
    auto sim = std::make_unique<mujoco::Simulate>(
        std::make_unique<mujoco::GlfwAdapter>(),
        &cam, &opt, &pert,
        false  // not passive
    );

    std::cout << "[SUCCESS] MuJoCo GUI initialized!" << std::endl;

    // ========================================================================
    // Step 5: Start Physics thread (will load model) and MPC thread
    // ========================================================================
    std::cout << "\n[5] Starting threads..." << std::endl;
    
    // Physics thread will load the model after RenderLoop starts
    std::thread physicsWorker(physicsThread, sim.get(), modelPath, initialState);
    std::thread mpcWorker(mpcThread, sim.get());

    // ========================================================================
    // Step 6: Run GUI render loop (must be on main thread)
    // ========================================================================
    std::cout << "\n[6] Starting GUI render loop..." << std::endl;
    std::cout << "\n========================================" << std::endl;
    std::cout << "  GUI is now running!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Keyboard/Mouse Controls:" << std::endl;
    std::cout << "    Space - Play/Pause" << std::endl;
    std::cout << "    Mouse - Rotate/Zoom/Pan" << std::endl;
    std::cout << "    Ctrl+Mouse - Apply force/torque" << std::endl;
    std::cout << "    Tab - Toggle UI panels" << std::endl;
    std::cout << "    ESC - Exit" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Xbox Controller:" << std::endl;
    std::cout << "    Left Stick  - Move X/Y target" << std::endl;
    std::cout << "    Right Stick Y - Move Z target" << std::endl;
    std::cout << "    Right Stick X - Yaw target" << std::endl;
    std::cout << "    A/B/X/Y - Select preset target 1/2/3/4" << std::endl;
    std::cout << "    Start - Toggle Play/Pause" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  MPC Targets:" << std::endl;
    std::cout << "    1: [" << targets[0](0) << ", " << targets[0](1) << ", " << targets[0](2) << "]" << std::endl;
    std::cout << "    2: [" << targets[1](0) << ", " << targets[1](1) << ", " << targets[1](2) << "]" << std::endl;
    std::cout << "    3: [" << targets[2](0) << ", " << targets[2](1) << ", " << targets[2](2) << "]" << std::endl;
    std::cout << "    4: [" << targets[3](0) << ", " << targets[3](1) << ", " << targets[3](2) << "]" << std::endl;
    std::cout << "========================================\n" << std::endl;

    sim->RenderLoop();

    // ========================================================================
    // Cleanup
    // ========================================================================
    std::cout << "\n[7] Cleaning up..." << std::endl;
    exitRequest = true;
    sim->exitrequest.store(1);
    
    if (physicsWorker.joinable()) {
        physicsWorker.join();
    }
    if (mpcWorker.joinable()) {
        mpcWorker.join();
    }
    
    // Clean up Xbox controller
    xboxController.reset();
    
    // Clean up Keyboard controller
    keyboardController.reset();

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Simulation Complete!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}

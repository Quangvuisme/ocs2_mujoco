/**
 * @file main_pinocchio_gui.cpp
 * @brief MuJoCo GUI with continuous Pinocchio calculations printing
 * 
 * This program provides:
 * - Full MuJoCo GUI with UI panels
 * - Real-time PD control for standing
 * - Continuous printing of Pinocchio calculations:
 *   - Forward Kinematics (foot positions)
 *   - Center of Mass
 *   - Jacobians
 *   - Mass matrix
 *   - Gravity compensation torques
 *   - Coriolis forces
 * 
 * IMPORTANT: Convention differences between MuJoCo and Pinocchio:
 * - MuJoCo quaternion: (w, x, y, z)
 * - Pinocchio quaternion: (x, y, z, w)
 * - Joint ordering may differ between URDF and MuJoCo XML
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
#include <map>

#include <anymal_pinocchio/AnymalPinocchioDefinition.h>
#include <anymal_pinocchio/package_path.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

#include <mujoco/mujoco.h>
#include "mujoco_gui/glfw_adapter.h"
#include "mujoco_gui/simulate.h"

using namespace anymal_pinocchio;
using namespace ocs2;

// ============================================================================
// Global Variables
// ============================================================================
std::atomic<bool> exitRequest{false};
std::atomic<bool> controlThreadRunning{false};

// Pinocchio interface
std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
AnymalParameters anymalParams;

// Target management
std::vector<Eigen::Matrix<double, 12, 1>> targetJointPositions;
std::atomic<int> currentTargetIndex{0};
std::mutex controlMutex;

// Control storage for physics thread
Eigen::Matrix<double, 12, 1> g_currentTorques;
std::mutex g_torqueMutex;

// Pinocchio print control
std::atomic<int> printCounter{0};
constexpr int PRINT_INTERVAL = 500;  // Print every N control cycles (0.5s at 1kHz)

// Joint mapping from MuJoCo to Pinocchio
// MuJoCo order: LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE
// Pinocchio order: depends on URDF parsing - need to check!
std::vector<int> mujoco_to_pinocchio_joint_map;
std::vector<std::string> pinocchio_joint_names;

// MuJoCo foot site names and positions for comparison
std::vector<std::string> MUJOCO_FOOT_GEOM_NAMES = {"LF_SHANK", "RF_SHANK", "LH_SHANK", "RH_SHANK"};

// ============================================================================
// Helper Functions
// ============================================================================
void quaternionToEuler(double qw, double qx, double qy, double qz,
                       double& roll, double& pitch, double& yaw) {
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);
    
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void printSeparator(const std::string& title) {
    std::cout << "\n============ " << title << " ============" << std::endl;
}

// ============================================================================
// Pinocchio Calculations and Printing (with MuJoCo comparison)
// ============================================================================
void computeAndPrintPinocchio(const Eigen::VectorXd& q_pin, 
                               const Eigen::VectorXd& v_pin,
                               double simTime,
                               mujoco::Simulate* sim) {
    if (!pinocchioInterfacePtr) return;
    
    const auto& model = pinocchioInterfacePtr->getModel();
    auto& data = pinocchioInterfacePtr->getData();
    
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "  PINOCCHIO CALCULATIONS @ t = " << std::fixed 
              << std::setprecision(3) << simTime << " s" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    // ================================================================
    // 0. Print Joint Ordering (first time or periodically)
    // ================================================================
    static bool printedJointOrder = false;
    if (!printedJointOrder) {
        printSeparator("0. JOINT ORDERING (Pinocchio)");
        std::cout << "  Pinocchio model joints:" << std::endl;
        for (int i = 1; i < model.njoints; ++i) {  // skip universe joint
            std::cout << "    Joint " << (i-1) << ": " << model.names[i] 
                      << " (idx_q=" << model.idx_qs[i] << ", idx_v=" << model.idx_vs[i] << ")" << std::endl;
        }
        std::cout << "\n  MuJoCo joint order (expected):" << std::endl;
        std::cout << "    0-2:  LF_HAA, LF_HFE, LF_KFE" << std::endl;
        std::cout << "    3-5:  RF_HAA, RF_HFE, RF_KFE" << std::endl;
        std::cout << "    6-8:  LH_HAA, LH_HFE, LH_KFE" << std::endl;
        std::cout << "    9-11: RH_HAA, RH_HFE, RH_KFE" << std::endl;
        printedJointOrder = true;
    }
    
    // ================================================================
    // 1. Configuration State
    // ================================================================
    printSeparator("1. CONFIGURATION");
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Base Position:    [" << q_pin.head<3>().transpose() << "]" << std::endl;
    std::cout << "  Base Quaternion:  [" << q_pin.segment<4>(3).transpose() << "] (x,y,z,w)" << std::endl;
    std::cout << "  Joint Positions (Pinocchio order):" << std::endl;
    std::cout << "    q[7-18]: [" << q_pin.tail<12>().transpose() << "]" << std::endl;
    
    // Also print MuJoCo raw state for comparison
    if (sim && sim->d_) {
        std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
        std::cout << "\n  MuJoCo Raw State:" << std::endl;
        std::cout << "    qpos[0-2] (pos):  [" << sim->d_->qpos[0] << ", " 
                  << sim->d_->qpos[1] << ", " << sim->d_->qpos[2] << "]" << std::endl;
        std::cout << "    qpos[3-6] (quat): [" << sim->d_->qpos[3] << ", " 
                  << sim->d_->qpos[4] << ", " << sim->d_->qpos[5] << ", " 
                  << sim->d_->qpos[6] << "] (w,x,y,z)" << std::endl;
        std::cout << "    qpos[7-18] (joints): [";
        for (int i = 0; i < 12; ++i) {
            std::cout << sim->d_->qpos[7+i] << (i < 11 ? ", " : "");
        }
        std::cout << "]" << std::endl;
    }
    
    // ================================================================
    // 2. Forward Kinematics - Compare with MuJoCo
    // ================================================================
    printSeparator("2. FORWARD KINEMATICS COMPARISON");
    pinocchio::forwardKinematics(model, data, q_pin);
    pinocchio::updateFramePlacements(model, data);
    
    // Print base frame pose from Pinocchio
    std::cout << "  Pinocchio Base Frame (oMi[1]):" << std::endl;
    std::cout << "    Translation: " << data.oMi[1].translation().transpose() << std::endl;
    std::cout << "    Rotation:\n" << data.oMi[1].rotation() << std::endl;
    
    std::cout << "\n  Pinocchio Foot Positions (World Frame):" << std::endl;
    for (const auto& footName : FOOT_FRAMES) {
        if (model.existFrame(footName)) {
            auto frameId = model.getFrameId(footName);
            const auto& footPose = data.oMf[frameId];
            std::cout << "    " << std::setw(8) << footName << ": [" 
                      << std::setw(9) << footPose.translation()(0) << ", "
                      << std::setw(9) << footPose.translation()(1) << ", "
                      << std::setw(9) << footPose.translation()(2) << "]" << std::endl;
        }
    }
    
    // Get MuJoCo foot positions for comparison
    if (sim && sim->m_ && sim->d_) {
        std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
        std::cout << "\n  MuJoCo Foot Positions (from geom/body):" << std::endl;
        
        // Get body positions
        const char* foot_bodies[] = {"LF_SHANK", "RF_SHANK", "LH_SHANK", "RH_SHANK"};
        for (int i = 0; i < 4; ++i) {
            int bodyId = mj_name2id(sim->m_, mjOBJ_BODY, foot_bodies[i]);
            if (bodyId >= 0) {
                // Get body position in world frame
                double* xpos = sim->d_->xpos + 3 * bodyId;
                std::cout << "    " << std::setw(8) << foot_bodies[i] << ": [" 
                          << std::setw(9) << xpos[0] << ", "
                          << std::setw(9) << xpos[1] << ", "
                          << std::setw(9) << xpos[2] << "]" << std::endl;
            }
        }
        
        // Also print the actual foot geom positions (collision spheres)
        std::cout << "\n  MuJoCo Foot Geom (collision sphere) positions:" << std::endl;
        for (int i = 0; i < sim->m_->ngeom; ++i) {
            const char* geomName = mj_id2name(sim->m_, mjOBJ_GEOM, i);
            if (geomName && strstr(geomName, "foot")) {
                double* geomPos = sim->d_->geom_xpos + 3 * i;
                std::cout << "    " << std::setw(15) << geomName << ": [" 
                          << std::setw(9) << geomPos[0] << ", "
                          << std::setw(9) << geomPos[1] << ", "
                          << std::setw(9) << geomPos[2] << "]" << std::endl;
            }
        }
    }
    
    // ================================================================
    // 3. Center of Mass
    // ================================================================
    printSeparator("3. CENTER OF MASS");
    Eigen::Vector3d com = pinocchio::centerOfMass(model, data, q_pin);
    std::cout << "  Pinocchio CoM: [" << com.transpose() << "]" << std::endl;
    std::cout << "  Total Mass:    " << data.mass[0] << " kg" << std::endl;
    
    // MuJoCo CoM
    if (sim && sim->m_ && sim->d_) {
        std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
        // MuJoCo computes subtree CoM - we need to get the root body's
        int rootBodyId = mj_name2id(sim->m_, mjOBJ_BODY, "base");
        if (rootBodyId >= 0) {
            double* subtree_com = sim->d_->subtree_com + 3 * rootBodyId;
            std::cout << "  MuJoCo CoM:    [" << subtree_com[0] << ", " 
                      << subtree_com[1] << ", " << subtree_com[2] << "]" << std::endl;
        }
    }
    
    // ================================================================
    // 4. Jacobians (LF_FOOT as example)
    // ================================================================
    printSeparator("4. JACOBIAN (LF_FOOT)");
    std::string footName = "LF_FOOT";
    if (model.existFrame(footName)) {
        auto frameId = model.getFrameId(footName);
        
        Eigen::MatrixXd J(6, model.nv);
        J.setZero();
        pinocchio::computeFrameJacobian(model, data, q_pin, frameId, 
                                        pinocchio::LOCAL_WORLD_ALIGNED, J);
        
        std::cout << "  Shape: " << J.rows() << " x " << J.cols() << std::endl;
        std::cout << "  J_linear row 0 (dx/d...): " << J.row(0) << std::endl;
        std::cout << "  J_linear row 1 (dy/d...): " << J.row(1) << std::endl;
        std::cout << "  J_linear row 2 (dz/d...): " << J.row(2) << std::endl;
    }
    
    // ================================================================
    // 5. Mass Matrix
    // ================================================================
    printSeparator("5. MASS MATRIX");
    pinocchio::crba(model, data, q_pin);
    Eigen::MatrixXd M = data.M;
    M.triangularView<Eigen::StrictlyLower>() = 
        M.transpose().triangularView<Eigen::StrictlyLower>();
    
    std::cout << "  Shape: " << M.rows() << " x " << M.cols() << std::endl;
    std::cout << "  M(0,0) Base mass:        " << M(0,0) << " kg" << std::endl;
    std::cout << "  M(3,3) Base rot inertia: " << M(3,3) << " kg.m^2" << std::endl;
    
    // ================================================================
    // 6. Gravity Compensation
    // ================================================================
    printSeparator("6. GRAVITY COMPENSATION (g)");
    Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, data, q_pin);
    std::cout << "  Base forces:  [" << g.head<6>().transpose() << "]" << std::endl;
    std::cout << "  Joint torques (Pinocchio order):" << std::endl;
    for (int i = 0; i < 12; ++i) {
        std::cout << "    g[" << (6+i) << "]: " << std::setw(10) << g(6+i) << std::endl;
    }
    
    // ================================================================
    // 7. Inverse Dynamics (RNEA)
    // ================================================================
    printSeparator("7. INVERSE DYNAMICS (RNEA)");
    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q_pin, v_pin, a_zero);
    
    std::cout << "  Required torques (with current velocity):" << std::endl;
    std::cout << "  Base wrench: [" << tau.head<6>().transpose() << "]" << std::endl;
    std::cout << "  Joint torques: [" << tau.tail<12>().transpose() << "]" << std::endl;
    
    // ================================================================
    // 8. Coriolis Matrix (C*v)
    // ================================================================
    printSeparator("8. CORIOLIS FORCES (C*v)");
    Eigen::VectorXd c = pinocchio::nonLinearEffects(model, data, q_pin, v_pin) - g;
    std::cout << "  Coriolis + centrifugal forces:" << std::endl;
    std::cout << "  Base:   [" << c.head<6>().transpose() << "]" << std::endl;
    std::cout << "  Joints: [" << c.tail<12>().transpose() << "]" << std::endl;
    
    std::cout << std::string(70, '=') << "\n" << std::endl;
}

// ============================================================================
// Control Thread Function - Computes PD control and Pinocchio
// ============================================================================
void controlThread(mujoco::Simulate* sim) {
    std::cout << "[Control Thread] Started" << std::endl;
    
    const double controlDt = 0.001;  // Control rate (1000 Hz)
    
    g_currentTorques.setZero();
    
    // Target joint position
    Eigen::Matrix<double, 12, 1> targetJointPos = DEFAULT_JOINT_ANGLES;
    Eigen::Matrix<double, 12, 1> targetJointVel = Eigen::Matrix<double, 12, 1>::Zero();
    
    controlThreadRunning = true;
    
    while (!exitRequest) {
        // ================================================================
        // Handle UI target change
        // ================================================================
        if (sim->target_changed.exchange(false)) {
            int newTargetIdx = sim->target_index;
            if (newTargetIdx >= 0 && newTargetIdx < static_cast<int>(targetJointPositions.size())) {
                targetJointPos = targetJointPositions[newTargetIdx];
                currentTargetIndex.store(newTargetIdx);
                std::cout << "[UI] Target updated to " << (newTargetIdx + 1) << std::endl;
            }
        }
        
        // ================================================================
        // Compute PD control and Pinocchio
        // ================================================================
        if (sim->m_ && sim->d_) {
            Eigen::Matrix<double, 12, 1> currentJointPos;
            Eigen::Matrix<double, 12, 1> currentJointVel;
            
            // Pinocchio configuration
            Eigen::VectorXd q_pin(19);  // 7 base (pos + quat) + 12 joints
            Eigen::VectorXd v_pin(18);  // 6 base + 12 joints
            double simTime = 0.0;
            
            {
                std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
                
                // Read state from MuJoCo
                for (int i = 0; i < 12; i++) {
                    currentJointPos(i) = sim->d_->qpos[7 + i];
                    currentJointVel(i) = sim->d_->qvel[6 + i];
                }
                
                // Build Pinocchio configuration
                // Base position
                q_pin(0) = sim->d_->qpos[0];  // x
                q_pin(1) = sim->d_->qpos[1];  // y
                q_pin(2) = sim->d_->qpos[2];  // z
                
                // Base quaternion (MuJoCo: w,x,y,z -> Pinocchio: x,y,z,w)
                q_pin(3) = sim->d_->qpos[4];  // qx
                q_pin(4) = sim->d_->qpos[5];  // qy
                q_pin(5) = sim->d_->qpos[6];  // qz
                q_pin(6) = sim->d_->qpos[3];  // qw
                
                // Joint positions - MAPPING FROM MUJOCO TO PINOCCHIO ORDER
                // MuJoCo order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
                // Pinocchio order: LF(0-2), LH(3-5), RF(6-8), RH(9-11)
                // MuJoCo[i] -> Pinocchio q[7 + mujoco_to_pinocchio[i]]
                // 
                // Pinocchio q indices: LF_HAA=7, LF_HFE=8, LF_KFE=9
                //                      LH_HAA=10, LH_HFE=11, LH_KFE=12
                //                      RF_HAA=13, RF_HFE=14, RF_KFE=15
                //                      RH_HAA=16, RH_HFE=17, RH_KFE=18
                //
                // Mapping: MuJoCo joint i (qpos[7+i]) -> Pinocchio q[pin_idx]
                static const int mujoco_to_pinocchio_q[12] = {
                    7, 8, 9,      // MuJoCo LF (0,1,2) -> Pinocchio q[7,8,9] (LF)
                    13, 14, 15,   // MuJoCo RF (3,4,5) -> Pinocchio q[13,14,15] (RF)
                    10, 11, 12,   // MuJoCo LH (6,7,8) -> Pinocchio q[10,11,12] (LH)
                    16, 17, 18    // MuJoCo RH (9,10,11) -> Pinocchio q[16,17,18] (RH)
                };
                
                for (int i = 0; i < 12; i++) {
                    q_pin(mujoco_to_pinocchio_q[i]) = sim->d_->qpos[7 + i];
                }
                
                // Build Pinocchio velocity
                // Base linear velocity
                v_pin(0) = sim->d_->qvel[0];
                v_pin(1) = sim->d_->qvel[1];
                v_pin(2) = sim->d_->qvel[2];
                
                // Base angular velocity
                v_pin(3) = sim->d_->qvel[3];
                v_pin(4) = sim->d_->qvel[4];
                v_pin(5) = sim->d_->qvel[5];
                
                // Joint velocities - same mapping
                // Pinocchio v indices: LF=6,7,8, LH=9,10,11, RF=12,13,14, RH=15,16,17
                static const int mujoco_to_pinocchio_v[12] = {
                    6, 7, 8,      // MuJoCo LF (0,1,2) -> Pinocchio v[6,7,8] (LF)
                    12, 13, 14,   // MuJoCo RF (3,4,5) -> Pinocchio v[12,13,14] (RF)
                    9, 10, 11,    // MuJoCo LH (6,7,8) -> Pinocchio v[9,10,11] (LH)
                    15, 16, 17    // MuJoCo RH (9,10,11) -> Pinocchio v[15,16,17] (RH)
                };
                
                for (int i = 0; i < 12; i++) {
                    v_pin(mujoco_to_pinocchio_v[i]) = sim->d_->qvel[6 + i];
                }
                
                simTime = sim->d_->time;
            }
            
            // Compute PD torques
            Eigen::Matrix<double, 12, 1> torques;
            torques.setZero();
            
            for (int leg = 0; leg < 4; leg++) {
                int baseIdx = leg * 3;
                
                // HAA joint
                double posErr = targetJointPos(baseIdx) - currentJointPos(baseIdx);
                double velErr = targetJointVel(baseIdx) - currentJointVel(baseIdx);
                torques(baseIdx) = anymalParams.kp_haa * posErr + anymalParams.kd_haa * velErr;
                torques(baseIdx) = std::clamp(torques(baseIdx), -anymalParams.maxTorque, anymalParams.maxTorque);
                
                // HFE joint
                posErr = targetJointPos(baseIdx + 1) - currentJointPos(baseIdx + 1);
                velErr = targetJointVel(baseIdx + 1) - currentJointVel(baseIdx + 1);
                torques(baseIdx + 1) = anymalParams.kp_hfe * posErr + anymalParams.kd_hfe * velErr;
                torques(baseIdx + 1) = std::clamp(torques(baseIdx + 1), -anymalParams.maxTorque, anymalParams.maxTorque);
                
                // KFE joint
                posErr = targetJointPos(baseIdx + 2) - currentJointPos(baseIdx + 2);
                velErr = targetJointVel(baseIdx + 2) - currentJointVel(baseIdx + 2);
                torques(baseIdx + 2) = anymalParams.kp_kfe * posErr + anymalParams.kd_kfe * velErr;
                torques(baseIdx + 2) = std::clamp(torques(baseIdx + 2), -anymalParams.maxTorque, anymalParams.maxTorque);
            }
            
            // Store torques for physics thread
            {
                std::lock_guard<std::mutex> lock(g_torqueMutex);
                g_currentTorques = torques;
            }
            
            // Print Pinocchio calculations periodically
            if (sim->run && ++printCounter >= PRINT_INTERVAL) {
                printCounter = 0;
                computeAndPrintPinocchio(q_pin, v_pin, simTime, sim);
            }
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(controlDt * 1e6)));
    }
    
    controlThreadRunning = false;
    std::cout << "[Control Thread] Stopped" << std::endl;
}

// ============================================================================
// Physics Thread Function
// ============================================================================
void physicsThread(mujoco::Simulate* sim, const std::string& modelPath) {
    std::cout << "[Physics Thread] Started" << std::endl;
    
    // Load model
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
    
    // Set initial joint positions
    std::cout << "[Physics Thread] Setting initial pose..." << std::endl;
    
    // Base position and orientation
    d->qpos[0] = 0.0;   // x
    d->qpos[1] = 0.0;   // y
    d->qpos[2] = 0.62;  // z (standing height)
    d->qpos[3] = 1.0;   // qw
    d->qpos[4] = 0.0;   // qx
    d->qpos[5] = 0.0;   // qy
    d->qpos[6] = 0.0;   // qz
    
    // Set joint angles (standing pose)
    for (int i = 0; i < 12; i++) {
        d->qpos[7 + i] = DEFAULT_JOINT_ANGLES(i);
    }
    
    mj_forward(m, d);
    
    // Load into simulate object
    sim->Load(m, d, modelPath.c_str());
    std::cout << "[Physics Thread] Model loaded successfully!" << std::endl;
    
    using Clock = std::chrono::steady_clock;
    auto syncCPU = Clock::now();
    mjtNum syncSim = 0;
    
    const double syncMisalign = 0.1;
    const double simRefreshFraction = 0.7;
    
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
                
                // Apply joint torques
                {
                    std::lock_guard<std::mutex> torqueLock(g_torqueMutex);
                    for (int i = 0; i < 12; i++) {
                        sim->d_->ctrl[i] = g_currentTorques(i);
                    }
                }
                
                // Step physics
                if (sim->run) {
                    auto startCPU = Clock::now();
                    auto elapsedCPU = startCPU - syncCPU;
                    double elapsedSim = sim->d_->time - syncSim;
                    
                    double slowdown = 100.0 / sim->percentRealTime[sim->real_time_index];
                    
                    bool misaligned = std::abs(std::chrono::duration<double>(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;
                    
                    if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
                        syncCPU.time_since_epoch().count() == 0 || misaligned || sim->speed_changed) {
                        syncCPU = startCPU;
                        syncSim = sim->d_->time;
                        sim->speed_changed = false;
                        
                        mj_step(sim->m_, sim->d_);
                    } else {
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

// ============================================================================
// Main Function
// ============================================================================
int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "  ANYmal Pinocchio GUI - Continuous Print" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Comparing MuJoCo vs Pinocchio FK" << std::endl;
    std::cout << "========================================" << std::endl;

    // ========================================================================
    // Find workspace path first
    // ========================================================================
    std::string packagePath = anymal_pinocchio::getPath();
    std::string workspacePath = packagePath;
    size_t srcPos = workspacePath.find("/src/");
    if (srcPos != std::string::npos) {
        workspacePath = workspacePath.substr(0, srcPos);
    }
    
    // ========================================================================
    // Step 1: Load Pinocchio model from LOCAL URDF (same as MuJoCo uses)
    // ========================================================================
    std::cout << "\n[1] Loading Pinocchio model from local URDF..." << std::endl;
    
    // Use the URDF from the workspace robots folder
    std::string urdfPath = workspacePath + "/robots/anymal_c/anymal_c_urdf/urdf/anymal.urdf";
    std::cout << "  URDF Path: " << urdfPath << std::endl;
    
    // Check if URDF exists
    std::ifstream urdfFile(urdfPath);
    if (!urdfFile.good()) {
        std::cerr << "[ERROR] URDF not found at: " << urdfPath << std::endl;
        return 1;
    }
    urdfFile.close();
    
    try {
        auto pinocchioInterface = getPinocchioInterfaceFromUrdfFile(
            urdfPath,
            pinocchio::JointModelFreeFlyer()
        );
        pinocchioInterfacePtr = std::make_unique<PinocchioInterface>(std::move(pinocchioInterface));
        
        const auto& model = pinocchioInterfacePtr->getModel();
        std::cout << "  Model name: " << model.name << std::endl;
        std::cout << "  nq (config dim): " << model.nq << std::endl;
        std::cout << "  nv (velocity dim): " << model.nv << std::endl;
        std::cout << "  njoints: " << model.njoints << std::endl;
        std::cout << "  nframes: " << model.nframes << std::endl;
        
        // Print all joints to understand ordering
        std::cout << "\n  Pinocchio Joint Order:" << std::endl;
        for (int i = 1; i < model.njoints; ++i) {  // skip universe
            std::cout << "    [" << (i-1) << "] " << model.names[i] 
                      << " (idx_q=" << model.idx_qs[i] << ", idx_v=" << model.idx_vs[i] 
                      << ", nq=" << model.nqs[i] << ", nv=" << model.nvs[i] << ")" << std::endl;
        }
        
        // Print foot frames
        std::cout << "\n  Foot Frames in Pinocchio:" << std::endl;
        for (const auto& footName : FOOT_FRAMES) {
            if (model.existFrame(footName)) {
                auto frameId = model.getFrameId(footName);
                std::cout << "    " << footName << " -> frameId=" << frameId 
                          << ", parentJoint=" << model.frames[frameId].parentJoint << std::endl;
            } else {
                std::cout << "    " << footName << " -> NOT FOUND!" << std::endl;
            }
        }
        
        std::cout << "\n[SUCCESS] Pinocchio model loaded!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load Pinocchio model: " << e.what() << std::endl;
        return 1;
    }

    // ========================================================================
    // Step 2: Find MuJoCo model
    // ========================================================================
    std::cout << "\n[2] Finding MuJoCo model..." << std::endl;
    
    std::string modelPath = workspacePath + "/robots/anymal_c/scene_anymal_c_real.xml";
    
    std::ifstream modelFile(modelPath);
    if (!modelFile.good()) {
        std::cerr << "[ERROR] Model not found at: " << modelPath << std::endl;
        return 1;
    }
    modelFile.close();
    
    std::cout << "  Model path: " << modelPath << std::endl;

    // ========================================================================
    // Step 3: Define target poses
    // ========================================================================
    std::cout << "\n[3] Defining target poses..." << std::endl;
    
    targetJointPositions.resize(4);
    
    // Target 1: Default standing
    targetJointPositions[0] = DEFAULT_JOINT_ANGLES;
    
    // Target 2: Wider stance
    targetJointPositions[1] = DEFAULT_JOINT_ANGLES;
    targetJointPositions[1](0) = -0.35;
    targetJointPositions[1](3) = 0.35;
    targetJointPositions[1](6) = -0.35;
    targetJointPositions[1](9) = 0.35;
    
    // Target 3: Lower crouch
    targetJointPositions[2] = DEFAULT_JOINT_ANGLES;
    targetJointPositions[2](1) = 0.8;
    targetJointPositions[2](2) = -1.2;
    targetJointPositions[2](4) = 0.8;
    targetJointPositions[2](5) = -1.2;
    targetJointPositions[2](7) = -0.8;
    targetJointPositions[2](8) = 1.2;
    targetJointPositions[2](10) = -0.8;
    targetJointPositions[2](11) = 1.2;
    
    // Target 4: Higher stance
    targetJointPositions[3] = DEFAULT_JOINT_ANGLES;
    targetJointPositions[3](1) = 0.4;
    targetJointPositions[3](2) = -0.6;
    targetJointPositions[3](4) = 0.4;
    targetJointPositions[3](5) = -0.6;
    targetJointPositions[3](7) = -0.4;
    targetJointPositions[3](8) = 0.6;
    targetJointPositions[3](10) = -0.4;
    targetJointPositions[3](11) = 0.6;
    
    std::cout << "  4 target poses defined" << std::endl;

    // ========================================================================
    // Step 4: Initialize MuJoCo GUI
    // ========================================================================
    std::cout << "\n[4] Initializing MuJoCo GUI..." << std::endl;
    
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
        &cam, &opt, &pert,
        false
    );

    std::cout << "[SUCCESS] MuJoCo GUI initialized!" << std::endl;

    // ========================================================================
    // Step 5: Start threads
    // ========================================================================
    std::cout << "\n[5] Starting threads..." << std::endl;
    
    std::thread physicsWorker(physicsThread, sim.get(), modelPath);
    std::thread controlWorker(controlThread, sim.get());

    // ========================================================================
    // Step 6: Run GUI render loop
    // ========================================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "  GUI is now running!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Pinocchio prints every " << PRINT_INTERVAL << " ms" << std::endl;
    std::cout << "  Calculations printed:" << std::endl;
    std::cout << "    - Forward Kinematics (foot positions)" << std::endl;
    std::cout << "    - Center of Mass" << std::endl;
    std::cout << "    - Jacobians" << std::endl;
    std::cout << "    - Mass Matrix" << std::endl;
    std::cout << "    - Gravity Compensation" << std::endl;
    std::cout << "    - Inverse Dynamics (RNEA)" << std::endl;
    std::cout << "    - Coriolis Forces" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Controls:" << std::endl;
    std::cout << "    Space - Play/Pause" << std::endl;
    std::cout << "    Mouse - Rotate/Zoom/Pan" << std::endl;
    std::cout << "    Tab - Toggle UI panels" << std::endl;
    std::cout << "    ESC - Exit" << std::endl;
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
    if (controlWorker.joinable()) {
        controlWorker.join();
    }
    
    pinocchioInterfacePtr.reset();

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Simulation Complete!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}

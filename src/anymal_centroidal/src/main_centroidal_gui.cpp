/**
 * @file main_centroidal_gui.cpp
 * @brief ANYmal MuJoCo GUI with OCS2 Centroidal Model calculations
 * 
 * This tests the OCS2 Centroidal Model convention WITHOUT running controller.
 * Compares:
 * 1. Direct Pinocchio calculations (FreeFlyer base)
 * 2. OCS2 Centroidal Model calculations (SphericalZYX base)
 * 
 * Key differences:
 * - Pinocchio FreeFlyer: q = [pos(3), quat(4), joints(12)] = 19D
 * - OCS2 Centroidal: q = [pos(3), euler_zyx(3), joints(12)] = 18D
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>

// MuJoCo
#include <mujoco/mujoco.h>
#include "mujoco_gui/glfw_adapter.h"
#include "mujoco_gui/simulate.h"

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/parsers/urdf.hpp>

// OCS2 Centroidal Model
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

// Package
#include <anymal_centroidal/package_path.h>
#include <anymal_centroidal/AnymalDefinition.h>

using namespace anymal_centroidal;

// =============================================================================
// Global Variables
// =============================================================================
std::atomic<bool> exitRequest{false};
std::mutex g_dataMutex;

// Pinocchio models
std::unique_ptr<pinocchio::Model> g_pinModelFreeFlyer;  // Standard Pinocchio
std::unique_ptr<pinocchio::Data> g_pinDataFreeFlyer;

// OCS2 Centroidal model
std::unique_ptr<ocs2::PinocchioInterface> g_ocs2PinInterface;
ocs2::CentroidalModelInfo g_centroidalInfo;
std::unique_ptr<ocs2::CentroidalModelPinocchioMapping> g_centroidalMapping;
std::unique_ptr<ocs2::CentroidalModelRbdConversions> g_rbdConversions;

// Current state from MuJoCo
Eigen::VectorXd g_qPinFreeFlyer;   // 19D: pos(3) + quat(4) + joints(12)
Eigen::VectorXd g_vPinFreeFlyer;   // 18D: lin_vel(3) + ang_vel(3) + joint_vel(12)
Eigen::VectorXd g_qOcs2;           // 18D: pos(3) + euler_zyx(3) + joints(12)
Eigen::VectorXd g_vOcs2;           // 18D: same structure
Eigen::VectorXd g_centroidalState; // 24D: momentum(6) + generalized_coords(18)
double g_simTime = 0.0;

// =============================================================================
// Mapping indices
// MuJoCo joint order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
// OCS2/Pinocchio joint order: LF(0-2), LH(3-5), RF(6-8), RH(9-11)
// =============================================================================
const int MUJOCO_TO_OCS2_JOINT[12] = {
    0, 1, 2,    // LF -> LF
    6, 7, 8,    // RF -> RF (OCS2 idx 6-8)
    3, 4, 5,    // LH -> LH (OCS2 idx 3-5)
    9, 10, 11   // RH -> RH
};

const int OCS2_TO_MUJOCO_JOINT[12] = {
    0, 1, 2,    // LF -> LF
    6, 7, 8,    // LH -> LH (MuJoCo idx 6-8)
    3, 4, 5,    // RF -> RF (MuJoCo idx 3-5)
    9, 10, 11   // RH -> RH
};

// =============================================================================
// Helper: Quaternion (w,x,y,z) to ZYX Euler (yaw, pitch, roll)
// =============================================================================
void quatToEulerZYX(double qw, double qx, double qy, double qz,
                    double& yaw, double& pitch, double& roll) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// =============================================================================
// Build state vectors from MuJoCo
// =============================================================================
void buildStateFromMujoco(const mjData* d) {
    // === 1. Build Pinocchio FreeFlyer state ===
    // q = [x, y, z, qx, qy, qz, qw, joints...]
    g_qPinFreeFlyer.resize(19);
    g_qPinFreeFlyer.head<3>() << d->qpos[0], d->qpos[1], d->qpos[2];
    // MuJoCo quat: (w,x,y,z) -> Pinocchio: (x,y,z,w)
    g_qPinFreeFlyer(3) = d->qpos[4];  // qx
    g_qPinFreeFlyer(4) = d->qpos[5];  // qy
    g_qPinFreeFlyer(5) = d->qpos[6];  // qz
    g_qPinFreeFlyer(6) = d->qpos[3];  // qw
    
    // Joints: MuJoCo order -> Pinocchio FreeFlyer order (alphabetical: LF,LH,RF,RH)
    for (int i = 0; i < 12; i++) {
        int ocs2_idx = MUJOCO_TO_OCS2_JOINT[i];
        g_qPinFreeFlyer(7 + ocs2_idx) = d->qpos[7 + i];
    }
    
    // v = [vx, vy, vz, wx, wy, wz, joint_vels...]
    g_vPinFreeFlyer.resize(18);
    g_vPinFreeFlyer.head<3>() << d->qvel[0], d->qvel[1], d->qvel[2];
    g_vPinFreeFlyer.segment<3>(3) << d->qvel[3], d->qvel[4], d->qvel[5];
    for (int i = 0; i < 12; i++) {
        int ocs2_idx = MUJOCO_TO_OCS2_JOINT[i];
        g_vPinFreeFlyer(6 + ocs2_idx) = d->qvel[6 + i];
    }
    
    // === 2. Build OCS2 Centroidal state ===
    // q = [x, y, z, yaw, pitch, roll, joints...]
    g_qOcs2.resize(18);
    g_qOcs2.head<3>() << d->qpos[0], d->qpos[1], d->qpos[2];
    
    double yaw, pitch, roll;
    quatToEulerZYX(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6], yaw, pitch, roll);
    g_qOcs2(3) = yaw;
    g_qOcs2(4) = pitch;
    g_qOcs2(5) = roll;
    
    // Joints in OCS2 order
    for (int i = 0; i < 12; i++) {
        int ocs2_idx = MUJOCO_TO_OCS2_JOINT[i];
        g_qOcs2(6 + ocs2_idx) = d->qpos[7 + i];
    }
    
    // v = [vx, vy, vz, euler_dot..., joint_vels...]
    // Note: euler_dot is NOT the same as angular velocity!
    // For now, approximate with angular velocity (valid when orientation is small)
    g_vOcs2.resize(18);
    g_vOcs2.head<3>() << d->qvel[0], d->qvel[1], d->qvel[2];
    g_vOcs2.segment<3>(3) << d->qvel[3], d->qvel[4], d->qvel[5];  // Approximation
    for (int i = 0; i < 12; i++) {
        int ocs2_idx = MUJOCO_TO_OCS2_JOINT[i];
        g_vOcs2(6 + ocs2_idx) = d->qvel[6 + i];
    }
}

// =============================================================================
// Print comparison between Pinocchio FreeFlyer and OCS2 Centroidal
// =============================================================================
void printComparison() {
    std::lock_guard<std::mutex> lock(g_dataMutex);
    
    std::cout << "\n======================================================================" << std::endl;
    std::cout << "  CENTROIDAL MODEL TEST @ t = " << std::fixed << std::setprecision(3) << g_simTime << " s" << std::endl;
    std::cout << "======================================================================" << std::endl;
    
    // === 1. Configuration comparison ===
    std::cout << "\n============ 1. CONFIGURATION ============" << std::endl;
    std::cout << "  Pinocchio FreeFlyer q (19D):" << std::endl;
    std::cout << "    pos: [" << g_qPinFreeFlyer.head<3>().transpose() << "]" << std::endl;
    std::cout << "    quat(x,y,z,w): [" << g_qPinFreeFlyer.segment<4>(3).transpose() << "]" << std::endl;
    std::cout << "    joints: [" << g_qPinFreeFlyer.tail<12>().transpose() << "]" << std::endl;
    
    std::cout << "\n  OCS2 Centroidal q (18D):" << std::endl;
    std::cout << "    pos: [" << g_qOcs2.head<3>().transpose() << "]" << std::endl;
    std::cout << "    euler(y,p,r): [" << g_qOcs2.segment<3>(3).transpose() << "]" << std::endl;
    std::cout << "    joints: [" << g_qOcs2.tail<12>().transpose() << "]" << std::endl;
    
    // === 2. Forward Kinematics comparison ===
    std::cout << "\n============ 2. FORWARD KINEMATICS ============" << std::endl;
    
    // Pinocchio FreeFlyer FK
    pinocchio::forwardKinematics(*g_pinModelFreeFlyer, *g_pinDataFreeFlyer, g_qPinFreeFlyer);
    pinocchio::updateFramePlacements(*g_pinModelFreeFlyer, *g_pinDataFreeFlyer);
    
    std::cout << "  Pinocchio FreeFlyer Foot Positions:" << std::endl;
    for (const auto& footName : FOOT_FRAMES) {
        if (g_pinModelFreeFlyer->existFrame(footName)) {
            auto frameId = g_pinModelFreeFlyer->getFrameId(footName);
            const auto& footPos = g_pinDataFreeFlyer->oMf[frameId].translation();
            std::cout << "    " << footName << ": [" << std::setw(8) << footPos.transpose() << "]" << std::endl;
        }
    }
    
    // OCS2 Centroidal FK
    const auto& ocs2Model = g_ocs2PinInterface->getModel();
    auto& ocs2Data = g_ocs2PinInterface->getData();
    pinocchio::forwardKinematics(ocs2Model, ocs2Data, g_qOcs2);
    pinocchio::updateFramePlacements(ocs2Model, ocs2Data);
    
    std::cout << "\n  OCS2 Centroidal Foot Positions:" << std::endl;
    for (size_t i = 0; i < g_centroidalInfo.endEffectorFrameIndices.size(); i++) {
        auto frameId = g_centroidalInfo.endEffectorFrameIndices[i];
        const auto& footPos = ocs2Data.oMf[frameId].translation();
        std::cout << "    " << ocs2Model.frames[frameId].name << ": [" << std::setw(8) << footPos.transpose() << "]" << std::endl;
    }
    
    // === 3. Center of Mass comparison ===
    std::cout << "\n============ 3. CENTER OF MASS ============" << std::endl;
    
    pinocchio::centerOfMass(*g_pinModelFreeFlyer, *g_pinDataFreeFlyer, g_qPinFreeFlyer);
    std::cout << "  Pinocchio FreeFlyer CoM: [" << g_pinDataFreeFlyer->com[0].transpose() << "]" << std::endl;
    
    pinocchio::centerOfMass(ocs2Model, ocs2Data, g_qOcs2);
    std::cout << "  OCS2 Centroidal CoM:     [" << ocs2Data.com[0].transpose() << "]" << std::endl;
    
    // === 4. Centroidal Dynamics ===
    std::cout << "\n============ 4. CENTROIDAL DYNAMICS ============" << std::endl;
    
    // OCS2 centroidal momentum
    ocs2::updateCentroidalDynamics(*g_ocs2PinInterface, g_centroidalInfo, g_qOcs2);
    const auto& Ag = ocs2::getCentroidalMomentumMatrix(*g_ocs2PinInterface);
    Eigen::VectorXd momentum = Ag * g_vOcs2;
    
    std::cout << "  Centroidal Momentum Matrix Ag shape: " << Ag.rows() << "x" << Ag.cols() << std::endl;
    std::cout << "  Centroidal Momentum h = Ag * v:" << std::endl;
    std::cout << "    Linear:  [" << momentum.head<3>().transpose() << "]" << std::endl;
    std::cout << "    Angular: [" << momentum.tail<3>().transpose() << "]" << std::endl;
    
    // Normalized momentum (for centroidal state)
    Eigen::VectorXd normalizedMomentum = momentum / g_centroidalInfo.robotMass;
    std::cout << "  Normalized Momentum (h/m):" << std::endl;
    std::cout << "    Linear:  [" << normalizedMomentum.head<3>().transpose() << "]" << std::endl;
    std::cout << "    Angular: [" << normalizedMomentum.tail<3>().transpose() << "]" << std::endl;
    
    // === 5. Gravity Compensation ===
    std::cout << "\n============ 5. GRAVITY COMPENSATION ============" << std::endl;
    
    // Pinocchio FreeFlyer
    Eigen::VectorXd vZero = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd aZero = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd tauFF = pinocchio::rnea(*g_pinModelFreeFlyer, *g_pinDataFreeFlyer, 
                                             g_qPinFreeFlyer, vZero, aZero);
    std::cout << "  Pinocchio FreeFlyer gravity (RNEA):" << std::endl;
    std::cout << "    Base wrench: [" << tauFF.head<6>().transpose() << "]" << std::endl;
    std::cout << "    Joint torques (OCS2 order): [" << tauFF.tail<12>().transpose() << "]" << std::endl;
    
    // OCS2 Centroidal
    Eigen::VectorXd tauOcs2 = pinocchio::rnea(ocs2Model, ocs2Data, g_qOcs2, vZero, aZero);
    std::cout << "\n  OCS2 Centroidal gravity (RNEA):" << std::endl;
    std::cout << "    Base wrench: [" << tauOcs2.head<6>().transpose() << "]" << std::endl;
    std::cout << "    Joint torques (OCS2 order): [" << tauOcs2.tail<12>().transpose() << "]" << std::endl;
    
    // Convert to MuJoCo order for verification
    std::cout << "\n  Joint torques in MuJoCo order (LF,RF,LH,RH):" << std::endl;
    std::cout << "    ";
    for (int i = 0; i < 12; i++) {
        int ocs2_idx = MUJOCO_TO_OCS2_JOINT[i];
        std::cout << std::setw(8) << std::setprecision(4) << tauOcs2(6 + ocs2_idx) << " ";
    }
    std::cout << std::endl;
    
    // === 6. RBD Conversions Test ===
    std::cout << "\n============ 6. RBD CONVERSIONS ============" << std::endl;
    
    // Build RBD state
    Eigen::VectorXd rbdState(36);
    // [0:3] = euler (yaw, pitch, roll)
    rbdState.head<3>() = g_qOcs2.segment<3>(3);
    // [3:6] = position
    rbdState.segment<3>(3) = g_qOcs2.head<3>();
    // [6:18] = joints
    rbdState.segment<12>(6) = g_qOcs2.tail<12>();
    // [18:21] = angular velocity
    rbdState.segment<3>(18) = g_vOcs2.segment<3>(3);
    // [21:24] = linear velocity
    rbdState.segment<3>(21) = g_vOcs2.head<3>();
    // [24:36] = joint velocities
    rbdState.tail<12>() = g_vOcs2.tail<12>();
    
    std::cout << "  RBD State (36D):" << std::endl;
    std::cout << "    euler[0:3]: [" << rbdState.head<3>().transpose() << "]" << std::endl;
    std::cout << "    pos[3:6]:   [" << rbdState.segment<3>(3).transpose() << "]" << std::endl;
    std::cout << "    joints[6:18]: [" << rbdState.segment<12>(6).transpose() << "]" << std::endl;
    
    // Convert to centroidal state
    g_centroidalState = g_rbdConversions->computeCentroidalStateFromRbdModel(rbdState);
    
    std::cout << "\n  Centroidal State (24D):" << std::endl;
    std::cout << "    momentum[0:6]:  [" << g_centroidalState.head<6>().transpose() << "]" << std::endl;
    std::cout << "    pos[6:9]:       [" << g_centroidalState.segment<3>(6).transpose() << "]" << std::endl;
    std::cout << "    euler[9:12]:    [" << g_centroidalState.segment<3>(9).transpose() << "]" << std::endl;
    std::cout << "    joints[12:24]:  [" << g_centroidalState.tail<12>().transpose() << "]" << std::endl;
    
    std::cout << "======================================================================\n" << std::endl;
}

// =============================================================================
// Control Thread (just reads state, no control)
// =============================================================================
void controlThread(mujoco::Simulate* sim) {
    std::cout << "[Control Thread] Started" << std::endl;
    
    int printCounter = 0;
    const int PRINT_INTERVAL = 500;  // Every 500ms
    
    while (!exitRequest && !sim->exitrequest.load()) {
        if (sim->m_ && sim->d_) {
            {
                std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
                std::lock_guard<std::mutex> dataLock(g_dataMutex);
                
                buildStateFromMujoco(sim->d_);
                g_simTime = sim->d_->time;
            }
            
            // Print comparison periodically
            if (sim->run && ++printCounter >= PRINT_INTERVAL) {
                printCounter = 0;
                printComparison();
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
    d->qpos[2] = 0.57;  // z
    d->qpos[3] = 1.0;   // qw
    d->qpos[4] = 0.0;   // qx
    d->qpos[5] = 0.0;   // qy
    d->qpos[6] = 0.0;   // qz
    
    // Joint positions (MuJoCo order: LF, RF, LH, RH)
    for (int i = 0; i < 12; i++) {
        d->qpos[7 + i] = DEFAULT_JOINT_ANGLES[i];
    }
    
    mj_forward(m, d);
    sim->Load(m, d, modelPath.c_str());
    std::cout << "[Physics Thread] Model loaded!" << std::endl;
    
    using Clock = std::chrono::steady_clock;
    auto syncCPU = Clock::now();
    mjtNum syncSim = 0;
    
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
                
                // Simple PD to hold position (no MPC)
                for (int i = 0; i < 12; i++) {
                    double pos = sim->d_->qpos[7 + i];
                    double vel = sim->d_->qvel[6 + i];
                    double target = DEFAULT_JOINT_ANGLES[i];
                    sim->d_->ctrl[i] = 8000.0 * (target - pos) - 50.0 * vel;
                }
                
                // Step physics
                if (sim->run) {
                    auto startCPU = Clock::now();
                    double slowdown = 100.0 / sim->percentRealTime[sim->real_time_index];
                    double elapsedSim = sim->d_->time - syncSim;
                    
                    if (elapsedSim < 0 || syncCPU.time_since_epoch().count() == 0 || sim->speed_changed) {
                        syncCPU = startCPU;
                        syncSim = sim->d_->time;
                        sim->speed_changed = false;
                    }
                    
                    while (std::chrono::duration<double>((sim->d_->time - syncSim) * slowdown) < 
                           Clock::now() - syncCPU) {
                        mj_step(sim->m_, sim->d_);
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
    std::cout << "  ANYmal Centroidal Model Test" << std::endl;
    std::cout << "  (No Controller - Convention Test)" << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::string basePath = getPath();
    std::string urdfPath = basePath + "/robots/anymal_c/urdf/anymal.urdf";
    std::string mujocoPath = basePath + "/robots/anymal_c/scene.xml";
    
    std::cout << "\n[1] Loading Pinocchio FreeFlyer model..." << std::endl;
    g_pinModelFreeFlyer = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), *g_pinModelFreeFlyer);
    g_pinDataFreeFlyer = std::make_unique<pinocchio::Data>(*g_pinModelFreeFlyer);
    std::cout << "    nq=" << g_pinModelFreeFlyer->nq << ", nv=" << g_pinModelFreeFlyer->nv << std::endl;
    std::cout << "    Total mass: " << pinocchio::computeTotalMass(*g_pinModelFreeFlyer) << " kg" << std::endl;
    
    std::cout << "\n[2] Loading OCS2 Centroidal model..." << std::endl;
    // Create PinocchioInterface with SphericalZYX base (like OCS2 does)
    g_ocs2PinInterface = std::make_unique<ocs2::PinocchioInterface>(
        ocs2::centroidal_model::createPinocchioInterface(urdfPath, JOINT_NAMES_URDF));
    
    // Create CentroidalModelInfo
    Eigen::VectorXd defaultJoints(12);
    // OCS2 order: LF, LH, RF, RH
    defaultJoints << -0.25, 0.60, -0.85,  // LF
                     -0.25, -0.60, 0.85,  // LH
                      0.25, 0.60, -0.85,  // RF
                      0.25, -0.60, 0.85;  // RH
    
    g_centroidalInfo = ocs2::centroidal_model::createCentroidalModelInfo(
        *g_ocs2PinInterface,
        ocs2::CentroidalModelType::SingleRigidBodyDynamics,
        defaultJoints,
        FOOT_FRAMES,  // 3DoF contacts
        {}            // No 6DoF contacts
    );
    
    std::cout << "    stateDim=" << g_centroidalInfo.stateDim << std::endl;
    std::cout << "    inputDim=" << g_centroidalInfo.inputDim << std::endl;
    std::cout << "    robotMass=" << g_centroidalInfo.robotMass << " kg" << std::endl;
    std::cout << "    generalizedCoordinatesNum=" << g_centroidalInfo.generalizedCoordinatesNum << std::endl;
    
    // Print joint order
    const auto& model = g_ocs2PinInterface->getModel();
    std::cout << "\n    OCS2 Pinocchio Joint Order:" << std::endl;
    for (int i = 2; i < model.njoints; i++) {
        std::cout << "      [" << i << "] " << model.names[i] << " (q_idx=" << model.idx_qs[i] << ")" << std::endl;
    }
    
    // Create CentroidalModelPinocchioMapping
    g_centroidalMapping = std::make_unique<ocs2::CentroidalModelPinocchioMapping>(g_centroidalInfo);
    
    // Create RBD conversions
    g_rbdConversions = std::make_unique<ocs2::CentroidalModelRbdConversions>(
        *g_ocs2PinInterface, g_centroidalInfo);
    
    std::cout << "\n[3] Initializing MuJoCo GUI..." << std::endl;
    
    // Camera, option, perturb for visualization
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
    
    std::cout << "\n[4] Starting threads..." << std::endl;
    std::thread physThread(physicsThread, sim.get(), mujocoPath);
    std::thread ctrlThread(controlThread, sim.get());
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Simulation Running!" << std::endl;
    std::cout << "  Watch console for comparison output" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    sim->RenderLoop();
    
    exitRequest = true;
    physThread.join();
    ctrlThread.join();
    
    return 0;
}

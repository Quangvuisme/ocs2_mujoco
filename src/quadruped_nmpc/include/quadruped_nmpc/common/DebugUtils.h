#pragma once

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <mutex>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace quadruped_nmpc {

/**
 * @brief Debug flags to enable/disable specific debug outputs
 */
struct DebugFlags {
    bool enabled = false;              // Master switch for all debug
    bool printFK = false;              // Forward Kinematics (foot positions)
    bool printJacobian = false;        // Jacobian matrices
    bool printContactForce = false;    // Contact forces from MPC
    bool printFootPosition = false;    // Foot positions in world frame
    bool printContactStatus = false;   // Contact status per foot
    bool printJointTarget = false;     // Target joint positions
    bool printJointCurrent = false;    // Current joint positions/velocities
    bool printBaseState = false;       // Base position/orientation
    bool printMpcState = false;        // MPC optimal state
    bool printWbcTorques = false;      // WBC computed torques
    bool printStateValidity = false;   // Check for NaN/Inf values
    bool logToFile = false;            // Log debug to file
    
    double printInterval = 0.5;        // Print interval in seconds
    int printFrequencyMod = 100;       // Print every N control cycles
    
    void enableAll() {
        enabled = true;
        printFK = true;
        printJacobian = true;
        printContactForce = true;
        printFootPosition = true;
        printContactStatus = true;
        printJointTarget = true;
        printJointCurrent = true;
        printBaseState = true;
        printMpcState = true;
        printWbcTorques = true;
        printStateValidity = true;
    }
    
    void disableAll() {
        enabled = false;
        printFK = false;
        printJacobian = false;
        printContactForce = false;
        printFootPosition = false;
        printContactStatus = false;
        printJointTarget = false;
        printJointCurrent = false;
        printBaseState = false;
        printMpcState = false;
        printWbcTorques = false;
        printStateValidity = false;
    }
    
    void setMinimal() {
        enabled = true;
        printFK = false;
        printJacobian = false;
        printContactForce = true;
        printFootPosition = true;
        printContactStatus = true;
        printJointTarget = false;
        printJointCurrent = false;
        printBaseState = true;
        printMpcState = false;
        printWbcTorques = false;
        printStateValidity = true;
    }
};

/**
 * @brief Debug utilities for legged robot control
 */
class DebugUtils {
public:
    DebugUtils() = default;
    
    void setFlags(const DebugFlags& flags) { flags_ = flags; }
    DebugFlags& getFlags() { return flags_; }
    const DebugFlags& getFlags() const { return flags_; }
    
    /**
     * @brief Check if value is valid (not NaN or Inf)
     */
    static bool isValidScalar(double value) {
        return !std::isnan(value) && !std::isfinite(value) ? false : true;
    }
    
    /**
     * @brief Check if vector contains valid values
     */
    static bool isValidVector(const ocs2::vector_t& vec) {
        for (int i = 0; i < vec.size(); ++i) {
            if (std::isnan(vec(i)) || std::isinf(vec(i))) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * @brief Check if matrix contains valid values
     */
    static bool isValidMatrix(const ocs2::matrix_t& mat) {
        for (int i = 0; i < mat.rows(); ++i) {
            for (int j = 0; j < mat.cols(); ++j) {
                if (std::isnan(mat(i, j)) || std::isinf(mat(i, j))) {
                    return false;
                }
            }
        }
        return true;
    }
    
    /**
     * @brief Print state validity check
     */
    void printStateValidity(const std::string& name, const ocs2::vector_t& vec, double time) {
        if (!flags_.enabled || !flags_.printStateValidity) return;
        
        bool valid = true;
        int invalidIdx = -1;
        for (int i = 0; i < vec.size(); ++i) {
            if (std::isnan(vec(i)) || std::isinf(vec(i))) {
                valid = false;
                invalidIdx = i;
                break;
            }
        }
        
        if (!valid) {
            std::lock_guard<std::mutex> lock(printMutex_);
            std::cout << "[DEBUG INVALID] t=" << std::fixed << std::setprecision(3) << time 
                      << " " << name << " INVALID at idx=" << invalidIdx 
                      << " value=" << vec(invalidIdx) << std::endl;
        }
    }
    
    /**
     * @brief Print base state (position + orientation)
     */
    void printBaseState(double time, 
                        const Eigen::Vector3d& position, 
                        double yaw, double pitch, double roll,
                        const Eigen::Vector3d& linearVel,
                        const Eigen::Vector3d& angularVel) {
        if (!flags_.enabled || !flags_.printBaseState) return;
        
        std::lock_guard<std::mutex> lock(printMutex_);
        std::cout << "[DEBUG BASE] t=" << std::fixed << std::setprecision(3) << time
                  << " pos=[" << std::setprecision(4) 
                  << position.x() << ", " << position.y() << ", " << position.z() << "]"
                  << " rpy=[" << roll << ", " << pitch << ", " << yaw << "]"
                  << " linVel=[" << linearVel.x() << ", " << linearVel.y() << ", " << linearVel.z() << "]"
                  << " angVel=[" << angularVel.x() << ", " << angularVel.y() << ", " << angularVel.z() << "]"
                  << std::endl;
    }
    
    /**
     * @brief Print foot positions in world frame
     */
    void printFootPositions(double time, 
                            const std::array<Eigen::Vector3d, 4>& footPositions,
                            const std::array<std::string, 4>& footNames = {"LF", "LH", "RF", "RH"}) {
        if (!flags_.enabled || !flags_.printFootPosition) return;
        
        std::lock_guard<std::mutex> lock(printMutex_);
        std::cout << "[DEBUG FOOT_POS] t=" << std::fixed << std::setprecision(3) << time << std::endl;
        for (int i = 0; i < 4; ++i) {
            std::cout << "  " << footNames[i] << ": [" << std::setprecision(4)
                      << footPositions[i].x() << ", " 
                      << footPositions[i].y() << ", " 
                      << footPositions[i].z() << "]" << std::endl;
        }
    }
    
    /**
     * @brief Print contact forces from MPC
     */
    void printContactForces(double time, const ocs2::vector_t& input, 
                            const ocs2::CentroidalModelInfo& info) {
        if (!flags_.enabled || !flags_.printContactForce) return;
        
        std::lock_guard<std::mutex> lock(printMutex_);
        std::cout << "[DEBUG CONTACT_F] t=" << std::fixed << std::setprecision(3) << time << std::endl;
        
        const std::array<std::string, 4> names = {"LF", "LH", "RF", "RH"};
        for (size_t i = 0; i < 4; ++i) {
            auto force = ocs2::centroidal_model::getContactForces(input, i, info);
            double fMag = force.norm();
            std::cout << "  " << names[i] << ": [" << std::setprecision(2)
                      << force.x() << ", " << force.y() << ", " << force.z() 
                      << "] |F|=" << fMag;
            
            // Warning for abnormal forces
            if (fMag > 1000.0) {
                std::cout << " [HIGH!]";
            } else if (force.z() < -10.0) {
                std::cout << " [PULL!]";
            }
            std::cout << std::endl;
        }
    }
    
    /**
     * @brief Print contact status
     */
    void printContactStatus(double time, const std::array<bool, 4>& contacts,
                            const std::array<std::string, 4>& footNames = {"LF", "LH", "RF", "RH"}) {
        if (!flags_.enabled || !flags_.printContactStatus) return;
        
        std::lock_guard<std::mutex> lock(printMutex_);
        std::cout << "[DEBUG CONTACT] t=" << std::fixed << std::setprecision(3) << time
                  << " [";
        for (int i = 0; i < 4; ++i) {
            std::cout << footNames[i] << ":" << (contacts[i] ? "1" : "0");
            if (i < 3) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    /**
     * @brief Print joint states (target and current)
     */
    void printJointStates(double time,
                          const double* targetJoints,
                          const double* currentJoints,
                          const double* jointVelocities) {
        if (!flags_.enabled) return;
        
        std::lock_guard<std::mutex> lock(printMutex_);
        
        // MuJoCo joint order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
        const std::array<std::string, 4> legNames = {"LF", "RF", "LH", "RH"};
        const std::array<std::string, 3> jointTypes = {"HAA", "HFE", "KFE"};
        
        if (flags_.printJointTarget) {
            std::cout << "[DEBUG JOINT_TGT] t=" << std::fixed << std::setprecision(3) << time << std::endl;
            for (int leg = 0; leg < 4; ++leg) {
                std::cout << "  " << legNames[leg] << ": [";
                for (int j = 0; j < 3; ++j) {
                    std::cout << std::setprecision(4) << targetJoints[leg * 3 + j];
                    if (j < 2) std::cout << ", ";
                }
                std::cout << "]" << std::endl;
            }
        }
        
        if (flags_.printJointCurrent) {
            std::cout << "[DEBUG JOINT_CUR] t=" << std::fixed << std::setprecision(3) << time << std::endl;
            for (int leg = 0; leg < 4; ++leg) {
                std::cout << "  " << legNames[leg] << ": pos=[";
                for (int j = 0; j < 3; ++j) {
                    std::cout << std::setprecision(4) << currentJoints[leg * 3 + j];
                    if (j < 2) std::cout << ", ";
                }
                std::cout << "] vel=[";
                for (int j = 0; j < 3; ++j) {
                    std::cout << std::setprecision(4) << jointVelocities[leg * 3 + j];
                    if (j < 2) std::cout << ", ";
                }
                std::cout << "]" << std::endl;
            }
        }
    }
    
    /**
     * @brief Print WBC computed torques
     */
    void printWbcTorques(double time, const ocs2::vector_t& wbcTorques) {
        if (!flags_.enabled || !flags_.printWbcTorques) return;
        
        std::lock_guard<std::mutex> lock(printMutex_);
        std::cout << "[DEBUG WBC_TAU] t=" << std::fixed << std::setprecision(3) << time << std::endl;
        
        // Base wrench (first 6)
        std::cout << "  BaseWrench: [";
        for (int i = 0; i < 6; ++i) {
            std::cout << std::setprecision(2) << wbcTorques(i);
            if (i < 5) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        // Joint torques (next 12)
        const std::array<std::string, 4> legNames = {"LF", "LH", "RF", "RH"};
        for (int leg = 0; leg < 4; ++leg) {
            std::cout << "  " << legNames[leg] << "_tau: [";
            for (int j = 0; j < 3; ++j) {
                std::cout << std::setprecision(2) << wbcTorques(6 + leg * 3 + j);
                if (j < 2) std::cout << ", ";
            }
            std::cout << "]";
            
            // Check for extreme torques
            double maxTau = 0;
            for (int j = 0; j < 3; ++j) {
                maxTau = std::max(maxTau, std::abs(wbcTorques(6 + leg * 3 + j)));
            }
            if (maxTau > 100.0) {
                std::cout << " [HIGH!]";
            }
            std::cout << std::endl;
        }
    }
    
    /**
     * @brief Print MPC optimal state summary
     */
    void printMpcState(double time, const ocs2::vector_t& optimalState, 
                       const ocs2::vector_t& targetState) {
        if (!flags_.enabled || !flags_.printMpcState) return;
        
        std::lock_guard<std::mutex> lock(printMutex_);
        std::cout << "[DEBUG MPC_STATE] t=" << std::fixed << std::setprecision(3) << time << std::endl;
        
        // Centroidal state: [momentum(6), position(3), orientation(3), joints(12)] = 24
        if (optimalState.size() >= 12) {
            std::cout << "  OptPos: [" << std::setprecision(4) 
                      << optimalState(6) << ", " << optimalState(7) << ", " << optimalState(8) << "]"
                      << " OptYPR: [" << optimalState(9) << ", " << optimalState(10) << ", " << optimalState(11) << "]"
                      << std::endl;
        }
        
        if (targetState.size() >= 12) {
            std::cout << "  TgtPos: [" << std::setprecision(4)
                      << targetState(6) << ", " << targetState(7) << ", " << targetState(8) << "]"
                      << " TgtYPR: [" << targetState(9) << ", " << targetState(10) << ", " << targetState(11) << "]"
                      << std::endl;
        }
    }
    
    /**
     * @brief Print Jacobian for a foot
     */
    void printJacobian(double time, int footIdx, const Eigen::MatrixXd& jacobian) {
        if (!flags_.enabled || !flags_.printJacobian) return;
        
        std::lock_guard<std::mutex> lock(printMutex_);
        const std::array<std::string, 4> names = {"LF", "LH", "RF", "RH"};
        std::cout << "[DEBUG JACOBIAN] t=" << std::fixed << std::setprecision(3) << time
                  << " foot=" << names[footIdx] << std::endl;
        
        std::cout << "  J(" << jacobian.rows() << "x" << jacobian.cols() << "):" << std::endl;
        for (int i = 0; i < std::min(3, (int)jacobian.rows()); ++i) {
            std::cout << "    [";
            for (int j = 0; j < std::min(12, (int)jacobian.cols()); ++j) {
                std::cout << std::setw(8) << std::setprecision(3) << jacobian(i, j);
                if (j < std::min(12, (int)jacobian.cols()) - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
    }
    
    /**
     * @brief Print separator line
     */
    void printSeparator(const std::string& title = "") {
        if (!flags_.enabled) return;
        
        std::lock_guard<std::mutex> lock(printMutex_);
        if (title.empty()) {
            std::cout << "========================================" << std::endl;
        } else {
            std::cout << "======== " << title << " ========" << std::endl;
        }
    }
    
    /**
     * @brief Compute foot positions using Pinocchio FK
     */
    static std::array<Eigen::Vector3d, 4> computeFootPositions(
        ocs2::PinocchioInterface& pinocchioInterface,
        const ocs2::vector_t& q,
        const std::array<std::string, 4>& footFrameNames) {
        
        std::array<Eigen::Vector3d, 4> footPositions;
        
        const auto& model = pinocchioInterface.getModel();
        auto& data = pinocchioInterface.getData();
        
        // Forward kinematics
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        
        for (int i = 0; i < 4; ++i) {
            auto frameId = model.getFrameId(footFrameNames[i]);
            if (frameId < model.nframes) {
                footPositions[i] = data.oMf[frameId].translation();
            } else {
                footPositions[i] = Eigen::Vector3d::Zero();
                std::cerr << "[DEBUG] Frame not found: " << footFrameNames[i] << std::endl;
            }
        }
        
        return footPositions;
    }
    
    /**
     * @brief Check and warn about abnormal values
     */
    void checkAbnormalValues(double time,
                             const ocs2::vector_t& state,
                             const ocs2::vector_t& input,
                             const std::string& context) {
        if (!flags_.enabled || !flags_.printStateValidity) return;
        
        bool stateValid = isValidVector(state);
        bool inputValid = isValidVector(input);
        
        if (!stateValid || !inputValid) {
            std::lock_guard<std::mutex> lock(printMutex_);
            std::cout << "[DEBUG ABNORMAL] t=" << std::fixed << std::setprecision(3) << time
                      << " context=" << context << std::endl;
            
            if (!stateValid) {
                std::cout << "  STATE has NaN/Inf!" << std::endl;
                for (int i = 0; i < state.size(); ++i) {
                    if (std::isnan(state(i)) || std::isinf(state(i))) {
                        std::cout << "    state[" << i << "] = " << state(i) << std::endl;
                    }
                }
            }
            
            if (!inputValid) {
                std::cout << "  INPUT has NaN/Inf!" << std::endl;
                for (int i = 0; i < input.size(); ++i) {
                    if (std::isnan(input(i)) || std::isinf(input(i))) {
                        std::cout << "    input[" << i << "] = " << input(i) << std::endl;
                    }
                }
            }
        }
        
        // Check for extremely large values
        double stateMax = state.cwiseAbs().maxCoeff();
        double inputMax = input.cwiseAbs().maxCoeff();
        
        if (stateMax > 1000.0 || inputMax > 10000.0) {
            std::lock_guard<std::mutex> lock(printMutex_);
            std::cout << "[DEBUG EXTREME] t=" << std::fixed << std::setprecision(3) << time
                      << " context=" << context
                      << " stateMax=" << stateMax
                      << " inputMax=" << inputMax << std::endl;
        }
    }
    
private:
    DebugFlags flags_;
    std::mutex printMutex_;
    std::ofstream logFile_;
};

// Global debug utilities instance
inline DebugUtils& getDebugUtils() {
    static DebugUtils instance;
    return instance;
}

}  // namespace quadruped_nmpc

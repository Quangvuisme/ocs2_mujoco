#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>

namespace anymal_pinocchio {

// ANYmal state dimensions
constexpr int BASE_STATE_DIM = 12;   // orientation(3) + position(3) + angular_vel(3) + linear_vel(3)
constexpr int JOINT_STATE_DIM = 24;  // 12 joints position + 12 joints velocity
constexpr int STATE_DIM = BASE_STATE_DIM + JOINT_STATE_DIM;  // 36
constexpr int INPUT_DIM = 12;  // 12 joint torques

// Joint names in order
const std::vector<std::string> JOINT_NAMES = {
    "LF_HAA", "LF_HFE", "LF_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE"
};

// Foot frame names
const std::vector<std::string> FOOT_FRAMES = {
    "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"
};

// Default standing pose joint angles
const Eigen::Matrix<double, 12, 1> DEFAULT_JOINT_ANGLES = (Eigen::Matrix<double, 12, 1>() <<
    -0.25,  0.60, -0.85,   // LF
     0.25,  0.60, -0.85,   // RF
    -0.25, -0.60,  0.85,   // LH
     0.25, -0.60,  0.85    // RH
).finished();

// Default standing height
constexpr double DEFAULT_STANDING_HEIGHT = 0.57;

// Robot parameters
struct AnymalParameters {
    double mass = 50.0;           // kg
    double gravity = 9.81;        // m/s^2
    double standingHeight = 0.57; // m
    
    // PD gains for joint control (tuned for stability)
    double kp_haa = 1.0;
    double kp_hfe = 1.0;
    double kp_kfe = 1.0;
    
    double kd_haa = 0.05;
    double kd_hfe = 0.05;
    double kd_kfe = 0.05;
    
    // Torque limits
    double maxTorque = 80.0;  // Nm
};

}  // namespace anymal_pinocchio

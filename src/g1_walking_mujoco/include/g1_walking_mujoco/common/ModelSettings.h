#pragma once

#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace g1_walking {

struct ModelSettings {
    ocs2::scalar_t positionErrorGain = 0.0;

    ocs2::scalar_t phaseTransitionStanceTime = 0.4;

    bool verboseCppAd = true;
    bool recompileLibrariesCppAd = true;
    std::string modelFolderCppAd = "/tmp/g1_walking_mujoco";

    // G1 leg joint names (same order as URDF). All other URDF joints are fixed.
    std::vector<std::string> jointNames{
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"
    };
    std::vector<std::string> contactNames6DoF{};
    // two point contacts per foot: {left toe, right toe, left heel, right heel}
    std::vector<std::string> contactNames3DoF{"l_foot_toe", "r_foot_toe", "l_foot_heel", "r_foot_heel"};
    // the toe-heel line contact cannot resist a moment about the sole axis: the MPC keeps these joints still
    std::vector<std::string> footRollJointNames{"left_ankle_roll_joint", "right_ankle_roll_joint"};
};

ModelSettings loadModelSettings(const std::string &filename, const std::string &fieldName = "model_settings",
                                bool verbose = "true");

}  // namespace g1_walking

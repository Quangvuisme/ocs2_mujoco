#pragma once

#include <string>
#include <vector>

namespace anymal_centroidal {

// ANYmal C joint names - URDF order (same as MuJoCo)
const std::vector<std::string> JOINT_NAMES_URDF = {
    "LF_HAA", "LF_HFE", "LF_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE"
};

// Foot frame names
const std::vector<std::string> FOOT_FRAMES = {
    "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"
};

// Default standing joint angles (MuJoCo/URDF order: LF, RF, LH, RH)
const double DEFAULT_JOINT_ANGLES[12] = {
    -0.25,  0.60, -0.85,   // LF
     0.25,  0.60, -0.85,   // RF
    -0.25, -0.60,  0.85,   // LH
     0.25, -0.60,  0.85    // RH
};

}  // namespace anymal_centroidal

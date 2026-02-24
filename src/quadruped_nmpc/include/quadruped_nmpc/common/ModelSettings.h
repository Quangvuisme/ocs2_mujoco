#pragma once

#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace quadruped_nmpc {

struct ModelSettings {
    ocs2::scalar_t positionErrorGain = 0.0;

    ocs2::scalar_t phaseTransitionStanceTime = 0.4;

    bool verboseCppAd = true;
    bool recompileLibrariesCppAd = true;
    std::string modelFolderCppAd = "/tmp/quadruped_nmpc";

    // Joint names - used to get names for knees and check URDF for extra joints
    // Default: ANYmal C naming convention
    std::vector<std::string> jointNames{
        "LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
        "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"
    };
    
    // Contact names for 3DoF feet (default: ANYmal C naming)
    // Can be overridden in config file for other robots (e.g., Go1/Go2 use FL_foot, etc.)
    std::vector<std::string> contactNames6DoF{};
    std::vector<std::string> contactNames3DoF{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
};

ModelSettings loadModelSettings(const std::string &filename, const std::string &fieldName = "model_settings",
                                bool verbose = "true");

}  // namespace quadruped_nmpc

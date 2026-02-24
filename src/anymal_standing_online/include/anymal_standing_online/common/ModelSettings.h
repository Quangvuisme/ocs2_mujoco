#pragma once

#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace anymal_standing_online {

struct ModelSettings {
    ocs2::scalar_t positionErrorGain = 0.0;

    ocs2::scalar_t phaseTransitionStanceTime = 0.4;

    bool verboseCppAd = true;
    bool recompileLibrariesCppAd = true;
    std::string modelFolderCppAd = "/tmp/anymal_standing_online";

    // ANYmal C joint names (same order as URDF)
    std::vector<std::string> jointNames{
        "LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
        "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"
    };
    std::vector<std::string> contactNames6DoF{};
    std::vector<std::string> contactNames3DoF{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
};

ModelSettings loadModelSettings(const std::string &filename, const std::string &fieldName = "model_settings",
                                bool verbose = "true");

}  // namespace anymal_standing_online

#include <anymal_walking_online/common/ModelSettings.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>


namespace anymal_walking_online {
    ModelSettings loadModelSettings(const std::string &filename, const std::string &fieldName, bool verbose) {
        ModelSettings modelSettings;

        boost::property_tree::ptree pt;
        read_info(filename, pt);

        if (verbose) {
            std::cerr << "\n #### Legged Robot Model Settings:";
            std::cerr << "\n #### =============================================================================\n";
        }

        ocs2::loadData::loadPtreeValue(pt, modelSettings.positionErrorGain, fieldName + ".positionErrorGain", verbose);
        ocs2::loadData::loadPtreeValue(pt, modelSettings.phaseTransitionStanceTime, fieldName + ".phaseTransitionStanceTime",
                                 verbose);

        ocs2::loadData::loadPtreeValue(pt, modelSettings.verboseCppAd, fieldName + ".verboseCppAd", verbose);
        ocs2::loadData::loadPtreeValue(pt, modelSettings.recompileLibrariesCppAd, fieldName + ".recompileLibrariesCppAd",
                                 verbose);
        ocs2::loadData::loadPtreeValue(pt, modelSettings.modelFolderCppAd, fieldName + ".modelFolderCppAd", verbose);

        if (verbose) {
            std::cerr << " #### =============================================================================" <<
                    std::endl;
        }

        return modelSettings;
    }
} // namespace anymal_walking_online

#include <quadruped_nmpc/common/ModelSettings.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>


namespace quadruped_nmpc {
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

        // Load joint names (optional - uses defaults if not specified)
        std::vector<std::string> jointNames;
        ocs2::loadData::loadStdVector(filename, fieldName + ".jointNames", jointNames, verbose);
        if (!jointNames.empty()) {
            modelSettings.jointNames = jointNames;
        }
        
        // Load contact names (optional - uses defaults if not specified)
        std::vector<std::string> contactNames3DoF;
        ocs2::loadData::loadStdVector(filename, fieldName + ".contactNames3DoF", contactNames3DoF, verbose);
        if (!contactNames3DoF.empty()) {
            modelSettings.contactNames3DoF = contactNames3DoF;
        }

        if (verbose) {
            std::cerr << " #### jointNames: ";
            for (const auto& name : modelSettings.jointNames) {
                std::cerr << name << " ";
            }
            std::cerr << std::endl;
            std::cerr << " #### contactNames3DoF: ";
            for (const auto& name : modelSettings.contactNames3DoF) {
                std::cerr << name << " ";
            }
            std::cerr << std::endl;
            std::cerr << " #### =============================================================================" <<
                    std::endl;
        }

        return modelSettings;
    }
} // namespace quadruped_nmpc

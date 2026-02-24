#include <anymal_standing_online/dynamics/LeggedRobotDynamicsAD.h>

namespace anymal_standing_online {
    LeggedRobotDynamicsAD::LeggedRobotDynamicsAD(const ocs2::PinocchioInterface &pinocchioInterface,
                                                 const ocs2::CentroidalModelInfo &info,
                                                 const std::string &modelName, const ModelSettings &modelSettings)
        : pinocchioCentroidalDynamicsAd_(pinocchioInterface, info, modelName, modelSettings.modelFolderCppAd,
                                         modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd) {
    }


    ocs2::vector_t LeggedRobotDynamicsAD::computeFlowMap(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                                                   const ocs2::PreComputation &preComp) {
        return pinocchioCentroidalDynamicsAd_.getValue(time, state, input);
    }


    ocs2::VectorFunctionLinearApproximation LeggedRobotDynamicsAD::linearApproximation(
        ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) {
        return pinocchioCentroidalDynamicsAd_.getLinearApproximation(time, state, input);
    }
}

#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "anymal_standing_online/common/ModelSettings.h"

namespace anymal_standing_online {
    class LeggedRobotDynamicsAD final : public ocs2::SystemDynamicsBase {
    public:
        LeggedRobotDynamicsAD(const ocs2::PinocchioInterface &pinocchioInterface, const ocs2::CentroidalModelInfo &info,
                              const std::string &modelName,
                              const ModelSettings &modelSettings);

        ~LeggedRobotDynamicsAD() override = default;

        LeggedRobotDynamicsAD *clone() const override { return new LeggedRobotDynamicsAD(*this); }

        ocs2::vector_t computeFlowMap(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                                const ocs2::PreComputation &preComp) override;

        ocs2::VectorFunctionLinearApproximation linearApproximation(ocs2::scalar_t time, const ocs2::vector_t &state,
                                                              const ocs2::vector_t &input,
                                                              const ocs2::PreComputation &preComp) override;

    private:
        LeggedRobotDynamicsAD(const LeggedRobotDynamicsAD &rhs) = default;

        ocs2::PinocchioCentroidalDynamicsAD pinocchioCentroidalDynamicsAd_;
    };
}

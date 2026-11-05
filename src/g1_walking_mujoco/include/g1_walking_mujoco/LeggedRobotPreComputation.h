#pragma once

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "g1_walking_mujoco/common/ModelSettings.h"
#include "g1_walking_mujoco/constraint/EndEffectorLinearConstraint.h"
#include "g1_walking_mujoco/foot_planner/SwingTrajectoryPlanner.h"

namespace g1_walking {
    /** Callback for caching and reference update */
    class LeggedRobotPreComputation : public ocs2::PreComputation {
    public:
        LeggedRobotPreComputation(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info,
                                  const SwingTrajectoryPlanner &swingTrajectoryPlanner, ModelSettings settings);

        ~LeggedRobotPreComputation() override = default;

        LeggedRobotPreComputation *clone() const override;

        void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t &x, const ocs2::vector_t &u) override;

        const std::vector<EndEffectorLinearConstraint::Config> &getEeNormalVelocityConstraintConfigs() const {
            return eeNormalVelConConfigs_;
        }

        ocs2::PinocchioInterface &getPinocchioInterface() { return pinocchioInterface_; }
        const ocs2::PinocchioInterface &getPinocchioInterface() const { return pinocchioInterface_; }

    private:
        LeggedRobotPreComputation(const LeggedRobotPreComputation &other) = default;

        ocs2::PinocchioInterface pinocchioInterface_;
        ocs2::CentroidalModelInfo info_;
        const SwingTrajectoryPlanner *swingTrajectoryPlannerPtr_;
        const ModelSettings settings_;

        std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
    };
}

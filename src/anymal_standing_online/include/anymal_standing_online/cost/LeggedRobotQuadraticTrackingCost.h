#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include "anymal_standing_online/common/utils.h"
#include "anymal_standing_online/reference_manager/SwitchedModelReferenceManager.h"

namespace anymal_standing_online {
    /**
    * State-input tracking cost used for intermediate times
    */
    class LeggedRobotStateInputQuadraticCost final : public ocs2::QuadraticStateInputCost {
    public:
        LeggedRobotStateInputQuadraticCost(ocs2::matrix_t Q, ocs2::matrix_t R, ocs2::CentroidalModelInfo info,
                                           const SwitchedModelReferenceManager &referenceManager)
            : QuadraticStateInputCost(std::move(Q), std::move(R)), info_(std::move(info)),
              referenceManagerPtr_(&referenceManager) {
        }

        ~LeggedRobotStateInputQuadraticCost() override = default;

        LeggedRobotStateInputQuadraticCost *clone() const override {
            return new LeggedRobotStateInputQuadraticCost(*this);
        }

    private:
        LeggedRobotStateInputQuadraticCost(const LeggedRobotStateInputQuadraticCost &rhs) = default;

        std::pair<ocs2::vector_t, ocs2::vector_t> getStateInputDeviation(ocs2::scalar_t time, const ocs2::vector_t &state,
                                                             const ocs2::vector_t &input,
                                                             const ocs2::TargetTrajectories &targetTrajectories)
        const override {
            const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
            const ocs2::vector_t xNominal = targetTrajectories.getDesiredState(time);
            const ocs2::vector_t uNominal = weightCompensatingInput(info_, contactFlags);
            return {state - xNominal, input - uNominal};
        }

        const ocs2::CentroidalModelInfo info_;
        const SwitchedModelReferenceManager *referenceManagerPtr_;
    };

    /**
    * State tracking cost used for the final time
    */
    class LeggedRobotStateQuadraticCost final : public ocs2::QuadraticStateCost {
    public:
        LeggedRobotStateQuadraticCost(ocs2::matrix_t Q, ocs2::CentroidalModelInfo info,
                                      const SwitchedModelReferenceManager &referenceManager)
            : QuadraticStateCost(std::move(Q)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {
        }

        ~LeggedRobotStateQuadraticCost() override = default;

        LeggedRobotStateQuadraticCost *clone() const override { return new LeggedRobotStateQuadraticCost(*this); }

    private:
        LeggedRobotStateQuadraticCost(const LeggedRobotStateQuadraticCost &rhs) = default;

        ocs2::vector_t getStateDeviation(ocs2::scalar_t time, const ocs2::vector_t &state,
                                   const ocs2::TargetTrajectories &targetTrajectories) const override {
            const ocs2::vector_t xNominal = targetTrajectories.getDesiredState(time);
            return state - xNominal;
        }

        const ocs2::CentroidalModelInfo info_;
        const SwitchedModelReferenceManager *referenceManagerPtr_;
    };
}

#include "quadruped_nmpc/initialization/LeggedRobotInitializer.h"

#include "quadruped_nmpc/common/utils.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>


namespace quadruped_nmpc {
    LeggedRobotInitializer::LeggedRobotInitializer(ocs2::CentroidalModelInfo info,
                                                   const SwitchedModelReferenceManager &referenceManager,
                                                   bool extendNormalizedMomentum)
        : info_(std::move(info)), referenceManagerPtr_(&referenceManager),
          extendNormalizedMomentum_(extendNormalizedMomentum) {
    }


    LeggedRobotInitializer *LeggedRobotInitializer::clone() const {
        return new LeggedRobotInitializer(*this);
    }


    void LeggedRobotInitializer::compute(ocs2::scalar_t time, const ocs2::vector_t &state, ocs2::scalar_t nextTime, ocs2::vector_t &input,
                                         ocs2::vector_t &nextState) {
        const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
        input = weightCompensatingInput(info_, contactFlags);
        nextState = state;
        if (!extendNormalizedMomentum_) {
            ocs2::centroidal_model::getNormalizedMomentum(nextState, info_).setZero();
        }
    }
} // namespace quadruped_nmpc

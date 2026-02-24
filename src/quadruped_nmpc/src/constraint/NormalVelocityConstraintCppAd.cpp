#include <quadruped_nmpc/constraint/NormalVelocityConstraintCppAd.h>
#include <quadruped_nmpc/LeggedRobotPreComputation.h>


namespace quadruped_nmpc {
    NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(
        const SwitchedModelReferenceManager &referenceManager,
        const ocs2::EndEffectorKinematics<ocs2::scalar_t> &endEffectorKinematics,
        size_t contactPointIndex)
        : StateInputConstraint(ocs2::ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 1)),
          contactPointIndex_(contactPointIndex) {
    }


    NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd &rhs)
        : StateInputConstraint(rhs),
          referenceManagerPtr_(rhs.referenceManagerPtr_),
          eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
          contactPointIndex_(rhs.contactPointIndex_) {
    }


    bool NormalVelocityConstraintCppAd::isActive(ocs2::scalar_t time) const {
        return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
    }


    ocs2::vector_t NormalVelocityConstraintCppAd::getValue(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                                                     const ocs2::PreComputation &preComp) const {
        const auto &preCompLegged = ocs2::cast<LeggedRobotPreComputation>(preComp);
        eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

        return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
    }


    ocs2::VectorFunctionLinearApproximation NormalVelocityConstraintCppAd::getLinearApproximation(
        ocs2::scalar_t time, const ocs2::vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const {
        const auto &preCompLegged = ocs2::cast<LeggedRobotPreComputation>(preComp);
        eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

        return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
    }
} // namespace quadruped_nmpc

#include <anymal_standing_online/constraint/ZeroVelocityConstraintCppAd.h>


namespace anymal_standing_online {
    ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const SwitchedModelReferenceManager &referenceManager,
                                                             const ocs2::EndEffectorKinematics<ocs2::scalar_t> &
                                                             endEffectorKinematics,
                                                             size_t contactPointIndex,
                                                             EndEffectorLinearConstraint::Config config)
        : StateInputConstraint(ocs2::ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 3, std::move(config))),
          contactPointIndex_(contactPointIndex) {
    }


    ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const ZeroVelocityConstraintCppAd &rhs)
        : StateInputConstraint(rhs),
          referenceManagerPtr_(rhs.referenceManagerPtr_),
          eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
          contactPointIndex_(rhs.contactPointIndex_) {
    }


    bool ZeroVelocityConstraintCppAd::isActive(ocs2::scalar_t time) const {
        return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
    }


    ocs2::vector_t ZeroVelocityConstraintCppAd::getValue(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                                                   const ocs2::PreComputation &preComp) const {
        return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
    }


    ocs2::VectorFunctionLinearApproximation ZeroVelocityConstraintCppAd::getLinearApproximation(
        ocs2::scalar_t time, const ocs2::vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const {
        return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
    }
} // namespace anymal_standing_online

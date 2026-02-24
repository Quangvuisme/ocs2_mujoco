#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "anymal_walking_online/constraint/EndEffectorLinearConstraint.h"
#include "anymal_walking_online/reference_manager/SwitchedModelReferenceManager.h"

namespace anymal_walking_online {
    /**
    * Specializes the CppAd version of zero velocity constraint on an end-effector position and linear velocity.
    * Constructs the member EndEffectorLinearConstraint object with number of constraints of 3.
    *
    * See also EndEffectorLinearConstraint for the underlying computation.
    */
    class ZeroVelocityConstraintCppAd final : public ocs2::StateInputConstraint {
    public:
        /**
         * Constructor
         * @param [in] referenceManager : Switched model ReferenceManager
         * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
         * @param [in] contactPointIndex : The 3 DoF contact index.
         * @param [in] config: The constraint coefficients
         */
        ZeroVelocityConstraintCppAd(const SwitchedModelReferenceManager &referenceManager,
                                    const ocs2::EndEffectorKinematics<ocs2::scalar_t> &endEffectorKinematics,
                                    size_t contactPointIndex,
                                    EndEffectorLinearConstraint::Config config = EndEffectorLinearConstraint::Config());

        ~ZeroVelocityConstraintCppAd() override = default;

        ZeroVelocityConstraintCppAd *clone() const override { return new ZeroVelocityConstraintCppAd(*this); }

        bool isActive(ocs2::scalar_t time) const override;

        size_t getNumConstraints(ocs2::scalar_t time) const override { return 3; }

        ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                          const ocs2::PreComputation &preComp) const override;

        ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t &state,
                                                                 const ocs2::vector_t &input,
                                                                 const ocs2::PreComputation &preComp) const override;

    private:
        ZeroVelocityConstraintCppAd(const ZeroVelocityConstraintCppAd &rhs);

        const SwitchedModelReferenceManager *referenceManagerPtr_;
        std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
        const size_t contactPointIndex_;
    };
}

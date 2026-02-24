#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "anymal_walking_online/constraint/EndEffectorLinearConstraint.h"
#include "anymal_walking_online/reference_manager/SwitchedModelReferenceManager.h"

namespace anymal_walking_online {
    /**
    * Specializes the CppAd version of normal velocity constraint on an end-effector position and linear velocity.
    * Constructs the member EndEffectorLinearConstraint object with number of constraints of 1.
    *
    * See also EndEffectorLinearConstraint for the underlying computation.
    */
    class NormalVelocityConstraintCppAd final : public ocs2::StateInputConstraint {
    public:
        /**
         * Constructor
         * @param [in] referenceManager : Switched model ReferenceManager
         * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
         * @param [in] contactPointIndex : The 3 DoF contact index.
         */
        NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager &referenceManager,
                                      const ocs2::EndEffectorKinematics<ocs2::scalar_t> &endEffectorKinematics,
                                      size_t contactPointIndex);

        ~NormalVelocityConstraintCppAd() override = default;

        NormalVelocityConstraintCppAd *clone() const override { return new NormalVelocityConstraintCppAd(*this); }

        bool isActive(ocs2::scalar_t time) const override;

        size_t getNumConstraints(ocs2::scalar_t time) const override { return 1; }

        ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                          const ocs2::PreComputation &preComp) const override;

        ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t &state,
                                                                 const ocs2::vector_t &input,
                                                                 const ocs2::PreComputation &preComp) const override;

    private:
        NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd &rhs);

        const SwitchedModelReferenceManager *referenceManagerPtr_;
        std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
        const size_t contactPointIndex_;
    };
}

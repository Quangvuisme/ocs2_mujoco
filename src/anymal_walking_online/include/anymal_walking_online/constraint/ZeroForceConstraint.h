#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "anymal_walking_online/reference_manager/SwitchedModelReferenceManager.h"

namespace anymal_walking_online {
    class ZeroForceConstraint final : public ocs2::StateInputConstraint {
    public:
        /*
         * Constructor
         * @param [in] referenceManager : Switched model ReferenceManager.
         * @param [in] contactPointIndex : The 3 DoF contact index.
         * @param [in] info : The centroidal model information.
         */
        ZeroForceConstraint(const SwitchedModelReferenceManager &referenceManager, size_t contactPointIndex,
                            ocs2::CentroidalModelInfo info);

        ~ZeroForceConstraint() override = default;

        ZeroForceConstraint *clone() const override { return new ZeroForceConstraint(*this); }

        bool isActive(ocs2::scalar_t time) const override;

        size_t getNumConstraints(ocs2::scalar_t time) const override { return 3; }

        ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                          const ocs2::PreComputation &preComp) const override;

        ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t &state,
                                                                 const ocs2::vector_t &input,
                                                                 const ocs2::PreComputation &preComp) const override;

    private:
        ZeroForceConstraint(const ZeroForceConstraint &other) = default;

        const SwitchedModelReferenceManager *referenceManagerPtr_;
        const size_t contactPointIndex_;
        const ocs2::CentroidalModelInfo info_;
    };
}

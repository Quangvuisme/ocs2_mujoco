#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/initialization/Initializer.h>

#include "anymal_standing_online/reference_manager/SwitchedModelReferenceManager.h"

namespace anymal_standing_online {
    class LeggedRobotInitializer final : public ocs2::Initializer {
    public:
        /**
         * Constructor
         * @param [in] info : The centroidal model information.
         * @param [in] referenceManager : Switched system reference manager.
         * @param [in] extendNormalizedMomentum: If true, it extrapolates the normalized momenta; otherwise sets them to zero.
         */
        LeggedRobotInitializer(ocs2::CentroidalModelInfo info, const SwitchedModelReferenceManager &referenceManager,
                               bool extendNormalizedMomentum = false);

        ~LeggedRobotInitializer() override = default;

        LeggedRobotInitializer *clone() const override;

        void compute(ocs2::scalar_t time, const ocs2::vector_t &state, ocs2::scalar_t nextTime, ocs2::vector_t &input,
                     ocs2::vector_t &nextState) override;

    private:
        LeggedRobotInitializer(const LeggedRobotInitializer &other) = default;

        const ocs2::CentroidalModelInfo info_;
        const SwitchedModelReferenceManager *referenceManagerPtr_;
        const bool extendNormalizedMomentum_;
    };
}

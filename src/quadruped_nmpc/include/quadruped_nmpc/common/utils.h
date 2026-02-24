#pragma once

#include <array>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

#include "quadruped_nmpc/common/Types.h"

namespace quadruped_nmpc {

/** Counts contact feet */
inline size_t numberOfClosedContacts(const contact_flag_t &contactFlags) {
    size_t numStanceLegs = 0;
    for (auto legInContact : contactFlags) {
        if (legInContact) {
            ++numStanceLegs;
        }
    }
    return numStanceLegs;
}

/** Computes an input with zero joint velocity and forces which equally distribute the robot weight between contact feet. */
inline ocs2::vector_t weightCompensatingInput(const ocs2::CentroidalModelInfoTpl<ocs2::scalar_t> &info,
                                              const contact_flag_t &contactFlags) {
    const auto numStanceLegs = numberOfClosedContacts(contactFlags);
    ocs2::vector_t input = ocs2::vector_t::Zero(info.inputDim);
    if (numStanceLegs > 0) {
        const ocs2::scalar_t totalWeight = info.robotMass * 9.81;
        const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
        for (size_t i = 0; i < contactFlags.size(); i++) {
            if (contactFlags[i]) {
                ocs2::centroidal_model::getContactForces(input, i, info) = forceInInertialFrame;
            }
        }
    }
    return input;
}

}  // namespace quadruped_nmpc

#pragma once

#include "g1_perceptive_mujoco/foot_planner/CubicSpline.h"

namespace g1_perceptive {
    class SplineCpg {
    public:
        /** @param apexPhase: time of the apex as a fraction of the swing (0.5: middle). */
        SplineCpg(CubicSpline::Node liftOff, ocs2::scalar_t midHeight, CubicSpline::Node touchDown,
                  ocs2::scalar_t apexPhase = 0.5);

        ocs2::scalar_t position(ocs2::scalar_t time) const;

        ocs2::scalar_t velocity(ocs2::scalar_t time) const;

        ocs2::scalar_t acceleration(ocs2::scalar_t time) const;

        ocs2::scalar_t startTimeDerivative(ocs2::scalar_t time) const;

        ocs2::scalar_t finalTimeDerivative(ocs2::scalar_t time) const;

    private:
        ocs2::scalar_t apexPhase_;
        ocs2::scalar_t midTime_;
        CubicSpline leftSpline_;
        CubicSpline rightSpline_;
    };
}

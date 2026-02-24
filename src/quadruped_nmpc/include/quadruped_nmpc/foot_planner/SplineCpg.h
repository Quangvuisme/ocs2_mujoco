#pragma once

#include "quadruped_nmpc/foot_planner/CubicSpline.h"

namespace quadruped_nmpc {
    class SplineCpg {
    public:
        SplineCpg(CubicSpline::Node liftOff, ocs2::scalar_t midHeight, CubicSpline::Node touchDown);

        ocs2::scalar_t position(ocs2::scalar_t time) const;

        ocs2::scalar_t velocity(ocs2::scalar_t time) const;

        ocs2::scalar_t acceleration(ocs2::scalar_t time) const;

        ocs2::scalar_t startTimeDerivative(ocs2::scalar_t time) const;

        ocs2::scalar_t finalTimeDerivative(ocs2::scalar_t time) const;

    private:
        ocs2::scalar_t midTime_;
        CubicSpline leftSpline_;
        CubicSpline rightSpline_;
    };
}

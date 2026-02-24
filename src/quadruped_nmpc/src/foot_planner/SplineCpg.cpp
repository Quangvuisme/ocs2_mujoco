#include <quadruped_nmpc/foot_planner/SplineCpg.h>


namespace quadruped_nmpc {
    SplineCpg::SplineCpg(CubicSpline::Node liftOff, ocs2::scalar_t midHeight, CubicSpline::Node touchDown)
        : midTime_((liftOff.time + touchDown.time) / 2),
          leftSpline_(liftOff, CubicSpline::Node{midTime_, midHeight, 0.0}),
          rightSpline_(CubicSpline::Node{midTime_, midHeight, 0.0}, touchDown) {
    }


    ocs2::scalar_t SplineCpg::position(ocs2::scalar_t time) const {
        return (time < midTime_) ? leftSpline_.position(time) : rightSpline_.position(time);
    }


    ocs2::scalar_t SplineCpg::velocity(ocs2::scalar_t time) const {
        return (time < midTime_) ? leftSpline_.velocity(time) : rightSpline_.velocity(time);
    }


    ocs2::scalar_t SplineCpg::acceleration(ocs2::scalar_t time) const {
        return (time < midTime_) ? leftSpline_.acceleration(time) : rightSpline_.acceleration(time);
    }


    ocs2::scalar_t SplineCpg::startTimeDerivative(ocs2::scalar_t time) const {
        if (time <= midTime_) {
            return leftSpline_.startTimeDerivative(time) + 0.5 * leftSpline_.startTimeDerivative(time);
        }
        return 0.5 * rightSpline_.startTimeDerivative(time);
    }


    ocs2::scalar_t SplineCpg::finalTimeDerivative(ocs2::scalar_t time) const {
        if (time <= midTime_) {
            return 0.5 * leftSpline_.finalTimeDerivative(time);
        }
        return rightSpline_.finalTimeDerivative(time) + 0.5 * rightSpline_.finalTimeDerivative(time);
    }
} // namespace quadruped_nmpc

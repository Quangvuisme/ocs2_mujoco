#include <g1_perceptive_mujoco/foot_planner/SplineCpg.h>


namespace g1_perceptive {
    SplineCpg::SplineCpg(CubicSpline::Node liftOff, ocs2::scalar_t midHeight, CubicSpline::Node touchDown,
                         ocs2::scalar_t apexPhase)
        : apexPhase_(apexPhase),
          midTime_(liftOff.time + apexPhase * (touchDown.time - liftOff.time)),
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
            return leftSpline_.startTimeDerivative(time) + (1.0 - apexPhase_) * leftSpline_.startTimeDerivative(time);
        }
        return (1.0 - apexPhase_) * rightSpline_.startTimeDerivative(time);
    }


    ocs2::scalar_t SplineCpg::finalTimeDerivative(ocs2::scalar_t time) const {
        if (time <= midTime_) {
            return apexPhase_ * leftSpline_.finalTimeDerivative(time);
        }
        return rightSpline_.finalTimeDerivative(time) + apexPhase_ * rightSpline_.finalTimeDerivative(time);
    }
} // namespace g1_perceptive

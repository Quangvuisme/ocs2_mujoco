#include <quadruped_nmpc/foot_planner/CubicSpline.h>


namespace quadruped_nmpc {

    CubicSpline::CubicSpline(Node start, Node end) {
        assert(start.time < end.time);
        t0_ = start.time;
        t1_ = end.time;
        dt_ = end.time - start.time;

        ocs2::scalar_t dp = end.position - start.position;
        ocs2::scalar_t dv = end.velocity - start.velocity;

        dc0_ = 0.0;
        dc1_ = start.velocity;
        dc2_ = -(3.0 * start.velocity + dv);
        dc3_ = (2.0 * start.velocity + dv);

        c0_ = dc0_ * dt_ + start.position;
        c1_ = dc1_ * dt_;
        c2_ = dc2_ * dt_ + 3.0 * dp;
        c3_ = dc3_ * dt_ - 2.0 * dp;
    }


    ocs2::scalar_t CubicSpline::position(ocs2::scalar_t time) const {
        ocs2::scalar_t tn = normalizedTime(time);
        return c3_ * tn * tn * tn + c2_ * tn * tn + c1_ * tn + c0_;
    }


    ocs2::scalar_t CubicSpline::velocity(ocs2::scalar_t time) const {
        ocs2::scalar_t tn = normalizedTime(time);
        return (3.0 * c3_ * tn * tn + 2.0 * c2_ * tn + c1_) / dt_;
    }


    ocs2::scalar_t CubicSpline::acceleration(ocs2::scalar_t time) const {
        ocs2::scalar_t tn = normalizedTime(time);
        return (6.0 * c3_ * tn + 2.0 * c2_) / (dt_ * dt_);
    }


    ocs2::scalar_t CubicSpline::startTimeDerivative(ocs2::scalar_t t) const {
        ocs2::scalar_t tn = normalizedTime(t);
        ocs2::scalar_t dCoff = -(dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
        ocs2::scalar_t dTn = -(t1_ - t) / (dt_ * dt_);
        return velocity(t) * dt_ * dTn + dCoff;
    }


    ocs2::scalar_t CubicSpline::finalTimeDerivative(ocs2::scalar_t t) const {
        ocs2::scalar_t tn = normalizedTime(t);
        ocs2::scalar_t dCoff = (dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
        ocs2::scalar_t dTn = -(t - t0_) / (dt_ * dt_);
        return velocity(t) * dt_ * dTn + dCoff;
    }


    ocs2::scalar_t CubicSpline::normalizedTime(ocs2::scalar_t t) const {
        return (t - t0_) / dt_;
    }
} // namespace quadruped_nmpc

#pragma once

#include <ocs2_core/Types.h>

namespace anymal_walking_online {
    class CubicSpline {
    public:
        struct Node {
            ocs2::scalar_t time;
            ocs2::scalar_t position;
            ocs2::scalar_t velocity;
        };

        CubicSpline(Node start, Node end);

        ocs2::scalar_t position(ocs2::scalar_t time) const;

        ocs2::scalar_t velocity(ocs2::scalar_t time) const;

        ocs2::scalar_t acceleration(ocs2::scalar_t time) const;

        ocs2::scalar_t startTimeDerivative(ocs2::scalar_t t) const;

        ocs2::scalar_t finalTimeDerivative(ocs2::scalar_t t) const;

    private:
        ocs2::scalar_t normalizedTime(ocs2::scalar_t t) const;

        ocs2::scalar_t t0_;
        ocs2::scalar_t t1_;
        ocs2::scalar_t dt_;

        ocs2::scalar_t c0_;
        ocs2::scalar_t c1_;
        ocs2::scalar_t c2_;
        ocs2::scalar_t c3_;

        ocs2::scalar_t dc0_; // derivative w.r.t. dt_
        ocs2::scalar_t dc1_; // derivative w.r.t. dt_
        ocs2::scalar_t dc2_; // derivative w.r.t. dt_
        ocs2::scalar_t dc3_; // derivative w.r.t. dt_
    };
}

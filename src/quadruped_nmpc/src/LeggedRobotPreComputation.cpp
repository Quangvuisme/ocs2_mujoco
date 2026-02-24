#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/Numerics.h>

#include <quadruped_nmpc/LeggedRobotPreComputation.h>


namespace quadruped_nmpc {
    LeggedRobotPreComputation::LeggedRobotPreComputation(ocs2::PinocchioInterface pinocchioInterface,
                                                         ocs2::CentroidalModelInfo info,
                                                         const SwingTrajectoryPlanner &swingTrajectoryPlanner,
                                                         ModelSettings settings)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          info_(std::move(info)),
          swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
          settings_(std::move(settings)) {
        eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
    }


    LeggedRobotPreComputation *LeggedRobotPreComputation::clone() const {
        return new LeggedRobotPreComputation(*this);
    }


    void LeggedRobotPreComputation::request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t &x, const ocs2::vector_t &u) {
        if (!request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
            return;
        }

        // lambda to set config for normal velocity constraints
        auto eeNormalVelConConfig = [&](size_t footIndex) {
            EndEffectorLinearConstraint::Config config;
            config.b = (ocs2::vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).
                    finished();
            config.Av = (ocs2::matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
            if (!ocs2::numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
                config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(
                    footIndex, t);
                config.Ax = (ocs2::matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
            }
            return config;
        };

        if (request.contains(ocs2::Request::Constraint)) {
            for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
                eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
            }
        }
    }
} // namespace quadruped_nmpc

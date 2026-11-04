#include "roadster_stunt/StuntReference.h"

#include <algorithm>
#include <cmath>

#include <ocs2_core/misc/LinearInterpolation.h>

#include "roadster_stunt/RoadsterDefinition.h"

namespace roadster_stunt {

namespace {

/** Quintic smoothstep from 0 to 1 on [0, 1] with zero end velocity and acceleration: value and d/ds. */
std::pair<ocs2::scalar_t, ocs2::scalar_t> smoothStep(ocs2::scalar_t s) {
  if (s <= 0.0) {
    return {0.0, 0.0};
  }
  if (s >= 1.0) {
    return {1.0, 0.0};
  }
  const ocs2::scalar_t s3 = s * s * s;
  return {s3 * (10.0 - 15.0 * s + 6.0 * s * s), 30.0 * s * s * (1.0 - s) * (1.0 - s)};
}

/** Value and time derivative of a smooth transition from a to b over [t0, t1]. */
std::pair<ocs2::scalar_t, ocs2::scalar_t> blend(ocs2::scalar_t t, ocs2::scalar_t t0, ocs2::scalar_t t1, ocs2::scalar_t a,
                                                ocs2::scalar_t b) {
  const auto [s, ds] = smoothStep((t - t0) / (t1 - t0));
  return {a + (b - a) * s, (b - a) * ds / (t1 - t0)};
}

}  // namespace

ocs2::TargetTrajectories designStuntGuide(const RoadsterParameters& roadster, const StuntParameters& stunt, ocs2::scalar_t dt) {
  const ocs2::scalar_t T = stunt.flightTime_;
  const ocs2::scalar_t tEnter = (stunt.rowStart_ - stunt.rowMargin_) / stunt.launchSpeed_;
  const ocs2::scalar_t tExit = (stunt.rowStart_ + stunt.rowLength_ + stunt.rowMargin_) / stunt.launchSpeed_;
  const ocs2::scalar_t zCruise = stunt.rowHeight_ + stunt.clearanceRadius_ + stunt.apexMargin_;

  const size_t N = static_cast<size_t>(std::ceil(T / dt));
  ocs2::scalar_array_t timeTrajectory;
  ocs2::vector_array_t stateTrajectory, inputTrajectory;
  for (size_t k = 0; k <= N; ++k) {
    const ocs2::scalar_t t = std::min(k * dt, T);
    std::pair<ocs2::scalar_t, ocs2::scalar_t> z;
    if (t < tEnter) {
      z = blend(t, 0.0, tEnter, roadster.rideHeight_, zCruise);
    } else if (t < tExit) {
      z = {zCruise, 0.0};
    } else {
      z = blend(t, tExit, T, zCruise, roadster.touchHeight_ + stunt.flareHeight_);
    }
    const auto roll = blend(t, stunt.rollStartTime_, stunt.rollEndTime_, 0.0, 2.0 * M_PI);

    ocs2::vector_t x = ocs2::vector_t::Zero(STATE_DIM);
    x(Z) = z.first;
    x(VZ) = z.second;
    x(ROLL) = roll.first;
    x(ROLL_RATE) = roll.second;

    timeTrajectory.push_back(t);
    stateTrajectory.push_back(x);
    inputTrajectory.push_back(ocs2::vector_t::Zero(INPUT_DIM));
  }
  // end of the plan: just above the ground, sinking at the landing rate
  stateTrajectory.back()(VZ) = -stunt.landingSinkRate_;
  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

ocs2::TargetTrajectories planToTargets(const ocs2::PrimalSolution& plan, const RoadsterParameters& roadster,
                                       const StuntParameters& stunt, ocs2::scalar_t dt, ocs2::scalar_t tailDuration) {
  const ocs2::scalar_t T = plan.timeTrajectory_.back();
  ocs2::scalar_array_t timeTrajectory;
  ocs2::vector_array_t stateTrajectory, inputTrajectory;

  const size_t N = static_cast<size_t>(std::ceil(T / dt));
  for (size_t k = 0; k <= N; ++k) {
    const ocs2::scalar_t t = std::min(k * dt, T);
    timeTrajectory.push_back(t);
    stateTrajectory.push_back(ocs2::LinearInterpolation::interpolate(t, plan.timeTrajectory_, plan.stateTrajectory_));
    inputTrajectory.push_back(ocs2::LinearInterpolation::interpolate(t, plan.timeTrajectory_, plan.inputTrajectory_));
  }

  // landing tail: level, centered, sinking until the tires touch, hover thrust on the lift jets
  const ocs2::vector_t& xEnd = stateTrajectory.back();
  ocs2::vector_t hover = ocs2::vector_t::Zero(INPUT_DIM);
  hover(LIFT_L) = hover(LIFT_R) = 0.5 * roadster.mass_ * roadster.gravity_ / 1e3;
  const ocs2::scalar_t zFloor = roadster.touchHeight_ - 0.3;
  const size_t M = static_cast<size_t>(std::ceil(tailDuration / dt));
  for (size_t k = 1; k <= M; ++k) {
    const ocs2::scalar_t tau = k * dt;
    ocs2::vector_t x = ocs2::vector_t::Zero(STATE_DIM);
    x(Z) = std::max(xEnd(Z) - stunt.landingSinkRate_ * tau, zFloor);
    x(VZ) = x(Z) > zFloor ? -stunt.landingSinkRate_ : 0.0;
    x(ROLL) = 2.0 * M_PI;
    timeTrajectory.push_back(T + tau);
    stateTrajectory.push_back(x);
    inputTrajectory.push_back(hover);
  }
  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

}  // namespace roadster_stunt

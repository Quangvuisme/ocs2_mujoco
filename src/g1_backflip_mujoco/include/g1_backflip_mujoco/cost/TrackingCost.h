#pragma once

#include <algorithm>

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/StateInputCost.h>

#include "g1_backflip_mujoco/BackflipReferenceManager.h"

namespace g1_backflip {

/**
 * Quadratic tracking of the target state and input trajectories (both interpolated in time), plus a lift-off fade:
 *   L = 0.5 dx' Q dx + 0.5 du' R du + 0.5 (w(t) - 1) w_F' R_F w_F
 * with w_F the foot wrenches (absolute, not their deviation from the target) and w(t) rising towards the next
 * lift-off (STANCE -> FLY switch): liftoffForceWeight at the lift-off, quadratic ramp over liftoffRampTime. Right before a flight phase the optimal contact forces are very sensitive to the
 * momentum (they decide the whole flight); without the ramp the solver puts an impulse into the last stance node.
 * Physically the forces have to fade out at lift-off anyway.
 *
 * While both feet are in the air the state weights QFlight are used instead of Q: the angular momentum is fixed in
 * flight, so tracking the base orientation of the plan on the way only makes the MPC fight the tuck; the joints
 * (which set the rotation speed through the inertia) are tracked instead.
 */
class StateInputTrackingCost final : public ocs2::StateInputCost {
 public:
  StateInputTrackingCost(ocs2::matrix_t Q, ocs2::matrix_t QFlight, ocs2::matrix_t R, size_t numWrenchInputs,
                         const BackflipReferenceManager& referenceManager, ocs2::scalar_t liftoffForceWeight, ocs2::scalar_t liftoffRampTime)
      : Q_(std::move(Q)),
        QFlight_(std::move(QFlight)),
        R_(std::move(R)),
        numWrenchInputs_(numWrenchInputs),
        referenceManagerPtr_(&referenceManager),
        liftoffForceWeight_(liftoffForceWeight),
        liftoffRampTime_(liftoffRampTime) {}

  StateInputTrackingCost* clone() const override { return new StateInputTrackingCost(*this); }

  ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                          const ocs2::TargetTrajectories& targets, const ocs2::PreComputation& /*preComp*/) const override {
    const ocs2::vector_t dx = state - targets.getDesiredState(time);
    const ocs2::vector_t du = input - targets.getDesiredInput(time);
    const ocs2::scalar_t fade = wrenchWeightScale(time) - 1.0;
    const auto w = input.head(numWrenchInputs_);
    return 0.5 * dx.dot(stateWeight(time) * dx) + 0.5 * du.dot(R_ * du) +
           0.5 * fade * w.dot(R_.topLeftCorner(numWrenchInputs_, numWrenchInputs_) * w);
  }

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                       const ocs2::vector_t& input,
                                                                       const ocs2::TargetTrajectories& targets,
                                                                       const ocs2::PreComputation& /*preComp*/) const override {
    const ocs2::vector_t dx = state - targets.getDesiredState(time);
    const ocs2::vector_t du = input - targets.getDesiredInput(time);
    const ocs2::matrix_t& Q = stateWeight(time);
    const ocs2::scalar_t fade = wrenchWeightScale(time) - 1.0;
    const size_t nw = numWrenchInputs_;
    ocs2::ScalarFunctionQuadraticApproximation L;
    L.dfdxx = Q;
    L.dfduu = R_;
    L.dfdux.setZero(input.size(), state.size());
    L.dfdx.noalias() = Q * dx;
    L.dfdu.noalias() = R_ * du;
    L.f = 0.5 * dx.dot(L.dfdx) + 0.5 * du.dot(L.dfdu);
    if (fade > 0.0) {
      const ocs2::matrix_t Rw = fade * R_.topLeftCorner(nw, nw);
      const auto w = input.head(nw);
      L.dfduu.topLeftCorner(nw, nw) += Rw;
      L.dfdu.head(nw) += Rw * w;
      L.f += 0.5 * w.dot(Rw * w);
    }
    return L;
  }

 private:
  StateInputTrackingCost(const StateInputTrackingCost& rhs) = default;

  /** Wrench weight multiplier: 1 far from a lift-off, liftoffForceWeight at the lift-off (quadratic ramp). */
  ocs2::scalar_t wrenchWeightScale(ocs2::scalar_t time) const {
    using ocs2::humanoid::ModeNumber;
    if (liftoffForceWeight_ <= 1.0 || liftoffRampTime_ <= 0.0) {
      return 1.0;
    }
    const auto& schedule = referenceManagerPtr_->getModeSchedule();
    for (size_t i = 0; i < schedule.eventTimes.size(); ++i) {
      const bool isLiftoff = schedule.modeSequence[i] != ModeNumber::FLY && schedule.modeSequence[i + 1] == ModeNumber::FLY;
      const ocs2::scalar_t toLiftoff = schedule.eventTimes[i] - time;
      if (isLiftoff && toLiftoff >= 0.0 && toLiftoff < liftoffRampTime_) {
        const ocs2::scalar_t s = 1.0 - toLiftoff / liftoffRampTime_;
        return 1.0 + (liftoffForceWeight_ - 1.0) * s * s;
      }
    }
    return 1.0;
  }

  const ocs2::matrix_t& stateWeight(ocs2::scalar_t time) const {
    const auto flags = referenceManagerPtr_->getContactFlags(time);
    const bool inAir = std::none_of(flags.begin(), flags.end(), [](bool f) { return f; });
    return inAir ? QFlight_ : Q_;
  }


  const ocs2::matrix_t Q_, QFlight_, R_;
  const size_t numWrenchInputs_;
  const BackflipReferenceManager* referenceManagerPtr_;
  const ocs2::scalar_t liftoffForceWeight_, liftoffRampTime_;
};

/** Quadratic tracking of the target state at the final time. */
class FinalStateTrackingCost final : public ocs2::QuadraticStateCost {
 public:
  explicit FinalStateTrackingCost(ocs2::matrix_t Q) : QuadraticStateCost(std::move(Q)) {}
  FinalStateTrackingCost* clone() const override { return new FinalStateTrackingCost(*this); }

 private:
  FinalStateTrackingCost(const FinalStateTrackingCost& rhs) = default;

  ocs2::vector_t getStateDeviation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                   const ocs2::TargetTrajectories& targets) const override {
    return state - targets.getDesiredState(time);
  }
};

}  // namespace g1_backflip

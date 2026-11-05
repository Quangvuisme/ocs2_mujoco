#pragma once

#include <utility>
#include <vector>

#include <ocs2_core/cost/StateCost.h>

namespace g1_backflip {

/**
 * Left / right symmetry of a sagittal manoeuvre: 0.5 w sum_k (q_l,k - s_k q_r,k)^2 (positions and velocities), with
 * s_k = +1 for pitch joints and -1 for roll / yaw joints (mirrored). Keeps the plan from settling on a lopsided push.
 */
class SymmetryCost final : public ocs2::StateCost {
 public:
  struct Pair {
    size_t left, right;  // state indices
    double sign;
  };

  SymmetryCost(std::vector<Pair> pairs, ocs2::scalar_t weight) : pairs_(std::move(pairs)), weight_(weight) {}
  SymmetryCost* clone() const override { return new SymmetryCost(*this); }

  ocs2::scalar_t getValue(ocs2::scalar_t /*time*/, const ocs2::vector_t& state, const ocs2::TargetTrajectories& /*targets*/,
                          const ocs2::PreComputation& /*preComp*/) const override {
    ocs2::scalar_t c = 0.0;
    for (const auto& p : pairs_) {
      const ocs2::scalar_t e = state(p.left) - p.sign * state(p.right);
      c += e * e;
    }
    return 0.5 * weight_ * c;
  }

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                       const ocs2::TargetTrajectories& targets,
                                                                       const ocs2::PreComputation& preComp) const override {
    ocs2::ScalarFunctionQuadraticApproximation L;
    L.f = getValue(time, state, targets, preComp);
    L.dfdx.setZero(state.size());
    L.dfdxx.setZero(state.size(), state.size());
    for (const auto& p : pairs_) {
      const ocs2::scalar_t e = state(p.left) - p.sign * state(p.right);
      L.dfdx(p.left) += weight_ * e;
      L.dfdx(p.right) -= weight_ * p.sign * e;
      L.dfdxx(p.left, p.left) += weight_;
      L.dfdxx(p.right, p.right) += weight_;
      L.dfdxx(p.left, p.right) -= weight_ * p.sign;
      L.dfdxx(p.right, p.left) -= weight_ * p.sign;
    }
    return L;
  }

 private:
  std::vector<Pair> pairs_;
  ocs2::scalar_t weight_;
};

}  // namespace g1_backflip

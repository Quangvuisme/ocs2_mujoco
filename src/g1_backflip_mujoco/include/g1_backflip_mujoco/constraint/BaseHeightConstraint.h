#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

namespace g1_backflip {

/** Lower bound of the base (pelvis) height: h = z_base - minHeight >= 0 (relaxed barrier). Keeps the crouch realistic. */
class BaseHeightConstraint final : public ocs2::StateConstraint {
 public:
  explicit BaseHeightConstraint(ocs2::scalar_t minHeight) : StateConstraint(ocs2::ConstraintOrder::Linear), minHeight_(minHeight) {}
  BaseHeightConstraint* clone() const override { return new BaseHeightConstraint(*this); }
  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return 1; }
  ocs2::vector_t getValue(ocs2::scalar_t /*time*/, const ocs2::vector_t& state, const ocs2::PreComputation& /*preComp*/) const override {
    ocs2::vector_t h(1);
    h(0) = state(2) - minHeight_;
    return h;
  }
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComp) const override {
    ocs2::VectorFunctionLinearApproximation h;
    h.f = getValue(time, state, preComp);
    h.dfdx = ocs2::matrix_t::Zero(1, state.size());
    h.dfdx(0, 2) = 1.0;
    h.dfdu.setZero(1, 0);
    return h;
  }

 private:
  ocs2::scalar_t minHeight_;
};

}  // namespace g1_backflip

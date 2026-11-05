#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

namespace g1_walking {

/**
 * Zero velocity of one actuated joint: dq_j = 0, always active.
 *
 * Every G1 foot touches the ground at two points (toe, heel) on the sole axis. That line contact
 * cannot resist a moment about the sole axis, so the ankle roll joint is a free direction for the
 * MPC. This constraint keeps the ankle roll joints at their reference (ported from g1_control's
 * FootRollConstraint).
 */
class FootRollConstraint final : public ocs2::StateInputConstraint {
 public:
  /**
   * @param [in] jointIndex : index of the joint among the actuated joints.
   * @param [in] info : The centroidal model information.
   */
  FootRollConstraint(size_t jointIndex, ocs2::CentroidalModelInfo info)
      : StateInputConstraint(ocs2::ConstraintOrder::Linear), jointIndex_(jointIndex), info_(std::move(info)) {}

  ~FootRollConstraint() override = default;
  FootRollConstraint* clone() const override { return new FootRollConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return 1; }

  ocs2::vector_t getValue(ocs2::scalar_t /*time*/, const ocs2::vector_t& /*state*/, const ocs2::vector_t& input,
                          const ocs2::PreComputation& /*preComp*/) const override {
    return input.segment(inputIndex(), 1);
  }

  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::vector_t& input,
                                                                 const ocs2::PreComputation& preComp) const override {
    ocs2::VectorFunctionLinearApproximation approx;
    approx.f = getValue(time, state, input, preComp);
    approx.dfdx = ocs2::matrix_t::Zero(1, state.size());
    approx.dfdu = ocs2::matrix_t::Zero(1, input.size());
    approx.dfdu(0, inputIndex()) = 1.0;
    return approx;
  }

 private:
  FootRollConstraint(const FootRollConstraint& other) = default;

  // input = [contact forces (3 per 3-DoF contact, 6 per 6-DoF contact), joint velocities]
  size_t inputIndex() const { return 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts + jointIndex_; }

  const size_t jointIndex_;
  const ocs2::CentroidalModelInfo info_;
};

}  // namespace g1_walking

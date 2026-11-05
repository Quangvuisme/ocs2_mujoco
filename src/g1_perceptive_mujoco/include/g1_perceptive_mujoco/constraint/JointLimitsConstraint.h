#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateConstraint.h>


namespace g1_perceptive {

/**
 * Joint position limits of the actuated joints as a state constraint (wrap it in a StateSoftConstraint):
 *     q_j - (lower_j + margin) >= 0,   (upper_j - margin) - q_j >= 0
 *
 * The centroidal MPC has no joint limits otherwise. On stairs it plans postures beyond the G1 ankle pitch
 * range (-0.87 rad), the real ankle saturates and the sole tilts.
 */
class JointLimitsConstraint final : public ocs2::StateConstraint {
 public:
  JointLimitsConstraint(ocs2::CentroidalModelInfo info, ocs2::vector_t lower, ocs2::vector_t upper)
      : StateConstraint(ocs2::ConstraintOrder::Linear), info_(std::move(info)), lower_(std::move(lower)), upper_(std::move(upper)) {}

  ~JointLimitsConstraint() override = default;
  JointLimitsConstraint* clone() const override { return new JointLimitsConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return 2 * info_.actuatedDofNum; }

  ocs2::vector_t getValue(ocs2::scalar_t /*time*/, const ocs2::vector_t& state, const ocs2::PreComputation& /*preComp*/) const override {
    const auto n = static_cast<Eigen::Index>(info_.actuatedDofNum);
    const ocs2::vector_t q = state.tail(n);  // centroidal state: [momentum (6), base pose (6), joint angles]
    ocs2::vector_t value(2 * n);
    value.head(n) = q - lower_;
    value.tail(n) = upper_ - q;
    return value;
  }

  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComp) const override {
    const auto n = static_cast<Eigen::Index>(info_.actuatedDofNum);
    auto approx = ocs2::VectorFunctionLinearApproximation::Zero(2 * n, state.size(), 0);
    approx.f = getValue(time, state, preComp);
    approx.dfdx.block(0, state.size() - n, n, n).setIdentity();
    approx.dfdx.block(n, state.size() - n, n, n) = -ocs2::matrix_t::Identity(n, n);
    return approx;
  }

 private:
  JointLimitsConstraint(const JointLimitsConstraint& other) = default;

  const ocs2::CentroidalModelInfo info_;
  const ocs2::vector_t lower_;
  const ocs2::vector_t upper_;
};

}  // namespace g1_perceptive

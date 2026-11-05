#pragma once

#include <memory>
#include <string>

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>

namespace g1_backflip {

/**
 * Joint torque limits of the whole-body model as inequalities (used with a relaxed barrier penalty):
 *   h = [tau_max - tau; tau_max + tau] >= 0,  tau = inverse dynamics of the state, the foot wrenches and the joint
 *   accelerations (wb_humanoid_mpc's computeJointTorques, differentiated with CppAD).
 * The limits are the parameters of the generated library: changing them does not need a regeneration.
 */
class JointTorqueLimitConstraint final : public ocs2::StateInputConstraintCppAd {
 public:
  JointTorqueLimitConstraint(ocs2::vector_t torqueLimits, const ocs2::PinocchioInterface& pinocchioInterface,
                             const ocs2::humanoid::WBAccelMpcRobotModel<ocs2::ad_scalar_t>& mpcRobotModel,
                             const ocs2::humanoid::ModelSettings& modelSettings);
  ~JointTorqueLimitConstraint() override = default;
  JointTorqueLimitConstraint* clone() const override { return new JointTorqueLimitConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return 2 * torqueLimits_.size(); }
  ocs2::vector_t getParameters(ocs2::scalar_t /*time*/, const ocs2::PreComputation& /*preComp*/) const override { return torqueLimits_; }

 private:
  JointTorqueLimitConstraint(const JointTorqueLimitConstraint& other);
  ocs2::ad_vector_t constraintFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override;

  ocs2::vector_t torqueLimits_;
  mutable ocs2::PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  std::unique_ptr<ocs2::humanoid::WBAccelMpcRobotModel<ocs2::ad_scalar_t>> mpcRobotModelPtr_;
};

}  // namespace g1_backflip

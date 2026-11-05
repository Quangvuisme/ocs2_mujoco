#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include "g1_backflip_mujoco/constraint/JointTorqueLimitConstraint.h"

#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>

namespace g1_backflip {

JointTorqueLimitConstraint::JointTorqueLimitConstraint(ocs2::vector_t torqueLimits, const ocs2::PinocchioInterface& pinocchioInterface,
                                                       const ocs2::humanoid::WBAccelMpcRobotModel<ocs2::ad_scalar_t>& mpcRobotModel,
                                                       const ocs2::humanoid::ModelSettings& modelSettings)
    : StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear),
      torqueLimits_(std::move(torqueLimits)),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelPtr_(mpcRobotModel.clone()) {
  initialize(mpcRobotModel.getStateDim(), mpcRobotModel.getInputDim(), torqueLimits_.size(), "joint_torque_limits",
             modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
}

JointTorqueLimitConstraint::JointTorqueLimitConstraint(const JointTorqueLimitConstraint& other)
    : StateInputConstraintCppAd(other),
      torqueLimits_(other.torqueLimits_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelPtr_(other.mpcRobotModelPtr_->clone()) {}

ocs2::ad_vector_t JointTorqueLimitConstraint::constraintFunction(ocs2::ad_scalar_t /*time*/, const ocs2::ad_vector_t& state,
                                                                 const ocs2::ad_vector_t& input, const ocs2::ad_vector_t& parameters) const {
  const ocs2::ad_vector_t tau =
      ocs2::humanoid::computeJointTorques<ocs2::ad_scalar_t>(state, input, pinocchioInterfaceCppAd_, *mpcRobotModelPtr_);
  ocs2::ad_vector_t h(2 * tau.size());
  h << parameters - tau, parameters + tau;
  return h;
}

}  // namespace g1_backflip

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>

#include "g1_backflip_mujoco/BackflipReferenceManager.h"

namespace g1_backflip {

/** Initial guess: the input of the target trajectories with the wrenches of feet in the air set to zero; state held. */
class BackflipInitializer final : public ocs2::Initializer {
 public:
  BackflipInitializer(const BackflipReferenceManager& referenceManager, size_t numContacts)
      : referenceManagerPtr_(&referenceManager), numContacts_(numContacts) {}
  BackflipInitializer* clone() const override { return new BackflipInitializer(*this); }
  void compute(ocs2::scalar_t time, const ocs2::vector_t& state, ocs2::scalar_t /*nextTime*/, ocs2::vector_t& input,
               ocs2::vector_t& nextState) override {
    input = referenceManagerPtr_->getTargetTrajectories().getDesiredInput(time);
    const auto flags = referenceManagerPtr_->getContactFlags(time);
    for (size_t i = 0; i < numContacts_; ++i) {
      if (!flags[i]) {
        input.segment<6>(6 * i).setZero();
      }
    }
    nextState = state;
  }

 private:
  BackflipInitializer(const BackflipInitializer& other) = default;
  const BackflipReferenceManager* referenceManagerPtr_;
  size_t numContacts_;
};

/**
 * Optimal control problem of the backflip on the whole-body dynamics model of wb_humanoid_mpc (Unitree G1, 29-DoF
 * description, 23 MPC joints: the wrists are fixed as upstream).
 *
 *   dynamics  : full rigid-body dynamics, inputs = foot wrenches + joint accelerations (WBAccelDynamicsAD, CppAD)
 *   cost      : tracking of the target state / input trajectories (TrackingCost.h), final state tracking
 *   equality  : zero wrench of a foot in the air; zero foot acceleration (with velocity / pose feedback) in stance
 *   soft      : friction cones, centre of pressure inside the foot rectangle, joint limits, foot / knee collision
 *               (all from humanoid_common_mpc's HumanoidCostConstraintFactory)
 *
 * Unlike walking (WBMpcInterface) there is no swing-foot height constraint: in the flight phase both feet move freely.
 * The free-flyer is the URDF root "flip_base" (pelvis turned +90 deg about z): the backflip is a rotation about its
 * x axis, the last ZYX Euler angle, so the Euler parameterisation stays regular through the full turn.
 */
class G1WbBackflipInterface {
 public:
  /** @param solverPrefix : block of task.info with the multiple_shooting / mpc settings ("plan" or "online"). */
  G1WbBackflipInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& solverPrefix);

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const { return *problemPtr_; }
  const BackflipInitializer& getInitializer() const { return *initializerPtr_; }
  const std::shared_ptr<BackflipReferenceManager>& getReferenceManagerPtr() const { return referenceManagerPtr_; }

  const ocs2::humanoid::ModelSettings& modelSettings() const { return *modelSettingsPtr_; }
  const ocs2::sqp::Settings& sqpSettings() const { return sqpSettings_; }
  const ocs2::mpc::Settings& mpcSettings() const { return mpcSettings_; }
  ocs2::PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t>& getMpcRobotModel() { return *mpcRobotModelPtr_; }
  const std::string& taskFile() const { return taskFile_; }
  ocs2::scalar_t totalMass() const { return totalMass_; }

 private:
  std::string taskFile_;
  std::unique_ptr<ocs2::humanoid::ModelSettings> modelSettingsPtr_;
  ocs2::sqp::Settings sqpSettings_;
  ocs2::mpc::Settings mpcSettings_;
  ocs2::scalar_t totalMass_ = 0.0;

  std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
  std::unique_ptr<ocs2::humanoid::WBAccelMpcRobotModel<ocs2::scalar_t>> mpcRobotModelPtr_;
  std::unique_ptr<ocs2::humanoid::WBAccelMpcRobotModel<ocs2::ad_scalar_t>> mpcRobotModelADPtr_;
  std::unique_ptr<ocs2::OptimalControlProblem> problemPtr_;
  std::shared_ptr<BackflipReferenceManager> referenceManagerPtr_;
  std::unique_ptr<BackflipInitializer> initializerPtr_;
};

}  // namespace g1_backflip

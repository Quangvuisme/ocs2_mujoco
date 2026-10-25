#pragma once

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

// Roadster
#include "roadster_stunt/RoadsterDefinition.h"
#include "roadster_stunt/RoadsterParameters.h"

namespace roadster_stunt {

/**
 * RoadsterInterface - the OCS2 optimal control problem of the flying car.
 *
 *   cost:        0.5 |x - x_ref|_Q^2 + 0.5 |u - u_ref|_R^2,  final 0.5 |x - x_ref|_Qf^2
 *   dynamics:    planar roll model (RoadsterDynamics)
 *   soft constr: jet group limits 0 <= u <= u_max, clearance over the parked row (relaxed barriers)
 *
 * The same problem is used twice with different weights and solver settings, picked by `stage`:
 *   "plan" - offline SLQ over the whole flight against a hand-designed guide,
 *   "nmpc" - receding-horizon SLQ in the MuJoCo loop, tracking the plan.
 */
class RoadsterInterface final : public ocs2::RobotInterface {
 public:
  RoadsterInterface(const std::string& taskFile, const std::string& stage, bool verbose);

  ~RoadsterInterface() override = default;

  const ocs2::vector_t& getInitialState() const { return initialState_; }

  const RoadsterParameters& roadsterParameters() const { return roadsterParameters_; }

  const StuntParameters& stuntParameters() const { return stuntParameters_; }

  ocs2::ddp::Settings& ddpSettings() { return ddpSettings_; }

  ocs2::mpc::Settings& mpcSettings() { return mpcSettings_; }

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }

  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  const ocs2::RolloutBase& getRollout() const { return *rolloutPtr_; }

  const ocs2::Initializer& getInitializer() const override { return *initializerPtr_; }

  /** Replaces the default (hover thrust) initial guess, e.g. with the offline plan for the NMPC. */
  void setInitializer(std::unique_ptr<ocs2::Initializer> initializerPtr) { initializerPtr_ = std::move(initializerPtr); }

 private:
  RoadsterParameters roadsterParameters_;
  StuntParameters stuntParameters_;

  ocs2::ddp::Settings ddpSettings_;
  ocs2::mpc::Settings mpcSettings_;

  ocs2::OptimalControlProblem problem_;
  std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
  std::unique_ptr<ocs2::Initializer> initializerPtr_;

  ocs2::vector_t initialState_{STATE_DIM};
};

}  // namespace roadster_stunt

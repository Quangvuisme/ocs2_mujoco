#pragma once

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

// Quadrotor
#include "quadrotor_offline/QuadrotorParameters.h"
#include "quadrotor_offline/QuadrotorDefinition.h"

namespace quadrotor_offline {

/**
 * QuadrotorInterface - Main interface for the quadrotor OCS2 problem
 */
class QuadrotorInterface final : public ocs2::RobotInterface {
 public:
  /**
   * Constructor
   *
   * @param taskFile: Path to the configuration file
   * @param libraryFolder: Path to auto-generated library folder
   * @param verbose: Print loaded settings
   */
  QuadrotorInterface(const std::string& taskFile, 
                     const std::string& libraryFolder, 
                     bool verbose);

  /** Destructor */
  ~QuadrotorInterface() override = default;

  const ocs2::vector_t& getInitialState() { return initialState_; }

  const ocs2::vector_t& getInitialTarget() { return xFinal_; }

  ocs2::ddp::Settings& ddpSettings() { return ddpSettings_; }

  ocs2::mpc::Settings& mpcSettings() { return mpcSettings_; }

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }

  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override { 
    return referenceManagerPtr_; 
  }

  const ocs2::RolloutBase& getRollout() const { return *rolloutPtr_; }

  const ocs2::Initializer& getInitializer() const override { return *operatingPointPtr_; }

 private:
  ocs2::ddp::Settings ddpSettings_;
  ocs2::mpc::Settings mpcSettings_;

  ocs2::OptimalControlProblem problem_;
  std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
  std::unique_ptr<ocs2::Initializer> operatingPointPtr_;

  ocs2::vector_t initialState_{STATE_DIM};
  ocs2::vector_t xFinal_{STATE_DIM};
};

}  // namespace quadrotor_offline

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "CartPoleParameters.h"
#include "CartpoleDynamicsPinocchio.h"

namespace ocs2_cartpole {

constexpr size_t STATE_DIM = 4;
constexpr size_t INPUT_DIM = 1;

/**
 * CartPole Interface for OCS2
 * Based on ocs2_cartpole example but with UDP integration
 */
class CartPoleInterface {
 public:
  /**
   * Constructor
   * @param taskFile Path to task configuration file  
   * @param libraryFolder Path to generate CppAD library
   * @param verbose Print debug information
   */
  CartPoleInterface(const std::string& taskFile, 
                    const std::string& libraryFolder, 
                    bool verbose);

  ~CartPoleInterface() = default;

  const ocs2::vector_t& getInitialState() { return initialState_; }

  const ocs2::vector_t& getInitialTarget() { return xFinal_; }

  ocs2::ddp::Settings& ddpSettings() { return ddpSettings_; }

  ocs2::mpc::Settings& mpcSettings() { return mpcSettings_; }

  ocs2::OptimalControlProblem& optimalControlProblem() { return problem_; }
  
  const ocs2::OptimalControlProblem& getOptimalControlProblem() const { 
    return problem_; 
  }

  const ocs2::RolloutBase& getRollout() const { return *rolloutPtr_; }

  const ocs2::Initializer& getInitializer() const { 
    return *cartPoleInitializerPtr_; 
  }

  const CartPoleParameters& getParameters() const { return parameters_; }

 private:
  CartPoleParameters parameters_;
  ocs2::ddp::Settings ddpSettings_;
  ocs2::mpc::Settings mpcSettings_;

  ocs2::OptimalControlProblem problem_;

  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
  std::unique_ptr<ocs2::Initializer> cartPoleInitializerPtr_;

  ocs2::vector_t initialState_{STATE_DIM};
  ocs2::vector_t xFinal_{STATE_DIM};
};

}  // namespace ocs2_cartpole

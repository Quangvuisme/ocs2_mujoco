#include "ocs2_cartpole/CartPoleInterface.h"
#include "ocs2_cartpole/CartpoleDynamicsPinocchio.h"

#include <iostream>
#include <memory>
#include <filesystem>

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <pinocchio/parsers/urdf.hpp>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2_cartpole {

CartPoleInterface::CartPoleInterface(const std::string& taskFile,
                                     const std::string& libraryFolder,
                                     bool verbose) {
  // Check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[CartPoleInterface] Loading task file: " << taskFilePath << "\n";
  } else {
    throw std::invalid_argument("[CartPoleInterface] Task file not found: " +
                                taskFilePath.string());
  }

  // Create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[CartPoleInterface] Generated library path: " << libraryFolderPath
            << "\n";

  // Load initial condition and target
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  ocs2::loadData::loadEigenMatrix(taskFile, "x_final", xFinal_);
  if (verbose) {
    std::cerr << "x_init:   " << initialState_.transpose() << "\n";
    std::cerr << "x_final:  " << xFinal_.transpose() << "\n";
  }

  // DDP-MPC settings
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc", verbose);

  /*
   * Optimal control problem
   */
  // Cost
  ocs2::matrix_t Q(STATE_DIM, STATE_DIM);
  ocs2::matrix_t R(INPUT_DIM, INPUT_DIM);
  ocs2::matrix_t Qf(STATE_DIM, STATE_DIM);
  ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q);
  ocs2::loadData::loadEigenMatrix(taskFile, "R", R);
  ocs2::loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
  if (verbose) {
    std::cerr << "Q:  \n" << Q << "\n";
    std::cerr << "R:  \n" << R << "\n";
    std::cerr << "Q_final:\n" << Qf << "\n";
  }

  problem_.costPtr->add("cost",
                        std::make_unique<ocs2::QuadraticStateInputCost>(Q, R));
  problem_.finalCostPtr->add("finalCost",
                             std::make_unique<ocs2::QuadraticStateCost>(Qf));

  // Dynamics
  parameters_.loadSettings(taskFile, "cartpole_parameters", verbose);
  
  // Try to use Pinocchio dynamics with URDF, fallback to hardcoded if URDF not found
  std::string urdfPath = "src/ocs2_cartpole_upright/urdf/cartpole.urdf";
  
  // Check if we're running from workspace root (typical case)
  if (!std::filesystem::exists(urdfPath)) {
    // Try relative to current working directory
    urdfPath = "robots/cartpole/cartpole.urdf";
  }
  
  if (std::filesystem::exists(urdfPath)) {
    if (verbose) {
      std::cerr << "[CartPoleInterface] Using OCS2 PinocchioInterface with URDF: " << urdfPath << "\n";
    }
    try {
      // Load URDF using Pinocchio (fixed base - no FreeFlyer)
      // Joint order from URDF: world_base (fixed) -> slider (prismatic) -> hinge (revolute)
      // Pinocchio ignores fixed joints, so q = [x_slider, theta_hinge]
      pinocchio::Model pinocchioModel;
      pinocchio::urdf::buildModel(urdfPath, pinocchioModel);
      
      if (verbose) {
        std::cerr << "[CartPoleInterface] Pinocchio model loaded: nq=" << pinocchioModel.nq 
                  << ", nv=" << pinocchioModel.nv << "\n";
      }
      
      // Create OCS2 PinocchioInterface
      auto pinocchioInterface = std::make_shared<ocs2::PinocchioInterface>(pinocchioModel);
      problem_.dynamicsPtr = std::make_unique<CartpoleDynamicsPinocchio>(*pinocchioInterface, libraryFolder, verbose);
    } catch (const std::exception& e) {
      throw std::runtime_error("[CartPoleInterface] Failed to load URDF: " + std::string(e.what()) + 
                               "\nURDF required at: " + urdfPath);
    }
  } else {
    throw std::invalid_argument("[CartPoleInterface] URDF file not found at: " + urdfPath);
  }

  // Rollout
  auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout", verbose);
  rolloutPtr_ = std::make_unique<ocs2::TimeTriggeredRollout>(*problem_.dynamicsPtr,
                                                              rolloutSettings);

  // Constraints
  auto getPenalty = [&]() {
    using penalty_type = ocs2::augmented::SlacknessSquaredHingePenalty;
    penalty_type::Config boundsConfig;
    ocs2::loadData::loadPenaltyConfig(taskFile, "bounds_penalty_config",
                                      boundsConfig, verbose);
    return penalty_type::create(boundsConfig);
  };

  auto getConstraint = [&]() {
    constexpr size_t numIneqConstraint = 2;
    const ocs2::vector_t e =
        (ocs2::vector_t(numIneqConstraint) << parameters_.maxInput_,
         parameters_.maxInput_)
            .finished();
    const ocs2::vector_t D =
        (ocs2::vector_t(numIneqConstraint) << 1.0, -1.0).finished();
    const ocs2::matrix_t C = ocs2::matrix_t::Zero(numIneqConstraint, STATE_DIM);
    return std::make_unique<ocs2::LinearStateInputConstraint>(e, C, D);
  };

  problem_.inequalityLagrangianPtr->add("InputLimits",
                                        ocs2::create(getConstraint(), getPenalty()));

  // Initialization
  cartPoleInitializerPtr_ = std::make_unique<ocs2::DefaultInitializer>(INPUT_DIM);
}

}  // namespace ocs2_cartpole

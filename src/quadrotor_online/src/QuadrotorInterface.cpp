#include "quadrotor_online/QuadrotorInterface.h"

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/misc/LoadData.h>

#include <iostream>
#include <memory>
#include <string>

#include "quadrotor_online/QuadrotorDynamics.h"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace quadrotor_online {

QuadrotorInterface::QuadrotorInterface(const std::string& taskFile,
                                       const std::string& libraryFolder, 
                                       bool verbose) {
  // Check that task file exists
  if (const boost::filesystem::path taskFilePath(taskFile); exists(taskFilePath)) {
    std::cerr << "[QuadrotorInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[QuadrotorInterface] Task file not found: " + taskFilePath.string());
  }
  
  // Create library folder if it does not exist
  const boost::filesystem::path libraryFolderPath(libraryFolder);
  create_directories(libraryFolderPath);
  std::cerr << "[QuadrotorInterface] Generated library path: " << libraryFolderPath << std::endl;

  // Load initial condition
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  ocs2::loadData::loadEigenMatrix(taskFile, "x_final", xFinal_);
  
  if (verbose) {
    std::cerr << "x_init:   " << initialState_.transpose() << "\n";
    std::cerr << "x_final:  " << xFinal_.transpose() << "\n";
  }

  // Solver settings
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc");

  // Reference manager
  referenceManagerPtr_ = std::make_shared<ocs2::ReferenceManager>();

  // Cost function
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

  problem_.costPtr->add("cost", std::make_unique<ocs2::QuadraticStateInputCost>(Q, R));
  problem_.finalCostPtr->add("finalCost", std::make_unique<ocs2::QuadraticStateCost>(Qf));

  // Dynamics and parameters
  quadrotorParams_ = loadSettings(taskFile, "QuadrotorParameters", verbose);
  problem_.dynamicsPtr = std::make_unique<QuadrotorDynamics>(quadrotorParams_);

  // Rollout
  const auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_ = std::make_unique<ocs2::TimeTriggeredRollout>(*problem_.dynamicsPtr, rolloutSettings);

  // Initialization - hover thrust
  ocs2::vector_t initialInput = ocs2::vector_t::Zero(INPUT_DIM);
  initialInput(0) = quadrotorParams_.quadrotorMass_ * quadrotorParams_.gravity_;
  operatingPointPtr_ = std::make_unique<ocs2::OperatingPoints>(initialState_, initialInput);
}

}  // namespace quadrotor_online

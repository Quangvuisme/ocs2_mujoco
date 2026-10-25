#include "roadster_stunt/RoadsterInterface.h"

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include <iostream>
#include <memory>
#include <string>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include "roadster_stunt/ClearanceConstraint.h"
#include "roadster_stunt/RoadsterDynamics.h"

namespace roadster_stunt {

namespace {

ocs2::RelaxedBarrierPenalty::Config loadBarrier(const std::string& taskFile, const std::string& field, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  ocs2::RelaxedBarrierPenalty::Config config;
  ocs2::loadData::loadPtreeValue(pt, config.mu, field + ".mu", verbose);
  ocs2::loadData::loadPtreeValue(pt, config.delta, field + ".delta", verbose);
  return config;
}

}  // namespace

RoadsterInterface::RoadsterInterface(const std::string& taskFile, const std::string& stage, bool verbose) {
  // Check that task file exists
  if (const boost::filesystem::path taskFilePath(taskFile); exists(taskFilePath)) {
    std::cerr << "[RoadsterInterface] Loading task file: " << taskFilePath << " (stage: " << stage << ")" << std::endl;
  } else {
    throw std::invalid_argument("[RoadsterInterface] Task file not found: " + taskFilePath.string());
  }

  roadsterParameters_ = loadRoadsterParameters(taskFile, "roadster", verbose);
  stuntParameters_ = loadStuntParameters(taskFile, "stunt", verbose);

  // On the road at the launch line: centered, at ride height, level
  initialState_.setZero();
  initialState_(Z) = roadsterParameters_.rideHeight_;

  // Solver settings
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile, stage + ".ddp", verbose);
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc", verbose);

  // Reference manager
  referenceManagerPtr_ = std::make_shared<ocs2::ReferenceManager>();

  // Cost function
  ocs2::matrix_t Q(STATE_DIM, STATE_DIM);
  ocs2::matrix_t R(INPUT_DIM, INPUT_DIM);
  ocs2::matrix_t Qf(STATE_DIM, STATE_DIM);
  ocs2::loadData::loadEigenMatrix(taskFile, stage + ".Q", Q);
  ocs2::loadData::loadEigenMatrix(taskFile, stage + ".R", R);
  ocs2::loadData::loadEigenMatrix(taskFile, stage + ".Q_final", Qf);
  if (verbose) {
    std::cerr << "Q:  \n" << Q << "\n";
    std::cerr << "R:  \n" << R << "\n";
    std::cerr << "Q_final:\n" << Qf << "\n";
  }
  problem_.costPtr->add("tracking", std::make_unique<ocs2::QuadraticStateInputCost>(Q, R));
  problem_.finalCostPtr->add("finalTracking", std::make_unique<ocs2::QuadraticStateCost>(Qf));

  // Jet group limits [kN]: 0 <= lift_L, lift_R <= 2 liftMax and 0 <= roof_L, roof_R <= 2 roofMax, scaled per stage
  // (the plan keeps some thrust in reserve for the NMPC)
  const auto boxConfig = loadBarrier(taskFile, "constraints.jetLimits", verbose);
  ocs2::scalar_t limitScale = 1.0;
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    ocs2::loadData::loadPtreeValue(pt, limitScale, stage + ".jetLimitScale", verbose);
  }
  const ocs2::scalar_t liftLimit = 2e-3 * limitScale * roadsterParameters_.liftMax_;
  const ocs2::scalar_t roofLimit = 2e-3 * limitScale * roadsterParameters_.roofMax_;
  const ocs2::scalar_t upperBound[INPUT_DIM] = {liftLimit, liftLimit, roofLimit, roofLimit};
  std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint> inputLimits(INPUT_DIM);
  for (size_t i = 0; i < INPUT_DIM; ++i) {
    inputLimits[i].index = i;
    inputLimits[i].lowerBound = 0.0;
    inputLimits[i].upperBound = upperBound[i];
    inputLimits[i].penaltyPtr = std::make_unique<ocs2::RelaxedBarrierPenalty>(boxConfig);
  }
  problem_.softConstraintPtr->add(
      "jetLimits", std::make_unique<ocs2::StateInputSoftBoxConstraint>(std::vector<ocs2::StateInputSoftBoxConstraint::BoxConstraint>{},
                                                                       std::move(inputLimits)));

  // Clearance over the parked row
  const auto clearanceConfig = loadBarrier(taskFile, "constraints.clearance", verbose);
  problem_.stateSoftConstraintPtr->add(
      "clearance", std::make_unique<ocs2::StateSoftConstraint>(std::make_unique<ClearanceConstraint>(roadsterParameters_, stuntParameters_),
                                                               std::make_unique<ocs2::RelaxedBarrierPenalty>(clearanceConfig)));

  // Dynamics
  problem_.dynamicsPtr = std::make_unique<RoadsterDynamics>(roadsterParameters_);

  // Rollout
  const auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout", verbose);
  rolloutPtr_ = std::make_unique<ocs2::TimeTriggeredRollout>(*problem_.dynamicsPtr, rolloutSettings);

  // Initialization: hover thrust on the lift jets
  ocs2::vector_t hover = ocs2::vector_t::Zero(INPUT_DIM);
  hover(LIFT_L) = hover(LIFT_R) = 0.5 * roadsterParameters_.mass_ * roadsterParameters_.gravity_ / 1e3;
  initializerPtr_ = std::make_unique<ocs2::OperatingPoints>(initialState_, hover);
}

}  // namespace roadster_stunt

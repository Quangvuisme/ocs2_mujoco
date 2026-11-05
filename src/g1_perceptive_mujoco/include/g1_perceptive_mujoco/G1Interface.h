#pragma once

#include <memory>
#include <string>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "g1_perceptive_mujoco/common/ModelSettings.h"
#include "g1_perceptive_mujoco/initialization/LeggedRobotInitializer.h"
#include "g1_perceptive_mujoco/reference_manager/PerceptiveReferenceManager.h"
#include "g1_perceptive_mujoco/terrain/TerrainMap.h"

namespace g1_perceptive {

/**
 * Centroidal-model MPC problem of the Unitree G1 (12 leg joints), 4 point contacts (toe + heel per foot):
 *   cost      : base / joint tracking (Q, R)
 *   dynamics  : single rigid body centroidal dynamics (CppAD)
 *   equality  : zero force in swing, zero contact velocity in stance, swing foot z-velocity profile,
 *               zero ankle roll velocity (the toe-heel line contact gives no roll support)
 *   inequality: friction cones (relaxed barrier)
 *   perceptive: toe / heel inside the foot placement regions of their footholds, swing toe / heel and
 *               shins above the elevation map (relaxed barriers)
 */
class G1Interface final : public ocs2::RobotInterface {
 public:
  G1Interface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
              std::shared_ptr<TerrainMapBuffer> terrainBuffer);

  ~G1Interface() override = default;

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ocs2::ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const ocs2::mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const ocs2::rollout::Settings& rolloutSettings() const { return rolloutSettings_; }

  const ocs2::vector_t& getInitialState() const { return initialState_; }
  const ocs2::RolloutBase& getRollout() const { return *rolloutPtr_; }
  ocs2::PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const ocs2::CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }

  const LeggedRobotInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }
  std::shared_ptr<PerceptiveReferenceManager> getPerceptiveReferenceManagerPtr() const { return referenceManagerPtr_; }

 private:
  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                  const std::string& referenceFile, bool verbose);

  std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;
  ocs2::matrix_t initializeInputCostWeight(const std::string& taskFile, const ocs2::CentroidalModelInfo& info);
  std::unique_ptr<ocs2::StateInputCost> getBaseTrackingCost(const std::string& taskFile, const ocs2::CentroidalModelInfo& info,
                                                             bool verbose);
  std::pair<ocs2::scalar_t, ocs2::RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile,
                                                                                         bool verbose) const;

  ModelSettings modelSettings_;
  ocs2::ddp::Settings ddpSettings_;
  ocs2::mpc::Settings mpcSettings_;
  ocs2::rollout::Settings rolloutSettings_;
  ocs2::scalar_t contactHeight_ = 0.0;
  ocs2::scalar_t jointLimitMargin_ = -1.0;  // < 0: no joint limits in the MPC
  ocs2::scalar_t jointLimitMu_ = 0.1;
  ocs2::scalar_t jointLimitDelta_ = 0.05;
  PerceptionSettings perceptionSettings_;
  std::shared_ptr<TerrainMapBuffer> terrainBuffer_;

  std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
  ocs2::CentroidalModelInfo centroidalModelInfo_;

  std::unique_ptr<ocs2::OptimalControlProblem> problemPtr_;
  std::shared_ptr<PerceptiveReferenceManager> referenceManagerPtr_;
  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
  std::unique_ptr<LeggedRobotInitializer> initializerPtr_;

  ocs2::vector_t initialState_;
};

}  // namespace g1_perceptive

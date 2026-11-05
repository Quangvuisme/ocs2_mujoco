#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "g1_perceptive_mujoco/controller/LegPostureIk.h"
#include "g1_perceptive_mujoco/reference_manager/SwitchedModelReferenceManager.h"
#include "g1_perceptive_mujoco/terrain/FootholdPlanner.h"
#include "g1_perceptive_mujoco/terrain/TerrainMap.h"

namespace g1_perceptive {

struct PerceptionSettings {
  // base reference
  int numTargetNodes = 11;      // target trajectory is resampled on this many nodes over the horizon
  double basePitchGain = 0.0;   // fraction of the terrain slope the base follows (humanoid: keep the pelvis level)
  double basePitchGainDown = 0.0;  // same, when the slope goes down in the walking direction (nose down)
  double baseRollGain = 0.0;
  double maxBasePitch = 0.35;   // [rad]
  double maxBaseRoll = 0.20;    // [rad]
  bool jointReferenceFromIk = true;  // joint posture reference by leg IK onto the terrain (else: default stance)
  // feet
  double footFrameHeight = 0.005;  // [m] height of the contact frames above the contact surface
  // swing collision avoidance (soft constraint, active away from lift-off / touch-down)
  double swingClearance = 0.01;           // [m] margin above the inflated terrain
  double swingConstraintLiftOffMargin = 0.03;  // [s]
  double swingConstraintTouchDownMargin = 0.06;  // [s]
  // penalties
  bool enableFootPlacement = true;
  bool enableSwingClearance = true;
  bool enableShankClearance = true;       // keep the lower legs off step noses / risers
  double shankRadius = 0.05;              // [m] shank radius + margin
  // scale of the horizontal map gradient in the clearance constraints: 1 pushes swing feet / shins sideways off
  // obstacles, 0 only up (at a drop the gradient otherwise pulls the swing foot forward over the edge)
  double clearanceGradientScale = 1.0;
  double ankleClearance = 0.04;           // [m] swing ankle (pitch joint) above the sole frames + margin
  double shankClearanceMu = 1e-2;
  double shankClearanceDelta = 5e-3;
  double footPlacementMu = 1e-2;
  double footPlacementDelta = 5e-3;
  double swingClearanceMu = 1e-2;
  double swingClearanceDelta = 5e-3;
  // foothold planning
  FootholdPlannerConfig foothold;
};

PerceptionSettings loadPerceptionSettings(const std::string& file, const std::string& prefix, bool verbose);

/**
 * Reference manager of the perceptive G1 MPC. Before every solve it
 *  1. takes the latest elevation map from the TerrainMapBuffer (filled from MuJoCo),
 *  2. lifts the commanded pelvis trajectory onto the terrain (the commanded z is a height above the
 *     terrain) and sets the joint posture reference by leg IK (feet flat on the terrain under them),
 *  3. selects a foothold for every future stance phase of each foot: toe and heel must both land on
 *     steppable cells on one level, each gets its own convex region,
 *  4. plans terrain-aware swing height trajectories (lift-off / touch-down heights + obstacle apex).
 *
 * Contacts: {0, 1} = left / right toe, {2, 3} = left / right heel; foot f has the contacts f and f + 2.
 *
 * Everything the constraints read during the solve (map snapshot, footholds) is written here only,
 * i.e. on the MPC thread before the solve starts.
 */
class PerceptiveReferenceManager final : public SwitchedModelReferenceManager {
 public:
  PerceptiveReferenceManager(ocs2::CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                             const ocs2::PinocchioInterface& pinocchioInterface, const ocs2::vector_t& nominalState,
                             PerceptionSettings settings, std::shared_ptr<TerrainMapBuffer> terrainBuffer);

  ~PerceptiveReferenceManager() override = default;

  /**
   * Thread-safe. Base trajectory command: same layout as the MPC state, but index 8 (base z) is
   * the height above the terrain. Re-applied (and re-lifted onto the newest map) at every solve.
   */
  void setCommandTrajectories(ocs2::TargetTrajectories command);

  const PerceptionSettings& settings() const { return settings_; }

  /** Map used by the current solve (never null). */
  const TerrainMap& terrain() const { return *terrain_; }

  /** Foothold of `leg` for the phase active at `time` (nullptr while the leg swings). */
  const Foothold* getFoothold(size_t leg, ocs2::scalar_t time) const;

  /** True during future stance phases that have a foothold region. */
  bool isFootPlacementActive(size_t leg, ocs2::scalar_t time) const;

  /** Copy of the current plan for visualisation (thread-safe). */
  struct Plan {
    double time = 0.0;
    std::vector<std::pair<size_t, Foothold>> footholds;  // future footholds within the horizon
    std::vector<Eigen::Vector3d> baseTrajectory;
    std::shared_ptr<const TerrainMap> terrain;
  };
  Plan getPlan() const;

 protected:
  void modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t& initState,
                        ocs2::TargetTrajectories& targetTrajectories, ocs2::ModeSchedule& modeSchedule) override;

 private:
  ocs2::TargetTrajectories liftCommandOntoTerrain(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
                                                  const ocs2::vector_t& initState);
  void planFootholds(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t& initState,
                     const ocs2::TargetTrajectories& targetTrajectories, const ocs2::ModeSchedule& modeSchedule);
  void planSwingTrajectories(const ocs2::ModeSchedule& modeSchedule);
  feet_array_t<vector3_t> computeFeetPositions(const ocs2::vector_t& state);
  Eigen::Vector2d nominalFootXY(size_t leg, const ocs2::vector_t& state) const;

  const ocs2::CentroidalModelInfo info_;
  ocs2::PinocchioInterface pinocchioInterface_;
  const PerceptionSettings settings_;
  std::shared_ptr<TerrainMapBuffer> terrainBuffer_;
  std::shared_ptr<const TerrainMap> flatTerrain_;
  std::shared_ptr<const TerrainMap> terrain_;
  FootholdPlanner footholdPlanner_;

  feet_array_t<Eigen::Vector2d> nominalFootInBase_;  // hip-projected foot positions in the yaw frame
  std::unique_ptr<LegPostureIk> postureIk_;          // 6D foot IK for the joint posture reference
  double nominalBaseHeight_ = 0.0;                   // pelvis height of the nominal posture on flat ground

  // command (relative height)
  mutable std::mutex commandMutex_;
  ocs2::TargetTrajectories command_;

  // plan for the current solve
  ocs2::scalar_array_t eventTimes_;
  feet_array_t<std::vector<Foothold>> footholds_;  // per phase (entries of swing phases are unused)
  feet_array_t<std::vector<bool>> contactFlags_;
  feet_array_t<ocs2::scalar_t> currentStanceFinalTime_;
  feet_array_t<vector3_t> lastStancePosition_;
  feet_array_t<bool> hasLastStancePosition_{};

  mutable std::mutex planMutex_;
  Plan plan_;
};

}  // namespace g1_perceptive

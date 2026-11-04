#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "anymal_perceptive_mujoco/common/Types.h"
#include "anymal_perceptive_mujoco/foot_planner/SplineCpg.h"

namespace anymal_perceptive {

/**
 * Plans the vertical (z) foot trajectory of every swing phase as a two-piece cubic spline:
 * lift-off height -> apex -> touch-down height.
 *
 * The perceptive reference manager feeds it per-phase lift-off / touch-down heights taken from the
 * terrain and the height of the highest obstacle on the straight line between lift-off and
 * touch-down, so the apex clears steps and boxes.
 */
class SwingTrajectoryPlanner {
 public:
  struct Config {
    ocs2::scalar_t liftOffVelocity = 0.0;
    ocs2::scalar_t touchDownVelocity = 0.0;
    ocs2::scalar_t swingHeight = 0.1;     // apex above max(lift-off, touch-down, obstacle)
    ocs2::scalar_t swingTimeScale = 0.15;  // swing phases shorter than this are scaled down in height and velocity
  };

  SwingTrajectoryPlanner(Config config, size_t numFeet);

  /** Flat terrain at the given height. */
  void update(const ocs2::ModeSchedule& modeSchedule, ocs2::scalar_t terrainHeight);

  /**
   * Terrain-aware update. All sequences are indexed by the phase index of modeSchedule.modeSequence.
   * @param obstacleHeightSequence: per swing phase, height of the highest obstacle to clear (NaN: none).
   */
  void update(const ocs2::ModeSchedule& modeSchedule, const feet_array_t<ocs2::scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<ocs2::scalar_array_t>& touchDownHeightSequence,
              const feet_array_t<ocs2::scalar_array_t>& obstacleHeightSequence);

  ocs2::scalar_t getZvelocityConstraint(size_t leg, ocs2::scalar_t time) const;

  ocs2::scalar_t getZpositionConstraint(size_t leg, ocs2::scalar_t time) const;

  const Config& config() const { return config_; }

  /** Extracts for each leg the contact sequence over the motion phase sequence. */
  feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

 private:
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  static std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(const std::vector<bool>& contactFlagStock);

  static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                       const std::vector<size_t>& phaseIDsStock);

  static ocs2::scalar_t swingTrajectoryScaling(ocs2::scalar_t startTime, ocs2::scalar_t finalTime,
                                               ocs2::scalar_t swingTimeScale);

  const Config config_;
  const size_t numFeet_;

  feet_array_t<std::vector<SplineCpg>> feetHeightTrajectories_;
  feet_array_t<std::vector<ocs2::scalar_t>> feetHeightTrajectoriesEvents_;
};

SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName,
                                                           const std::string& fieldName = "swing_trajectory_config",
                                                           bool verbose = true);

}  // namespace anymal_perceptive

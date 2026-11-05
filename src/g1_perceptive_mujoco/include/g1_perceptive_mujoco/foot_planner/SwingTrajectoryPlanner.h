#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "g1_perceptive_mujoco/common/Types.h"
#include "g1_perceptive_mujoco/foot_planner/SplineCpg.h"

namespace g1_perceptive {

/**
 * Plans the vertical (z) foot trajectory of every swing phase as a two-piece cubic spline:
 * lift-off height -> apex -> touch-down height.
 *
 * On flat ground every phase starts and ends at the same height (the contact frame height above the
 * ground). The per-phase overload also accepts lift-off / touch-down heights and an obstacle height.
 */
class SwingTrajectoryPlanner {
 public:
  struct Config {
    ocs2::scalar_t liftOffVelocity = 0.0;
    ocs2::scalar_t touchDownVelocity = 0.0;
    ocs2::scalar_t swingHeight = 0.1;     // apex above max(lift-off, touch-down, obstacle)
    ocs2::scalar_t swingTimeScale = 0.15;  // swing phases shorter than this are scaled down in height and velocity
    // time of the apex as a fraction of the swing when the touch-down is >= 5 cm lower / higher than the lift-off:
    // late when stepping down (the sole stays above the nose it leaves), early when stepping up
    ocs2::scalar_t stepDownApexPhase = 0.5;
    ocs2::scalar_t stepUpApexPhase = 0.5;
  };

  SwingTrajectoryPlanner(Config config, size_t numFeet);

  /** Flat ground: contact frames at the given height. */
  void update(const ocs2::ModeSchedule& modeSchedule, ocs2::scalar_t terrainHeight);

  /**
   * Per-phase heights. All sequences are indexed by the phase index of modeSchedule.modeSequence.
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

}  // namespace g1_perceptive

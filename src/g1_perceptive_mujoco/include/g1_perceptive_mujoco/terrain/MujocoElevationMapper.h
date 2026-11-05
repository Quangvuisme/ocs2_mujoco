#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "g1_perceptive_mujoco/terrain/TerrainMap.h"

namespace g1_perceptive {

struct ElevationMapperConfig {
  double resolution = 0.02;      // [m]
  double lengthX = 3.2;          // [m] map extent along world x, centred on the robot
  double lengthY = 3.2;          // [m]
  double rayStartHeight = 1.0;   // [m] rays start this far above the base and point straight down
  double updateRate = 20.0;      // [Hz]
  int numThreads = 2;            // threads used for ray casting
  std::array<bool, mjNGROUP> geomGroups{true, true, false, false, false, false};  // terrain geom groups
  TerrainProcessingConfig processing;
};

ElevationMapperConfig loadElevationMapperConfig(const std::string& file, const std::string& prefix, bool verbose);

/**
 * Builds a robot-centric elevation map directly from the MuJoCo scene: one vertical mj_ray per
 * grid cell, against the geoms of the configured geom groups (the G1 model keeps its visual
 * and collision geoms in groups 2 and 3, so terrain in groups 0/1 is all the rays can hit).
 *
 * Threading: captureState() copies qpos/mocap from the live mjData and must be called while the
 * simulation is locked; it is cheap. update() works on a private mjData and can run concurrently
 * with the simulation.
 */
class MujocoElevationMapper {
 public:
  MujocoElevationMapper(const mjModel* model, ElevationMapperConfig config);
  ~MujocoElevationMapper();

  MujocoElevationMapper(const MujocoElevationMapper&) = delete;
  MujocoElevationMapper& operator=(const MujocoElevationMapper&) = delete;

  void captureState(const mjData* data);

  /** Ray casts around the base position of the last captured state and post-processes the map. */
  std::shared_ptr<TerrainMap> update();

  const ElevationMapperConfig& config() const { return config_; }

  /** Wall-clock duration of the last update() [s]. */
  double lastUpdateDuration() const { return lastUpdateDuration_; }

 private:
  void castRows(TerrainMap& map, int rowBegin, int rowEnd, double rayStartZ) const;

  const mjModel* model_;
  mjData* data_;
  ElevationMapperConfig config_;
  std::array<mjtByte, mjNGROUP> geomGroupMask_{};
  int baseQposAdr_ = 0;
  double captureTime_ = 0.0;
  bool captured_ = false;
  double lastUpdateDuration_ = 0.0;
};

}  // namespace g1_perceptive

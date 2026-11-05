#pragma once

#include <mujoco/mujoco.h>

#include "g1_perceptive_mujoco/reference_manager/PerceptiveReferenceManager.h"

namespace g1_perceptive {

/**
 * Draws the perception state as MuJoCo decoration geoms:
 *  - elevation map cells (green: steppable, red: not), subsampled by `mapStride`,
 *  - planned footholds (one colour per leg) and their foot placement regions,
 *  - the terrain-lifted base reference trajectory.
 */
class TerrainVisualizer {
 public:
  struct Options {
    bool showMap = true;
    bool showFootholds = true;
    bool showBaseTrajectory = true;
    int mapStride = 2;
    float mapAlpha = 0.45f;
  };

  TerrainVisualizer() = default;
  explicit TerrainVisualizer(Options options) : options_(options) {}

  Options& options() { return options_; }

  /** Replaces the content of `scene` (scene->ngeom is reset). */
  void draw(const PerceptiveReferenceManager::Plan& plan, mjvScene* scene) const;

  /** Appends to `scene` without clearing it. */
  void append(const PerceptiveReferenceManager::Plan& plan, mjvScene* scene) const;

 private:
  Options options_;
};

}  // namespace g1_perceptive

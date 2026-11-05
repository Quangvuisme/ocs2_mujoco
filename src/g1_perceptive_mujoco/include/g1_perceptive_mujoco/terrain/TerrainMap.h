#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace g1_perceptive {

/** Parameters of the post-processing that turns raw ray-cast heights into planning layers. */
struct TerrainProcessingConfig {
  double smoothingRadius = 0.06;   // [m] box filter for the base reference and the normals
  double gapFillRadius = 0.0;      // [m] closing (max then min filter) before smoothing: gaps narrower than
                                   //     twice this vanish from the smooth layer, step edges stay in place
  double footRadius = 0.04;        // [m] window in which the surface must be flat for a cell to be steppable
  double maxRoughness = 0.03;      // [m] max deviation from the local plane inside the foot window
  double maxSlope = 0.6;           // [rad] steepest steppable slope
  double swingInflation = 0.04;    // [m] max-filter radius of the swing clearance layer
  double maxEdgeDistance = 0.30;   // [m] edge distance saturates at this value
};

/**
 * Robot-centric 2.5D elevation map on a regular grid.
 *
 * Cell (ix, iy) is centred at origin + resolution * (ix, iy). All layers are stored row-major
 * (index = iy * sizeX + ix). Queries outside the map are clamped to the border cells.
 *
 * Layers
 *  - rawHeight     : ray-cast height, NaN where the ray hit nothing
 *  - height        : rawHeight with the holes filled from their neighbours
 *  - smooth        : height with narrow gaps closed, box filtered (base reference, posture, normals)
 *  - inflated      : max-filtered + blurred height, used to keep swing feet above the terrain
 *  - steppable     : 1 where the surface under a foot is flat and not too steep
 *  - edgeDistance  : distance [m] to the closest non-steppable cell
 */
class TerrainMap {
 public:
  TerrainMap() = default;
  TerrainMap(double resolution, int sizeX, int sizeY, const Eigen::Vector2d& origin);

  /** A flat map (all layers at the given height, everything steppable). */
  static std::shared_ptr<TerrainMap> flat(double height, double resolution = 0.05, int size = 81);

  double resolution() const { return resolution_; }
  int sizeX() const { return sizeX_; }
  int sizeY() const { return sizeY_; }
  const Eigen::Vector2d& origin() const { return origin_; }
  size_t numCells() const { return static_cast<size_t>(sizeX_) * sizeY_; }
  size_t linearIndex(int ix, int iy) const { return static_cast<size_t>(iy) * sizeX_ + ix; }
  Eigen::Vector2d cellCenter(int ix, int iy) const { return origin_ + resolution_ * Eigen::Vector2d(ix, iy); }
  Eigen::Vector2i nearestIndex(const Eigen::Vector2d& position) const;
  bool isInside(const Eigen::Vector2d& position) const;

  std::vector<float>& rawHeight() { return rawHeight_; }
  const std::vector<float>& rawHeight() const { return rawHeight_; }
  const std::vector<float>& height() const { return height_; }
  const std::vector<float>& smooth() const { return smooth_; }
  const std::vector<float>& inflated() const { return inflated_; }
  const std::vector<uint8_t>& steppable() const { return steppable_; }
  const std::vector<float>& edgeDistance() const { return edgeDistance_; }

  /** Computes every derived layer from rawHeight(). */
  void process(const TerrainProcessingConfig& config);

  /** Bilinear lookups (gradient is d/dx, d/dy in world frame). */
  double heightAt(const Eigen::Vector2d& p) const { return bilinear(height_, p, nullptr); }
  double smoothHeightAt(const Eigen::Vector2d& p, Eigen::Vector2d* gradient = nullptr) const {
    return bilinear(smooth_, p, gradient);
  }
  double inflatedHeightAt(const Eigen::Vector2d& p, Eigen::Vector2d* gradient = nullptr) const {
    return bilinear(inflated_, p, gradient);
  }

  /** Unit surface normal from the smooth layer. */
  Eigen::Vector3d normalAt(const Eigen::Vector2d& p) const;

  /** Nearest-cell lookups. */
  bool isSteppable(const Eigen::Vector2d& p) const;
  double edgeDistanceAt(const Eigen::Vector2d& p) const;

  /** Highest value of the inflated layer on the straight segment a -> b. */
  double maxInflatedHeightOnSegment(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;

  /** Simulation time at which the map was captured. */
  double time = 0.0;

 private:
  double bilinear(const std::vector<float>& layer, const Eigen::Vector2d& p, Eigen::Vector2d* gradient) const;

  double resolution_ = 0.05;
  int sizeX_ = 0;
  int sizeY_ = 0;
  Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();

  std::vector<float> rawHeight_;
  std::vector<float> height_;
  std::vector<float> smooth_;
  std::vector<float> inflated_;
  std::vector<uint8_t> steppable_;
  std::vector<float> edgeDistance_;
};

/** Thread-safe hand-over of the latest map from the mapping thread to the MPC thread. */
class TerrainMapBuffer {
 public:
  void set(std::shared_ptr<const TerrainMap> map) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_ = std::move(map);
  }
  std::shared_ptr<const TerrainMap> get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_;
  }

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<const TerrainMap> map_;
};

/** Reads the "terrain_processing" block of an info file. */
TerrainProcessingConfig loadTerrainProcessingConfig(const std::string& file, const std::string& prefix, bool verbose);

}  // namespace g1_perceptive

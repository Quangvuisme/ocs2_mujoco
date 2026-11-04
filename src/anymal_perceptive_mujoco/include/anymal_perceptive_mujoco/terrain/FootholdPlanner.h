#pragma once

#include <map>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "anymal_perceptive_mujoco/terrain/TerrainMap.h"

namespace anymal_perceptive {

struct FootholdPlannerConfig {
  double searchRadius = 0.15;         // [m] footholds are searched within this radius of the nominal one
  double minEdgeDistance = 0.04;      // [m] candidates closer than this to an edge are rejected
  double desiredEdgeDistance = 0.10;  // [m] candidates closer than this are penalised
  double edgeWeight = 4.0;            // weight of the edge penalty (relative to squared distance to nominal)
  double previousWeight = 0.5;        // weight on staying close to the foothold chosen at the last MPC iteration
  double regionInset = 0.02;          // [m] foot placement region radius = edge distance - inset
  double regionMinRadius = 0.02;      // [m]
  double regionMaxRadius = 0.06;      // [m]
  int numVertices = 8;                // polygon approximating the region
  double feedbackGain = 1.0;          // Raibert feedback gain on sqrt(h/g) * (v_measured - v_desired)
  double maxFeedbackX = 0.08;         // [m]
  double maxFeedbackY = 0.06;         // [m]
};

FootholdPlannerConfig loadFootholdPlannerConfig(const std::string& file, const std::string& prefix, bool verbose);

/** A foothold and the convex region (in its tangent plane) the stance foot has to stay in. */
struct Foothold {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  // on the terrain surface (world)
  Eigen::Vector3d nominal = Eigen::Vector3d::Zero();   // where the foot would go on flat ground
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double regionRadius = 0.0;
  bool constrained = false;  // true for future stance phases with a valid region
  Eigen::Matrix<double, Eigen::Dynamic, 3> A;  // region: A * p_foot + b >= 0
  Eigen::VectorXd b;
};

/**
 * Picks footholds on the elevation map: among the steppable cells within searchRadius of the
 * nominal (Raibert) foothold, take the one minimising
 *     |p - p_nominal|^2 + w_edge * max(0, d_desired - d_edge)^2 + w_prev * |p - p_previous|^2
 * and build a regular polygon region around it whose size follows the distance to the nearest edge.
 */
class FootholdPlanner {
 public:
  explicit FootholdPlanner(FootholdPlannerConfig config) : config_(std::move(config)) {}

  const FootholdPlannerConfig& config() const { return config_; }

  /** Call once per MPC iteration before select(); keeps last iteration's choices for hysteresis. */
  void beginIteration();

  /** Selects the foothold of the stance phase of `leg` that starts at `stanceStartTime`. */
  Foothold select(size_t leg, double stanceStartTime, const Eigen::Vector3d& nominal, const TerrainMap& map);

  /** Foothold at a fixed position (current stance), no region constraint. */
  static Foothold fixed(const Eigen::Vector3d& position, const TerrainMap& map);

 private:
  using Key = std::pair<size_t, long>;
  static Key key(size_t leg, double time) { return {leg, std::lround(time * 1000.0)}; }

  void buildRegion(Foothold& foothold, double edgeDistance) const;

  FootholdPlannerConfig config_;
  std::map<Key, Eigen::Vector2d> previous_;
  std::map<Key, Eigen::Vector2d> current_;
};

}  // namespace anymal_perceptive

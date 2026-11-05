#pragma once

#include <map>
#include <string>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "g1_perceptive_mujoco/terrain/TerrainMap.h"

namespace g1_perceptive {

struct FootholdPlannerConfig {
  double searchRadius = 0.15;         // [m] footholds are searched within this radius of the nominal one
  double minEdgeDistance = 0.04;      // [m] candidates closer than this to an edge are rejected
  double desiredEdgeDistance = 0.10;  // [m] candidates closer than this are penalised
  double edgeWeight = 4.0;            // weight of the edge penalty (relative to squared distance to nominal)
  double previousWeight = 0.5;        // weight on staying close to the foothold chosen at the last MPC iteration
  double forwardWeight = 0.0;         // extra weight on moving beyond the nominal foothold in the walking direction
  double maxStepHeight = 1e9;         // [m] candidates higher / lower than the previous stance of the foot are rejected
  double expandedSearchFactor = 1.0;  // selectFoot: search radius factor of the second pass (nothing found in the first)
  double minLateralOffset = -1e9;     // [m] selectFoot: sole centre at least this far on its own side of the pelvis line
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

  /**
   * Foothold of a foot that touches the ground at several points (e.g. toe and heel of a humanoid sole).
   * The foot centre is searched around `nominalCenter`; a candidate is valid only if every support point
   * (centre + offset, offsets in world x/y) lies on a steppable cell at least minEdgeDistance from an edge.
   * The edge penalty uses the smallest edge distance of the support points. Steps longer than nominal along
 * `heading` (unit vector, or zero) cost forwardWeight * overshoot^2 extra: on stairs a shorter step onto the
 * current tread is preferred over reaching for the next one. Candidates more than maxStepHeight above or below
 * `previousHeight` (the last stance of this foot, NaN: no limit) are rejected, e.g. the floor of a pit. With
 * side = +1 (left foot) / -1 (right foot), the sole centre must lie at least minLateralOffset to that side of the
 * line through `pelvis` along `heading`: no crossed legs when the free cells are on the other side (stepping stones).
   * @return one foothold per support point, each with its own region.
   */
  std::vector<Foothold> selectFoot(size_t foot, double stanceStartTime, const Eigen::Vector2d& nominalCenter,
                                   const std::vector<Eigen::Vector2d>& supportOffsets, const TerrainMap& map,
                                   const Eigen::Vector2d& heading = Eigen::Vector2d::Zero(),
                                   double previousHeight = std::numeric_limits<double>::quiet_NaN(),
                                   const Eigen::Vector2d& pelvis = Eigen::Vector2d::Zero(), double side = 0.0);

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

}  // namespace g1_perceptive

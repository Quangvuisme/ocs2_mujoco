#include "anymal_perceptive_mujoco/terrain/FootholdPlanner.h"

#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace anymal_perceptive {

void FootholdPlanner::beginIteration() {
  previous_.swap(current_);
  current_.clear();
}

Foothold FootholdPlanner::fixed(const Eigen::Vector3d& position, const TerrainMap& map) {
  Foothold f;
  f.position = position;
  f.nominal = position;
  f.normal = map.normalAt(position.head<2>());
  f.constrained = false;
  return f;
}

Foothold FootholdPlanner::select(size_t leg, double stanceStartTime, const Eigen::Vector3d& nominal, const TerrainMap& map) {
  const Key k = key(leg, stanceStartTime);
  const auto prevIt = previous_.find(k);
  const bool hasPrevious = prevIt != previous_.end();

  const double res = map.resolution();
  const int radiusCells = std::max(1, static_cast<int>(std::ceil(config_.searchRadius / res)));
  const Eigen::Vector2i centre = map.nearestIndex(nominal.head<2>());
  const double searchRadius2 = config_.searchRadius * config_.searchRadius;

  double bestCost = std::numeric_limits<double>::infinity();
  Eigen::Vector2d best = nominal.head<2>();
  double bestEdge = 0.0;
  for (int dy = -radiusCells; dy <= radiusCells; ++dy) {
    for (int dx = -radiusCells; dx <= radiusCells; ++dx) {
      const int ix = centre.x() + dx;
      const int iy = centre.y() + dy;
      if (ix < 0 || iy < 0 || ix >= map.sizeX() || iy >= map.sizeY()) {
        continue;
      }
      const size_t i = map.linearIndex(ix, iy);
      if (!map.steppable()[i]) {
        continue;
      }
      const double edge = map.edgeDistance()[i];
      if (edge < config_.minEdgeDistance) {
        continue;
      }
      const Eigen::Vector2d p = map.cellCenter(ix, iy);
      const double d2 = (p - nominal.head<2>()).squaredNorm();
      if (d2 > searchRadius2) {
        continue;
      }
      const double edgeDeficit = std::max(0.0, config_.desiredEdgeDistance - edge);
      double cost = d2 + config_.edgeWeight * edgeDeficit * edgeDeficit;
      if (hasPrevious) {
        cost += config_.previousWeight * (p - prevIt->second).squaredNorm();
      }
      if (cost < bestCost) {
        bestCost = cost;
        best = p;
        bestEdge = edge;
      }
    }
  }

  Foothold f;
  f.nominal = nominal;
  if (std::isfinite(bestCost)) {
    f.position << best, map.heightAt(best);
    f.normal = map.normalAt(best);
    buildRegion(f, bestEdge);
    current_[k] = best;
  } else {
    // nothing steppable nearby: aim at the nominal foothold and let the MPC decide
    f.position << nominal.head<2>(), map.heightAt(nominal.head<2>());
    f.normal = map.normalAt(nominal.head<2>());
    f.constrained = false;
  }
  return f;
}

void FootholdPlanner::buildRegion(Foothold& f, double edgeDistance) const {
  const int n = std::max(3, config_.numVertices);
  f.regionRadius = std::clamp(edgeDistance - config_.regionInset, config_.regionMinRadius, config_.regionMaxRadius);

  // tangent frame of the surface at the foothold
  Eigen::Vector3d normal = f.normal;
  if (normal.z() < 0.3) {
    normal = Eigen::Vector3d::UnitZ();
  }
  Eigen::Vector3d tx = Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX().dot(normal) * normal;
  tx.normalize();
  const Eigen::Vector3d ty = normal.cross(tx);

  // regular polygon inscribed in the circle of radius r: half-planes m_k . q <= r cos(pi/n)
  const double apothem = f.regionRadius * std::cos(M_PI / n);
  f.A.resize(n, 3);
  f.b.resize(n);
  for (int k = 0; k < n; ++k) {
    const double theta = 2.0 * M_PI * k / n;
    const Eigen::Vector3d direction = std::cos(theta) * tx + std::sin(theta) * ty;  // outward normal in world
    // apothem - direction . (p - c) >= 0
    f.A.row(k) = -direction.transpose();
    f.b(k) = apothem + direction.dot(f.position);
  }
  f.constrained = true;
}

FootholdPlannerConfig loadFootholdPlannerConfig(const std::string& file, const std::string& prefix, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  FootholdPlannerConfig c;
  const std::string p = prefix + ".";
  ocs2::loadData::loadPtreeValue(pt, c.searchRadius, p + "searchRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.minEdgeDistance, p + "minEdgeDistance", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.desiredEdgeDistance, p + "desiredEdgeDistance", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.edgeWeight, p + "edgeWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.previousWeight, p + "previousWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.regionInset, p + "regionInset", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.regionMinRadius, p + "regionMinRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.regionMaxRadius, p + "regionMaxRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.numVertices, p + "numVertices", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.feedbackGain, p + "feedbackGain", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.maxFeedbackX, p + "maxFeedbackX", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.maxFeedbackY, p + "maxFeedbackY", verbose);
  return c;
}

}  // namespace anymal_perceptive

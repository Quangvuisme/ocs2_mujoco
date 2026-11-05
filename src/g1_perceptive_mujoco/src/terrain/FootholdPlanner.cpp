#include "g1_perceptive_mujoco/terrain/FootholdPlanner.h"

#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace g1_perceptive {

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

std::vector<Foothold> FootholdPlanner::selectFoot(size_t foot, double stanceStartTime, const Eigen::Vector2d& nominalCenter,
                                                  const std::vector<Eigen::Vector2d>& supportOffsets, const TerrainMap& map,
                                                  const Eigen::Vector2d& heading, double previousHeight,
                                                  const Eigen::Vector2d& pelvis, double side) {
  const Eigen::Vector2d left(-heading.y(), heading.x());  // lateral direction (left) of the heading
  const Key k = key(foot, stanceStartTime);
  const auto prevIt = previous_.find(k);
  const bool hasPrevious = prevIt != previous_.end();

  const double res = map.resolution();
  const Eigen::Vector2i centre = map.nearestIndex(nominalCenter);

  // edge distance of a support point (0 if it is not steppable)
  auto supportEdge = [&](const Eigen::Vector2d& p) {
    if (!map.isInside(p) || !map.isSteppable(p)) {
      return 0.0;
    }
    return map.edgeDistanceAt(p);
  };

  // passes, until a candidate is found:
  //   0: search radius, every support point >= minEdgeDistance from an edge
  //   1: search radius x expandedSearchFactor (e.g. the nominal sole is over a gap wider than the search radius)
  //   2: expanded radius, only steppable (no region constraint)
  double bestCost = std::numeric_limits<double>::infinity();
  Eigen::Vector2d best = nominalCenter;
  bool regions = true;
  for (int pass = 0; pass < 3 && !std::isfinite(bestCost); ++pass) {
    const double minEdge = pass < 2 ? config_.minEdgeDistance : 1e-9;
    regions = pass < 2;
    const double radius = pass == 0 ? config_.searchRadius : config_.searchRadius * config_.expandedSearchFactor;
    const int radiusCells = std::max(1, static_cast<int>(std::ceil(radius / res)));
    const double searchRadius2 = radius * radius;
    for (int dy = -radiusCells; dy <= radiusCells; ++dy) {
      for (int dx = -radiusCells; dx <= radiusCells; ++dx) {
        const int ix = centre.x() + dx;
        const int iy = centre.y() + dy;
        if (ix < 0 || iy < 0 || ix >= map.sizeX() || iy >= map.sizeY()) {
          continue;
        }
        const Eigen::Vector2d c = map.cellCenter(ix, iy);
        const double d2 = (c - nominalCenter).squaredNorm();
        if (d2 > searchRadius2) {
          continue;
        }
        if (side != 0.0 && side * (c - pelvis).dot(left) < config_.minLateralOffset) {
          continue;  // would cross the other leg
        }
        double pointEdge = std::numeric_limits<double>::infinity();
        for (const auto& offset : supportOffsets) {
          pointEdge = std::min(pointEdge, supportEdge(c + offset));
        }
        if (pointEdge < minEdge) {
          continue;
        }
        // all support points on one level: no sole straddling a step nose
        const double h0 = map.heightAt(c + supportOffsets.front());
        bool level = true;
        for (const auto& offset : supportOffsets) {
          level = level && std::abs(map.heightAt(c + offset) - h0) < 0.02;
        }
        if (!level) {
          continue;
        }
        if (std::isfinite(previousHeight) && std::abs(h0 - previousHeight) > config_.maxStepHeight) {
          continue;
        }
        const double edgeDeficit = std::max(0.0, config_.desiredEdgeDistance - pointEdge);
        double cost = d2 + config_.edgeWeight * edgeDeficit * edgeDeficit;
        const double overshoot = std::max(0.0, (c - nominalCenter).dot(heading));
        cost += config_.forwardWeight * overshoot * overshoot;
        if (hasPrevious) {
          cost += config_.previousWeight * (c - prevIt->second).squaredNorm();
        }
        if (cost < bestCost) {
          bestCost = cost;
          best = c;
        }
      }
    }
  }

  // no level candidate at all: aim at the nominal sole, every support point at the (gap-closed) height under the sole
  const double soleHeight = map.smoothHeightAt(best);
  std::vector<Foothold> footholds(supportOffsets.size());
  for (size_t i = 0; i < supportOffsets.size(); ++i) {
    Foothold& f = footholds[i];
    const Eigen::Vector2d p = best + supportOffsets[i];
    f.nominal << nominalCenter + supportOffsets[i], map.heightAt(nominalCenter + supportOffsets[i]);
    f.position << p, std::isfinite(bestCost) ? map.heightAt(p) : soleHeight;
    f.normal = std::isfinite(bestCost) ? map.normalAt(p) : Eigen::Vector3d::UnitZ();
    if (std::isfinite(bestCost) && regions) {
      buildRegion(f, supportEdge(p));
    } else {
      f.constrained = false;  // the MPC places the sole near the target without a region
    }
  }
  if (std::isfinite(bestCost)) {
    current_[k] = best;
  }
  return footholds;
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
  ocs2::loadData::loadPtreeValue(pt, c.forwardWeight, p + "forwardWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.maxStepHeight, p + "maxStepHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.expandedSearchFactor, p + "expandedSearchFactor", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.minLateralOffset, p + "minLateralOffset", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.regionInset, p + "regionInset", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.regionMinRadius, p + "regionMinRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.regionMaxRadius, p + "regionMaxRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.numVertices, p + "numVertices", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.feedbackGain, p + "feedbackGain", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.maxFeedbackX, p + "maxFeedbackX", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.maxFeedbackY, p + "maxFeedbackY", verbose);
  return c;
}

}  // namespace g1_perceptive

#include "g1_perceptive_mujoco/terrain/TerrainMap.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace g1_perceptive {

namespace {

/** Separable box filter with replicated borders. */
void boxFilter(const std::vector<float>& in, std::vector<float>& out, int sizeX, int sizeY, int radius) {
  out = in;
  if (radius <= 0) {
    return;
  }
  std::vector<float> tmp(in.size());
  const float norm = 1.0f / static_cast<float>(2 * radius + 1);
  for (int iy = 0; iy < sizeY; ++iy) {
    for (int ix = 0; ix < sizeX; ++ix) {
      float sum = 0.0f;
      for (int k = -radius; k <= radius; ++k) {
        const int jx = std::clamp(ix + k, 0, sizeX - 1);
        sum += in[static_cast<size_t>(iy) * sizeX + jx];
      }
      tmp[static_cast<size_t>(iy) * sizeX + ix] = sum * norm;
    }
  }
  for (int iy = 0; iy < sizeY; ++iy) {
    for (int ix = 0; ix < sizeX; ++ix) {
      float sum = 0.0f;
      for (int k = -radius; k <= radius; ++k) {
        const int jy = std::clamp(iy + k, 0, sizeY - 1);
        sum += tmp[static_cast<size_t>(jy) * sizeX + ix];
      }
      out[static_cast<size_t>(iy) * sizeX + ix] = sum * norm;
    }
  }
}

/** Separable max filter (square window). */
void maxFilter(const std::vector<float>& in, std::vector<float>& out, int sizeX, int sizeY, int radius) {
  out = in;
  if (radius <= 0) {
    return;
  }
  std::vector<float> tmp(in.size());
  for (int iy = 0; iy < sizeY; ++iy) {
    for (int ix = 0; ix < sizeX; ++ix) {
      float m = -std::numeric_limits<float>::infinity();
      for (int k = -radius; k <= radius; ++k) {
        const int jx = std::clamp(ix + k, 0, sizeX - 1);
        m = std::max(m, in[static_cast<size_t>(iy) * sizeX + jx]);
      }
      tmp[static_cast<size_t>(iy) * sizeX + ix] = m;
    }
  }
  for (int iy = 0; iy < sizeY; ++iy) {
    for (int ix = 0; ix < sizeX; ++ix) {
      float m = -std::numeric_limits<float>::infinity();
      for (int k = -radius; k <= radius; ++k) {
        const int jy = std::clamp(iy + k, 0, sizeY - 1);
        m = std::max(m, tmp[static_cast<size_t>(jy) * sizeX + ix]);
      }
      out[static_cast<size_t>(iy) * sizeX + ix] = m;
    }
  }
}

/** Separable min filter (square window). */
void minFilter(const std::vector<float>& in, std::vector<float>& out, int sizeX, int sizeY, int radius) {
  std::vector<float> negated(in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    negated[i] = -in[i];
  }
  maxFilter(negated, out, sizeX, sizeY, radius);
  for (auto& v : out) {
    v = -v;
  }
}

}  // namespace

TerrainMap::TerrainMap(double resolution, int sizeX, int sizeY, const Eigen::Vector2d& origin)
    : resolution_(resolution), sizeX_(sizeX), sizeY_(sizeY), origin_(origin) {
  if (resolution <= 0.0 || sizeX < 2 || sizeY < 2) {
    throw std::invalid_argument("[TerrainMap] resolution must be > 0 and the map at least 2x2 cells");
  }
  rawHeight_.assign(numCells(), std::numeric_limits<float>::quiet_NaN());
}

std::shared_ptr<TerrainMap> TerrainMap::flat(double height, double resolution, int size) {
  const Eigen::Vector2d origin = -0.5 * resolution * (size - 1) * Eigen::Vector2d::Ones();
  auto map = std::make_shared<TerrainMap>(resolution, size, size, origin);
  std::fill(map->rawHeight_.begin(), map->rawHeight_.end(), static_cast<float>(height));
  map->process(TerrainProcessingConfig());
  return map;
}

Eigen::Vector2i TerrainMap::nearestIndex(const Eigen::Vector2d& position) const {
  if (!position.allFinite()) {  // diverged query (e.g. a NaN state): answer for the map centre
    return {sizeX_ / 2, sizeY_ / 2};
  }
  const Eigen::Vector2d u = (position - origin_) / resolution_;
  return {std::clamp(static_cast<int>(std::lround(u.x())), 0, sizeX_ - 1),
          std::clamp(static_cast<int>(std::lround(u.y())), 0, sizeY_ - 1)};
}

bool TerrainMap::isInside(const Eigen::Vector2d& position) const {
  const Eigen::Vector2d u = (position - origin_) / resolution_;
  return u.x() >= 0.0 && u.y() >= 0.0 && u.x() <= sizeX_ - 1 && u.y() <= sizeY_ - 1;
}

void TerrainMap::process(const TerrainProcessingConfig& config) {
  const size_t n = numCells();

  // 1) fill holes (cells without a ray return) from their valid neighbours
  height_ = rawHeight_;
  std::vector<uint8_t> valid(n);
  size_t numValid = 0;
  for (size_t i = 0; i < n; ++i) {
    valid[i] = std::isfinite(height_[i]) ? 1 : 0;
    numValid += valid[i];
  }
  if (numValid == 0) {
    std::fill(height_.begin(), height_.end(), 0.0f);
  } else if (numValid < n) {
    const int maxPasses = sizeX_ + sizeY_;
    for (int pass = 0; pass < maxPasses; ++pass) {
      std::vector<uint8_t> nextValid = valid;
      bool changed = false;
      for (int iy = 0; iy < sizeY_; ++iy) {
        for (int ix = 0; ix < sizeX_; ++ix) {
          const size_t i = linearIndex(ix, iy);
          if (valid[i]) {
            continue;
          }
          float sum = 0.0f;
          int count = 0;
          const int nx[4] = {ix - 1, ix + 1, ix, ix};
          const int ny[4] = {iy, iy, iy - 1, iy + 1};
          for (int k = 0; k < 4; ++k) {
            if (nx[k] < 0 || ny[k] < 0 || nx[k] >= sizeX_ || ny[k] >= sizeY_) {
              continue;
            }
            const size_t j = linearIndex(nx[k], ny[k]);
            if (valid[j]) {
              sum += height_[j];
              ++count;
            }
          }
          if (count > 0) {
            height_[i] = sum / count;
            nextValid[i] = 1;
            changed = true;
          }
        }
      }
      valid.swap(nextValid);
      if (!changed) {
        break;
      }
    }
  }

  // 2) smooth layer
  //    narrow gaps (pits) are closed first, so the base reference and the posture do not dive into them
  const int smoothRadius = static_cast<int>(std::lround(config.smoothingRadius / resolution_));
  const int gapRadius = static_cast<int>(std::lround(config.gapFillRadius / resolution_));
  if (gapRadius > 0) {
    std::vector<float> dilated, closed;
    maxFilter(height_, dilated, sizeX_, sizeY_, gapRadius);
    minFilter(dilated, closed, sizeX_, sizeY_, gapRadius);
    boxFilter(closed, smooth_, sizeX_, sizeY_, smoothRadius);
  } else {
    boxFilter(height_, smooth_, sizeX_, sizeY_, smoothRadius);
  }

  // 3) steppability: local plane fit inside the foot window
  const int footRadius = std::max(1, static_cast<int>(std::lround(config.footRadius / resolution_)));
  std::vector<Eigen::Vector2i> window;
  for (int dy = -footRadius; dy <= footRadius; ++dy) {
    for (int dx = -footRadius; dx <= footRadius; ++dx) {
      if (dx * dx + dy * dy <= footRadius * footRadius) {
        window.emplace_back(dx, dy);
      }
    }
  }
  const double maxGradient = std::tan(config.maxSlope);
  steppable_.assign(n, 0);
  for (int iy = 0; iy < sizeY_; ++iy) {
    for (int ix = 0; ix < sizeX_; ++ix) {
      const size_t i = linearIndex(ix, iy);
      if (!std::isfinite(rawHeight_[i])) {
        continue;
      }
      // least-squares plane h = a + gx * dx + gy * dy over the window cells inside the map
      double sumH = 0.0, sumXH = 0.0, sumYH = 0.0, sumXX = 0.0, sumYY = 0.0, sumX = 0.0, sumY = 0.0;
      int count = 0;
      bool hole = false;
      for (const auto& d : window) {
        const int jx = ix + d.x();
        const int jy = iy + d.y();
        if (jx < 0 || jy < 0 || jx >= sizeX_ || jy >= sizeY_) {
          continue;
        }
        const size_t j = linearIndex(jx, jy);
        if (!std::isfinite(rawHeight_[j])) {
          hole = true;
          break;
        }
        const double h = rawHeight_[j];
        sumH += h;
        sumXH += d.x() * h;
        sumYH += d.y() * h;
        sumXX += d.x() * d.x();
        sumYY += d.y() * d.y();
        sumX += d.x();
        sumY += d.y();
        ++count;
      }
      if (hole || count < 3) {
        continue;
      }
      const double meanH = sumH / count;
      const double meanX = sumX / count;
      const double meanY = sumY / count;
      const double varX = sumXX / count - meanX * meanX;
      const double varY = sumYY / count - meanY * meanY;
      const double gx = varX > 1e-9 ? (sumXH / count - meanX * meanH) / varX : 0.0;  // per cell
      const double gy = varY > 1e-9 ? (sumYH / count - meanY * meanH) / varY : 0.0;
      if (std::hypot(gx, gy) / resolution_ > maxGradient) {
        continue;
      }
      double maxDeviation = 0.0;
      for (const auto& d : window) {
        const int jx = ix + d.x();
        const int jy = iy + d.y();
        if (jx < 0 || jy < 0 || jx >= sizeX_ || jy >= sizeY_) {
          continue;
        }
        const double planeH = meanH + gx * (d.x() - meanX) + gy * (d.y() - meanY);
        maxDeviation = std::max(maxDeviation, std::abs(rawHeight_[linearIndex(jx, jy)] - planeH));
      }
      steppable_[i] = maxDeviation <= config.maxRoughness ? 1 : 0;
    }
  }

  // 4) distance to the closest non-steppable cell (two-pass chamfer transform, in cells)
  constexpr float kInf = 1e9f;
  constexpr float kDiag = 1.41421356f;
  edgeDistance_.assign(n, kInf);
  for (size_t i = 0; i < n; ++i) {
    if (!steppable_[i]) {
      edgeDistance_[i] = 0.0f;
    }
  }
  auto relax = [&](int ix, int iy, int jx, int jy, float cost) {
    if (jx < 0 || jy < 0 || jx >= sizeX_ || jy >= sizeY_) {
      return;
    }
    float& d = edgeDistance_[linearIndex(ix, iy)];
    d = std::min(d, edgeDistance_[linearIndex(jx, jy)] + cost);
  };
  for (int iy = 0; iy < sizeY_; ++iy) {
    for (int ix = 0; ix < sizeX_; ++ix) {
      relax(ix, iy, ix - 1, iy, 1.0f);
      relax(ix, iy, ix, iy - 1, 1.0f);
      relax(ix, iy, ix - 1, iy - 1, kDiag);
      relax(ix, iy, ix + 1, iy - 1, kDiag);
    }
  }
  for (int iy = sizeY_ - 1; iy >= 0; --iy) {
    for (int ix = sizeX_ - 1; ix >= 0; --ix) {
      relax(ix, iy, ix + 1, iy, 1.0f);
      relax(ix, iy, ix, iy + 1, 1.0f);
      relax(ix, iy, ix + 1, iy + 1, kDiag);
      relax(ix, iy, ix - 1, iy + 1, kDiag);
    }
  }
  const float maxDistance = static_cast<float>(config.maxEdgeDistance);
  for (auto& d : edgeDistance_) {
    d = std::min(d * static_cast<float>(resolution_), maxDistance);
  }

  // 5) swing clearance layer: dilate by the foot size, then blur so the gradient is informative
  const int inflation = static_cast<int>(std::lround(config.swingInflation / resolution_));
  std::vector<float> dilated;
  maxFilter(height_, dilated, sizeX_, sizeY_, inflation);
  boxFilter(dilated, inflated_, sizeX_, sizeY_, 1);
}

double TerrainMap::bilinear(const std::vector<float>& layer, const Eigen::Vector2d& p, Eigen::Vector2d* gradient) const {
  if (!p.allFinite()) {  // diverged query (e.g. a NaN state): flat answer at the map centre
    if (gradient != nullptr) {
      gradient->setZero();
    }
    return layer[linearIndex(sizeX_ / 2, sizeY_ / 2)];
  }
  const Eigen::Vector2d u = (p - origin_) / resolution_;
  const double ux = std::clamp(u.x(), 0.0, static_cast<double>(sizeX_ - 1));
  const double uy = std::clamp(u.y(), 0.0, static_cast<double>(sizeY_ - 1));
  const int ix = std::min(static_cast<int>(std::floor(ux)), sizeX_ - 2);
  const int iy = std::min(static_cast<int>(std::floor(uy)), sizeY_ - 2);
  const double fx = ux - ix;
  const double fy = uy - iy;
  const double h00 = layer[linearIndex(ix, iy)];
  const double h10 = layer[linearIndex(ix + 1, iy)];
  const double h01 = layer[linearIndex(ix, iy + 1)];
  const double h11 = layer[linearIndex(ix + 1, iy + 1)];
  if (gradient != nullptr) {
    const bool clampedX = u.x() < 0.0 || u.x() > sizeX_ - 1;
    const bool clampedY = u.y() < 0.0 || u.y() > sizeY_ - 1;
    gradient->x() = clampedX ? 0.0 : ((1.0 - fy) * (h10 - h00) + fy * (h11 - h01)) / resolution_;
    gradient->y() = clampedY ? 0.0 : ((1.0 - fx) * (h01 - h00) + fx * (h11 - h10)) / resolution_;
  }
  return (1.0 - fx) * (1.0 - fy) * h00 + fx * (1.0 - fy) * h10 + (1.0 - fx) * fy * h01 + fx * fy * h11;
}

Eigen::Vector3d TerrainMap::normalAt(const Eigen::Vector2d& p) const {
  Eigen::Vector2d g;
  smoothHeightAt(p, &g);
  return Eigen::Vector3d(-g.x(), -g.y(), 1.0).normalized();
}

bool TerrainMap::isSteppable(const Eigen::Vector2d& p) const {
  const auto idx = nearestIndex(p);
  return steppable_[linearIndex(idx.x(), idx.y())] != 0;
}

double TerrainMap::edgeDistanceAt(const Eigen::Vector2d& p) const {
  const auto idx = nearestIndex(p);
  return edgeDistance_[linearIndex(idx.x(), idx.y())];
}

double TerrainMap::maxInflatedHeightOnSegment(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
  const double length = (b - a).norm();
  const int numSamples = std::max(2, static_cast<int>(std::ceil(length / (0.5 * resolution_))) + 1);
  double maxHeight = -std::numeric_limits<double>::infinity();
  for (int k = 0; k < numSamples; ++k) {
    const Eigen::Vector2d p = a + (b - a) * (static_cast<double>(k) / (numSamples - 1));
    const auto idx = nearestIndex(p);
    maxHeight = std::max(maxHeight, static_cast<double>(inflated_[linearIndex(idx.x(), idx.y())]));
  }
  return maxHeight;
}

TerrainProcessingConfig loadTerrainProcessingConfig(const std::string& file, const std::string& prefix, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  TerrainProcessingConfig c;
  const std::string p = prefix + ".";
  ocs2::loadData::loadPtreeValue(pt, c.smoothingRadius, p + "smoothingRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.gapFillRadius, p + "gapFillRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.footRadius, p + "footRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.maxRoughness, p + "maxRoughness", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.maxSlope, p + "maxSlope", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.swingInflation, p + "swingInflation", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.maxEdgeDistance, p + "maxEdgeDistance", verbose);
  return c;
}

}  // namespace g1_perceptive

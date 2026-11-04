#include "anymal_perceptive_mujoco/terrain/MujocoElevationMapper.h"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <type_traits>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace anymal_perceptive {

namespace {

/**
 * mj_ray gained a surface-normal output argument in newer MuJoCo releases. Dispatch on the
 * signature so the package builds against both (MuJoCo >= 3.0).
 */
template <typename RayFn>
mjtNum castRay(RayFn rayFn, const mjModel* m, const mjData* d, const mjtNum* pnt, const mjtNum* vec,
               const mjtByte* geomGroup) {
  int geomId = -1;
  if constexpr (std::is_invocable_v<RayFn, const mjModel*, const mjData*, const mjtNum*, const mjtNum*, const mjtByte*,
                                    mjtByte, int, int*, mjtNum*>) {
    mjtNum normal[3];
    return rayFn(m, d, pnt, vec, geomGroup, 1, -1, &geomId, normal);
  } else {
    return rayFn(m, d, pnt, vec, geomGroup, 1, -1, &geomId);
  }
}

}  // namespace

MujocoElevationMapper::MujocoElevationMapper(const mjModel* model, ElevationMapperConfig config)
    : model_(model), data_(mj_makeData(model)), config_(std::move(config)) {
  if (data_ == nullptr) {
    throw std::runtime_error("[MujocoElevationMapper] mj_makeData failed");
  }
  for (int g = 0; g < mjNGROUP; ++g) {
    geomGroupMask_[g] = config_.geomGroups[g] ? 1 : 0;
  }
  int freeJoint = -1;
  for (int j = 0; j < model_->njnt; ++j) {
    if (model_->jnt_type[j] == mjJNT_FREE) {
      freeJoint = j;
      break;
    }
  }
  if (freeJoint < 0) {
    throw std::runtime_error("[MujocoElevationMapper] the model has no free joint (floating base)");
  }
  baseQposAdr_ = model_->jnt_qposadr[freeJoint];
}

MujocoElevationMapper::~MujocoElevationMapper() { mj_deleteData(data_); }

void MujocoElevationMapper::captureState(const mjData* data) {
  mju_copy(data_->qpos, data->qpos, model_->nq);
  if (model_->nmocap > 0) {
    mju_copy(data_->mocap_pos, data->mocap_pos, 3 * model_->nmocap);
    mju_copy(data_->mocap_quat, data->mocap_quat, 4 * model_->nmocap);
  }
  captureTime_ = data->time;
  captured_ = true;
}

void MujocoElevationMapper::castRows(TerrainMap& map, int rowBegin, int rowEnd, double rayStartZ) const {
  const mjtNum direction[3] = {0.0, 0.0, -1.0};
  auto& raw = map.rawHeight();
  for (int iy = rowBegin; iy < rowEnd; ++iy) {
    for (int ix = 0; ix < map.sizeX(); ++ix) {
      const Eigen::Vector2d c = map.cellCenter(ix, iy);
      const mjtNum start[3] = {c.x(), c.y(), rayStartZ};
      const mjtNum distance = castRay(&mj_ray, model_, data_, start, direction, geomGroupMask_.data());
      raw[map.linearIndex(ix, iy)] =
          distance >= 0.0 ? static_cast<float>(rayStartZ - distance) : std::numeric_limits<float>::quiet_NaN();
    }
  }
}

std::shared_ptr<TerrainMap> MujocoElevationMapper::update() {
  if (!captured_) {
    return nullptr;
  }
  const auto start = std::chrono::steady_clock::now();

  // poses of every geom (terrain may also live on mocap or moving bodies)
  mj_kinematics(model_, data_);

  const double res = config_.resolution;
  const int sizeX = std::max(2, static_cast<int>(std::lround(config_.lengthX / res)) | 1);  // odd: base cell centred
  const int sizeY = std::max(2, static_cast<int>(std::lround(config_.lengthY / res)) | 1);
  const Eigen::Vector2d base(data_->qpos[baseQposAdr_], data_->qpos[baseQposAdr_ + 1]);
  // snap the centre to the world grid so cells do not shimmer while the robot moves
  const Eigen::Vector2d centre = (base / res).array().round().matrix() * res;
  const Eigen::Vector2d origin = centre - 0.5 * res * Eigen::Vector2d(sizeX - 1, sizeY - 1);
  auto map = std::make_shared<TerrainMap>(res, sizeX, sizeY, origin);
  map->time = captureTime_;

  const double rayStartZ = data_->qpos[baseQposAdr_ + 2] + config_.rayStartHeight;
  const int numThreads = std::max(1, std::min(config_.numThreads, sizeY));
  if (numThreads == 1) {
    castRows(*map, 0, sizeY, rayStartZ);
  } else {
    std::vector<std::thread> workers;
    const int rowsPerThread = (sizeY + numThreads - 1) / numThreads;
    for (int t = 0; t < numThreads; ++t) {
      const int begin = t * rowsPerThread;
      const int end = std::min(sizeY, begin + rowsPerThread);
      if (begin < end) {
        workers.emplace_back([this, &map, begin, end, rayStartZ] { castRows(*map, begin, end, rayStartZ); });
      }
    }
    for (auto& w : workers) {
      w.join();
    }
  }

  map->process(config_.processing);
  lastUpdateDuration_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  return map;
}

ElevationMapperConfig loadElevationMapperConfig(const std::string& file, const std::string& prefix, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  ElevationMapperConfig c;
  const std::string p = prefix + ".";
  ocs2::loadData::loadPtreeValue(pt, c.resolution, p + "resolution", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.lengthX, p + "lengthX", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.lengthY, p + "lengthY", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.rayStartHeight, p + "rayStartHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.updateRate, p + "updateRate", verbose);
  ocs2::loadData::loadPtreeValue(pt, c.numThreads, p + "numThreads", verbose);
  for (int g = 0; g < mjNGROUP; ++g) {
    bool enabled = c.geomGroups[g];
    ocs2::loadData::loadPtreeValue(pt, enabled, p + "geomGroup" + std::to_string(g), verbose);
    c.geomGroups[g] = enabled;
  }
  c.processing = loadTerrainProcessingConfig(file, prefix + ".processing", verbose);
  return c;
}

}  // namespace anymal_perceptive

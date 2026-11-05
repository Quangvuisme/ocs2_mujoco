#include "g1_perceptive_mujoco/visualization/TerrainVisualizer.h"

#include <array>
#include <cmath>

#include <Eigen/Geometry>

namespace g1_perceptive {

namespace {

mjvGeom* nextGeom(mjvScene* scene) {
  if (scene->ngeom >= scene->maxgeom) {
    return nullptr;
  }
  return scene->geoms + scene->ngeom++;
}

void addGeom(mjvScene* scene, int type, const Eigen::Vector3d& size, const Eigen::Vector3d& pos, const Eigen::Matrix3d& R,
             const std::array<float, 4>& rgba) {
  mjvGeom* g = nextGeom(scene);
  if (g == nullptr) {
    return;
  }
  mjtNum s[3] = {size.x(), size.y(), size.z()};
  mjtNum p[3] = {pos.x(), pos.y(), pos.z()};
  mjtNum mat[9];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      mat[3 * r + c] = R(r, c);
    }
  }
  mjv_initGeom(g, type, s, p, mat, rgba.data());
  g->category = mjCAT_DECOR;
  g->emission = 0.3f;
}

Eigen::Matrix3d rotationFromNormal(const Eigen::Vector3d& n) {
  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), n.normalized()).toRotationMatrix();
}

constexpr std::array<std::array<float, 4>, 4> kLegColors{{
    {0.95f, 0.25f, 0.20f, 1.0f},  // LF
    {0.20f, 0.85f, 0.30f, 1.0f},  // RF
    {0.25f, 0.45f, 0.95f, 1.0f},  // LH
    {0.95f, 0.80f, 0.15f, 1.0f},  // RH
}};

}  // namespace

void TerrainVisualizer::draw(const PerceptiveReferenceManager::Plan& plan, mjvScene* scene) const {
  scene->ngeom = 0;
  append(plan, scene);
}

void TerrainVisualizer::append(const PerceptiveReferenceManager::Plan& plan, mjvScene* scene) const {
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  if (options_.showMap && plan.terrain) {
    const TerrainMap& map = *plan.terrain;
    const int stride = std::max(1, options_.mapStride);
    const double half = 0.42 * stride * map.resolution();
    const std::array<float, 4> green{0.15f, 0.80f, 0.35f, options_.mapAlpha};
    const std::array<float, 4> red{0.90f, 0.20f, 0.15f, options_.mapAlpha};
    for (int iy = 0; iy < map.sizeY(); iy += stride) {
      for (int ix = 0; ix < map.sizeX(); ix += stride) {
        const size_t i = map.linearIndex(ix, iy);
        const float h = map.height()[i];
        if (!std::isfinite(h)) {
          continue;
        }
        const Eigen::Vector2d c = map.cellCenter(ix, iy);
        addGeom(scene, mjGEOM_BOX, Eigen::Vector3d(half, half, 0.002), Eigen::Vector3d(c.x(), c.y(), h + 0.004), I,
                map.steppable()[i] ? green : red);
      }
    }
  }

  if (options_.showFootholds) {
    for (const auto& [leg, f] : plan.footholds) {
      auto color = kLegColors[leg % 4];
      if (!f.constrained) {
        color = {0.6f, 0.6f, 0.6f, 1.0f};
      }
      addGeom(scene, mjGEOM_SPHERE, Eigen::Vector3d::Constant(0.022), f.position + Eigen::Vector3d(0, 0, 0.022), I, color);
      if (f.constrained) {
        auto regionColor = color;
        regionColor[3] = 0.35f;
        addGeom(scene, mjGEOM_CYLINDER, Eigen::Vector3d(f.regionRadius, f.regionRadius, 0.003),
                f.position + 0.006 * f.normal, rotationFromNormal(f.normal), regionColor);
      }
      // thin line from the nominal (Raibert) foothold to the selected one
      mjvGeom* g = nextGeom(scene);
      if (g != nullptr) {
        const float lineColor[4] = {color[0], color[1], color[2], 0.8f};
        mjv_initGeom(g, mjGEOM_LINE, nullptr, nullptr, nullptr, lineColor);
        const mjtNum from[3] = {f.nominal.x(), f.nominal.y(), f.position.z() + 0.01};
        const mjtNum to[3] = {f.position.x(), f.position.y(), f.position.z() + 0.01};
        mjv_connector(g, mjGEOM_LINE, 2.0, from, to);
        g->category = mjCAT_DECOR;
      }
    }
  }

  if (options_.showBaseTrajectory) {
    const std::array<float, 4> white{0.95f, 0.95f, 0.95f, 0.9f};
    for (const auto& p : plan.baseTrajectory) {
      addGeom(scene, mjGEOM_SPHERE, Eigen::Vector3d::Constant(0.012), p, I, white);
    }
  }
}

}  // namespace g1_perceptive

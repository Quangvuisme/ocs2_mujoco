/**
 * Checks the MuJoCo elevation mapper and the biped foothold planner on the stairs of the G1 course:
 *  - ray-cast heights match the step heights, the robot's own geoms are not in the map,
 *  - step edges are not steppable, tread centres are,
 *  - a sole whose nominal centre is on a step edge is moved onto one tread: toe and heel on the same
 *    level, both away from the edges, each inside its own region,
 *  - the swing obstacle height between two treads is the higher tread.
 * No test framework needed: returns non-zero on failure.
 */
#include <cmath>
#include <iostream>
#include <string>

#include <mujoco/mujoco.h>

#include "g1_perceptive_mujoco/package_path.h"
#include "g1_perceptive_mujoco/terrain/FootholdPlanner.h"
#include "g1_perceptive_mujoco/terrain/MujocoElevationMapper.h"

using namespace g1_perceptive;

namespace {
int failures = 0;

void check(bool ok, const std::string& what) {
  std::cout << (ok ? "[ OK ] " : "[FAIL] ") << what << "\n";
  failures += ok ? 0 : 1;
}

bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }
}  // namespace

int main() {
  const std::string scene = getPackagePath() + "/robots/g1/scene_terrain.xml";
  char error[1000] = "";
  mjModel* m = mj_loadXML(scene.c_str(), nullptr, error, sizeof(error));
  if (m == nullptr) {
    std::cerr << "could not load " << scene << ": " << error << "\n";
    return 1;
  }
  mjData* d = mj_makeData(m);

  // stairs: step k covers x in [x0 + T k, x0 + T (k + 1)], top at R (k + 1)
  constexpr double x0 = 1.5, T = 0.45, R = 0.08;

  // put the robot on step 0; its own geoms must not show up in the map
  const int adr = m->jnt_qposadr[0];
  d->qpos[adr + 0] = x0 + 0.5 * T;
  d->qpos[adr + 1] = 0.0;
  d->qpos[adr + 2] = R + 0.74;
  mj_forward(m, d);

  ElevationMapperConfig config;
  MujocoElevationMapper mapper(m, config);
  mapper.captureState(d);
  const auto map = mapper.update();
  check(map != nullptr, "map computed");
  if (!map) {
    return 1;
  }
  std::cout << "       map " << map->sizeX() << "x" << map->sizeY() << " cells, " << 1e3 * mapper.lastUpdateDuration()
            << " ms\n";

  for (int k = 0; k < 4; ++k) {
    const Eigen::Vector2d tread(x0 + T * (k + 0.5), 0.5);
    check(near(map->heightAt(tread), R * (k + 1), 1e-3), "height of step " + std::to_string(k));
    check(map->isSteppable(tread), "centre of step " + std::to_string(k) + " is steppable");
    const Eigen::Vector2d edge(x0 + T * k, 0.5);
    check(!map->isSteppable(edge), "edge of step " + std::to_string(k) + " is not steppable");
  }
  check(near(map->heightAt(Eigen::Vector2d(0.8, 0.0)), 0.0, 1e-3), "floor height");
  // rays at the robot's position pass through the pelvis and legs; the map must show step 0
  check(near(map->heightAt(Eigen::Vector2d(x0 + 0.5 * T, 0.0)), R, 1e-3), "robot geoms are ignored by the rays");

  // sole centre exactly on the edge between step 0 and step 1: toe and heel must end up on one tread
  FootholdPlannerConfig plannerConfig;
  plannerConfig.minEdgeDistance = 0.03;
  plannerConfig.desiredEdgeDistance = 0.08;
  FootholdPlanner planner(plannerConfig);
  planner.beginIteration();
  const Eigen::Vector2d centre(x0 + T, 0.12);
  const std::vector<Eigen::Vector2d> offsets{{0.085, 0.0}, {-0.085, 0.0}};  // toe, heel
  const auto footholds = planner.selectFoot(0, 1.0, centre, offsets, *map);
  check(footholds.size() == 2, "one foothold per support point");
  const Foothold& toe = footholds[0];
  const Foothold& heel = footholds[1];
  std::cout << "       toe " << toe.position.transpose() << "  heel " << heel.position.transpose() << "\n";
  check(toe.constrained && heel.constrained, "toe and heel have regions");
  check(near(toe.position.z(), heel.position.z(), 1e-3), "toe and heel on the same level");
  const int step = static_cast<int>(std::floor((0.5 * (toe.position.x() + heel.position.x()) - x0) / T));
  const double treadStart = x0 + T * step, treadEnd = treadStart + T;
  check(heel.position.x() - treadStart >= 0.03 && treadEnd - toe.position.x() >= 0.03, "sole is >= 3 cm from the step edges");
  check(near(toe.position.z(), R * (step + 1), 1e-3), "foothold height matches its tread");
  check((toe.A * toe.position + toe.b).minCoeff() > 0.0 && (heel.A * heel.position + heel.b).minCoeff() > 0.0,
        "toe and heel lie inside their regions");

  // swing apex between step 0 and step 2 must clear step 2
  const double obstacle =
      map->maxInflatedHeightOnSegment(Eigen::Vector2d(x0 + 0.5 * T, 0.5), Eigen::Vector2d(x0 + 2.5 * T, 0.5));
  check(obstacle >= 3 * R - 1e-3, "swing obstacle height covers the higher step");

  mj_deleteData(d);
  mj_deleteModel(m);
  std::cout << (failures == 0 ? "ALL TESTS PASSED" : std::to_string(failures) + " TEST(S) FAILED") << "\n";
  return failures == 0 ? 0 : 1;
}

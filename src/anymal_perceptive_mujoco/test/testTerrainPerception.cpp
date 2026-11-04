/**
 * Checks the MuJoCo elevation mapper and the foothold planner on the stairs of scene_terrain.xml:
 *  - ray-cast heights match the step heights,
 *  - step edges are not steppable, tread centres are,
 *  - a nominal foothold on a step edge is moved onto a tread, away from the edge,
 *  - the swing obstacle height between two treads is the higher tread.
 * No test framework needed: returns non-zero on failure.
 */
#include <cmath>
#include <iostream>
#include <string>

#include <mujoco/mujoco.h>

#include "anymal_perceptive_mujoco/package_path.h"
#include "anymal_perceptive_mujoco/terrain/FootholdPlanner.h"
#include "anymal_perceptive_mujoco/terrain/MujocoElevationMapper.h"

using namespace anymal_perceptive;

namespace {
int failures = 0;

void check(bool ok, const std::string& what) {
  std::cout << (ok ? "[ OK ] " : "[FAIL] ") << what << "\n";
  failures += ok ? 0 : 1;
}

bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }
}  // namespace

int main() {
  const std::string scene = getPackagePath() + "/robots/anymal_c/scene_terrain.xml";
  char error[1000] = "";
  mjModel* m = mj_loadXML(scene.c_str(), nullptr, error, sizeof(error));
  if (m == nullptr) {
    std::cerr << "could not load " << scene << ": " << error << "\n";
    return 1;
  }
  mjData* d = mj_makeData(m);

  // put the robot in front of the stairs (x = 1.5); its own geoms must not show up in the map
  const int freeJoint = 0;
  const int adr = m->jnt_qposadr[freeJoint];
  d->qpos[adr + 0] = 1.5;
  d->qpos[adr + 1] = 0.0;
  d->qpos[adr + 2] = 0.65;
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

  // stairs: step k covers x in [1.2 + 0.3k, 1.5 + 0.3k], top at 0.1 (k + 1)
  for (int k = 0; k < 4; ++k) {
    const Eigen::Vector2d tread(1.35 + 0.3 * k, 0.0);
    check(near(map->heightAt(tread), 0.1 * (k + 1), 1e-3), "height of step " + std::to_string(k));
    check(map->isSteppable(tread), "centre of step " + std::to_string(k) + " is steppable");
    const Eigen::Vector2d edge(1.2 + 0.3 * k, 0.0);
    check(!map->isSteppable(edge), "edge of step " + std::to_string(k) + " is not steppable");
  }
  check(near(map->heightAt(Eigen::Vector2d(0.8, 0.0)), 0.0, 1e-3), "floor height");
  // rays at (1.6, 0) pass through the robot base box; the map must show step 1 (0.2 m), not the robot
  check(near(map->heightAt(Eigen::Vector2d(1.6, 0.0)), 0.2, 1e-3), "robot geoms are ignored by the rays");

  // foothold on an edge moves onto a tread, with clearance from the edge
  FootholdPlanner planner(FootholdPlannerConfig{});
  planner.beginIteration();
  const Eigen::Vector3d nominal(1.5, 0.1, 0.0);  // exactly on the edge between step 0 and step 1
  const Foothold f = planner.select(0, 1.0, nominal, *map);
  const double edgeDistance = std::min(std::abs(f.position.x() - 1.5), std::abs(f.position.x() - 1.2));
  std::cout << "       foothold " << f.position.transpose() << " region r=" << f.regionRadius << "\n";
  check(f.constrained, "foothold has a region");
  check(edgeDistance >= 0.04, "foothold is >= 4 cm from the step edges");
  check(near(f.position.z(), f.position.x() < 1.5 ? 0.1 : 0.2, 1e-3), "foothold height matches its tread");
  check((f.A * f.position + f.b).minCoeff() > 0.0, "foothold lies inside its region");

  // swing apex between step 0 and step 2 must clear step 2
  const double obstacle = map->maxInflatedHeightOnSegment(Eigen::Vector2d(1.35, 0.0), Eigen::Vector2d(1.95, 0.0));
  check(obstacle >= 0.3 - 1e-3, "swing obstacle height covers the higher step");

  mj_deleteData(d);
  mj_deleteModel(m);
  std::cout << (failures == 0 ? "ALL TESTS PASSED" : std::to_string(failures) + " TEST(S) FAILED") << "\n";
  return failures == 0 ? 0 : 1;
}

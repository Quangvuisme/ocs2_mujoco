/**
 * @file main_plan.cpp
 * @brief Computes the offline backflip plan only (no simulation) and writes it and the guide to CSV files.
 *
 *   g1_backflip_plan [--task task.info] [--csv plan.csv] [--guide guide.csv]
 */
#include <pinocchio/fwd.hpp>

#include <iostream>
#include <string>

#include "g1_backflip_mujoco/BackflipPlanner.h"
#include "g1_backflip_mujoco/package_path.h"

using namespace g1_backflip;

int main(int argc, char** argv) {
  const std::string pkg = getPackagePath();
  std::string taskFile = pkg + "/config/task.info";
  std::string csv, guideCsv;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--task" && i + 1 < argc) {
      taskFile = argv[++i];
    } else if (a == "--csv" && i + 1 < argc) {
      csv = argv[++i];
    } else if (a == "--guide" && i + 1 < argc) {
      guideCsv = argv[++i];
    } else {
      std::cerr << "Usage: g1_backflip_plan [--task task.info] [--csv plan.csv] [--guide guide.csv]\n";
      return 1;
    }
  }
  try {
    G1WbBackflipInterface interface(taskFile, pkg + "/robots/g1_29dof/urdf/g1_29dof_flip.urdf", "plan");
    BackflipGuide guide(taskFile, interface.getPinocchioInterface(), interface.modelSettings().contactNames);
    if (!guideCsv.empty()) {
      BackflipPlan g;
      const auto targets = guide.targetTrajectories(0.01);
      g.solution.timeTrajectory_ = targets.timeTrajectory;
      g.solution.stateTrajectory_ = targets.stateTrajectory;
      g.solution.inputTrajectory_ = targets.inputTrajectory;
      writePlanCsv(g, guideCsv);
    }
    const BackflipPlan plan = computeBackflipPlan(interface, guide);
    if (!csv.empty()) {
      writePlanCsv(plan, csv);
      std::cout << "wrote " << csv << "\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] " << e.what() << "\n";
    return 1;
  }
  return 0;
}

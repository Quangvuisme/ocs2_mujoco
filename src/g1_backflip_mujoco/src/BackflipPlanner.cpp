#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include "g1_backflip_mujoco/BackflipPlanner.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_sqp/SqpSolver.h>

#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>

namespace g1_backflip {

using namespace ocs2;

namespace {

PrimalSolution solveFull(G1WbBackflipInterface& interface, const BackflipGuide& guide, const TargetTrajectories& targets, size_t iterations) {
  sqp::Settings settings = interface.sqpSettings();
  settings.sqpIteration = iterations;
  SqpSolver solver(settings, interface.getOptimalControlProblem(), interface.getInitializer());
  solver.setReferenceManager(interface.getReferenceManagerPtr());

  // initial guess: the guide itself
  PrimalSolution initialGuess;
  initialGuess.timeTrajectory_ = targets.timeTrajectory;
  initialGuess.stateTrajectory_ = targets.stateTrajectory;
  initialGuess.inputTrajectory_ = targets.inputTrajectory;
  initialGuess.modeSchedule_ = guide.modeSchedule();
  solver.run(0.0, guide.initialState(), humanoid::ModeNumber::STANCE, guide.endTime(), initialGuess);
  return solver.primalSolution(guide.endTime());
}

}  // namespace

BackflipPlan computeBackflipPlan(G1WbBackflipInterface& interface, const BackflipGuide& guide, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(interface.taskFile(), pt);

  auto referenceManager = interface.getReferenceManagerPtr();
  const TargetTrajectories targets = guide.targetTrajectories(0.01);
  referenceManager->setTargetTrajectories(targets);
  referenceManager->setBackflipSchedule(guide.modeSchedule());

  const auto start = std::chrono::steady_clock::now();
  BackflipPlan plan;
  plan.solution = solveFull(interface, guide, targets, pt.get<size_t>("plan.iterations"));
  plan.solution.modeSchedule_ = guide.modeSchedule();
  plan.liftoffTime = guide.liftoffTime();
  plan.touchdownTime = guide.touchdownTime();
  plan.endTime = guide.endTime();
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  if (verbose) {
    auto& model = interface.getMpcRobotModel();
    auto& pin = interface.getPinocchioInterface();
    const auto& pm = pin.getModel();
    const size_t nj = model.getJointDim();
    const auto& sol = plan.solution;
    const auto& t = sol.timeTrajectory_;
    const auto& xs = sol.stateTrajectory_;
    const auto& us = sol.inputTrajectory_;
    const auto at = [&](double time) {
      size_t k = 0;
      while (k + 1 < t.size() && t[k + 1] <= time) {
        ++k;
      }
      return k;
    };
    double maxForce = 0.0, apex = 0.0, maxPitchSpeed = 0.0;
    vector_t maxRatio = vector_t::Zero(nj), maxTorque = vector_t::Zero(nj);
    double maxRatioTime = 0.0, worstRatio = 0.0;
    for (size_t k = 0; k < t.size(); ++k) {
      maxForce = std::max(maxForce, us[k](2) + us[k](8));
      apex = std::max(apex, xs[k](2));
      maxPitchSpeed = std::max(maxPitchSpeed, std::abs(xs[k](6 + nj + 5)));
      if (k + 1 < t.size() && t[k + 1] - t[k] < 1e-9) {
        continue;  // pre-event duplicate node: its input is never applied (zero-order hold over the intervals)
      }
      const vector_t tau = humanoid::computeJointTorques<scalar_t>(xs[k], us[k], pin, model);
      for (size_t j = 0; j < nj; ++j) {
        const double ratio = std::abs(tau(j)) / pm.effortLimit(6 + j);
        maxTorque(j) = std::max(maxTorque(j), std::abs(tau(j)));
        maxRatio(j) = std::max(maxRatio(j), ratio);
        if (ratio > worstRatio) {
          worstRatio = ratio;
          maxRatioTime = t[k];
        }
      }
    }
    // physics check: the centroidal angular momentum must stay constant while both feet are in the air
    double lMin = 1e9, lMax = -1e9;
    {
      pinocchio::Data data(pm);
      for (size_t k = 0; k < t.size(); ++k) {
        if (t[k] > plan.liftoffTime + 1e-3 && t[k] < plan.touchdownTime - 1e-3) {
          const vector_t q = xs[k].head(6 + nj), v = xs[k].tail(6 + nj);
          const double ly = pinocchio::computeCentroidalMomentum(pm, data, q, v).angular().y();
          lMin = std::min(lMin, ly);
          lMax = std::max(lMax, ly);
        }
      }
    }
    const auto& xLo = xs[at(plan.liftoffTime - 1e-6)];
    const auto& xTd = xs[at(plan.touchdownTime - 1e-6)];
    const auto& xEnd = xs.back();
    std::cout << std::fixed << std::setprecision(3) << "[Plan] OCS2 SQP over the manoeuvre: " << seconds << " s, " << t.size() << " nodes\n"
              << "[Plan] lift-off   t=" << plan.liftoffTime << ": pelvis v=(" << xLo(6 + nj) << ", " << xLo(8 + nj)
              << ") m/s, pitch=" << xLo(5) << " rad, pitch rate=" << xLo(11 + nj) << " rad/s, pelvis z=" << xLo(2) << "\n"
              << "[Plan] touch-down t=" << plan.touchdownTime << ": pelvis v=(" << xTd(6 + nj) << ", " << xTd(8 + nj)
              << ") m/s, pitch=" << xTd(5) << " rad, pelvis=(" << xTd(0) << ", " << xTd(2) << ")\n"
              << "[Plan] end: pitch=" << xEnd(5) << " rad, pelvis=(" << xEnd(0) << ", " << xEnd(2) << "), apex pelvis z=" << apex
              << ", peak pitch rate " << maxPitchSpeed << " rad/s\n"
              << "[Plan] flight angular momentum about the pitch axis: " << lMin << " .. " << lMax << " N m s (constant if physical)\n"
              << "[Plan] peak normal force " << maxForce << " N (" << maxForce / (interface.totalMass() * 9.81)
              << " body weights); worst torque / limit " << worstRatio << " at t=" << maxRatioTime << "\n[Plan] peak |torque| / limit:";
    for (size_t j = 0; j < nj; ++j) {
      const std::string& name = pm.names[2 + j];
      std::cout << " " << name.substr(0, name.size() - 6) << "=" << std::setprecision(0) << maxTorque(j) << "Nm(" << std::setprecision(2)
                << maxRatio(j) << ")";
      if (j == 11 || j == 14) std::cout << "\n          ";
    }
    std::cout << std::setprecision(3) << "\n";
  }
  return plan;
}

void writePlanCsv(const BackflipPlan& plan, const std::string& file) {
  std::ofstream out(file);
  const auto& sol = plan.solution;
  if (sol.timeTrajectory_.empty()) {
    return;
  }
  out << "t";
  for (long i = 0; i < sol.stateTrajectory_.front().size(); ++i) out << ",x" << i;
  for (long i = 0; i < sol.inputTrajectory_.front().size(); ++i) out << ",u" << i;
  out << "\n";
  for (size_t k = 0; k < sol.timeTrajectory_.size(); ++k) {
    out << sol.timeTrajectory_[k];
    for (long i = 0; i < sol.stateTrajectory_[k].size(); ++i) out << "," << sol.stateTrajectory_[k](i);
    for (long i = 0; i < sol.inputTrajectory_[k].size(); ++i) out << "," << sol.inputTrajectory_[k](i);
    out << "\n";
  }
}

}  // namespace g1_backflip

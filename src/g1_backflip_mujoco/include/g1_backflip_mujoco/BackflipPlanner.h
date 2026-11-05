#pragma once

#include <string>

#include <ocs2_oc/oc_data/PrimalSolution.h>

#include "g1_backflip_mujoco/BackflipGuide.h"
#include "g1_backflip_mujoco/G1WbBackflipInterface.h"

namespace g1_backflip {

/** Offline backflip plan on the whole-body model (stand, crouch, push, flight, landing, stand). */
struct BackflipPlan {
  ocs2::PrimalSolution solution;  // times relative to the start of the manoeuvre
  double liftoffTime = 0.0;
  double touchdownTime = 0.0;
  double endTime = 0.0;
};

/**
 * Solves the plan with OCS2 SQP (multiple shooting, HPIPM) on the "plan" problem of the interface: one SQP over the
 * whole manoeuvre tracking the guide, initialised with the guide's state / input trajectories. Prints a summary (lift-off velocity, rotation, peak forces, joint torques against the limits).
 */
BackflipPlan computeBackflipPlan(G1WbBackflipInterface& planInterface, const BackflipGuide& guide, bool verbose = true);

/** Writes a plan (time, state, input) to a CSV file. */
void writePlanCsv(const BackflipPlan& plan, const std::string& file);

}  // namespace g1_backflip

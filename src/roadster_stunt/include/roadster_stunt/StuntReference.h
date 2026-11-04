#pragma once

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#include "roadster_stunt/RoadsterParameters.h"

namespace roadster_stunt {

/**
 * Hand-designed guide for the offline stunt plan: climb to a cruise height above the row, hold it while
 * crossing, come down to just above touchdown height (the flare), and roll a full turn (0 -> 2 pi) in between. It is not
 * dynamically feasible (the climb asks for more than the jets give); the SLQ plan only tracks it loosely.
 * Inputs are zero, so the input cost acts as an air (fuel) penalty.
 */
ocs2::TargetTrajectories designStuntGuide(const RoadsterParameters& roadster, const StuntParameters& stunt, ocs2::scalar_t dt);

/**
 * Targets for the flight NMPC: the optimized plan resampled at dt, followed by a landing tail of tailDuration
 * that keeps descending at landingSinkRate, level, with hover thrust as the input reference.
 */
ocs2::TargetTrajectories planToTargets(const ocs2::PrimalSolution& plan, const RoadsterParameters& roadster,
                                       const StuntParameters& stunt, ocs2::scalar_t dt, ocs2::scalar_t tailDuration);

/** Initial guess from a target trajectory: its input at each time, state held constant. */
class TargetInputInitializer final : public ocs2::Initializer {
 public:
  explicit TargetInputInitializer(ocs2::TargetTrajectories targets) : targets_(std::move(targets)) {}

  ~TargetInputInitializer() override = default;

  TargetInputInitializer* clone() const override { return new TargetInputInitializer(*this); }

  void compute(ocs2::scalar_t time, const ocs2::vector_t& state, ocs2::scalar_t nextTime, ocs2::vector_t& input,
               ocs2::vector_t& nextState) override {
    input = targets_.getDesiredInput(time);
    nextState = state;
  }

 private:
  TargetInputInitializer(const TargetInputInitializer& other) = default;

  ocs2::TargetTrajectories targets_;
};

}  // namespace roadster_stunt

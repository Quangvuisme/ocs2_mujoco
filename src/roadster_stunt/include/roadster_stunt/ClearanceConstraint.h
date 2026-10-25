#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

#include "roadster_stunt/RoadsterParameters.h"

namespace roadster_stunt {

/**
 * Keeps the car above the parked row: h(t, x) = z - zMin(t) >= 0.
 *
 * The car crosses the row at the launch speed, so the required height is a smooth bump over time:
 *   zMin(t) = rideHeight + (rowHeight + clearanceRadius - rideHeight) * bump(launchSpeed * t)
 * where bump() is ~1 from rowStart - rowMargin to rowStart + rowLength + rowMargin and ~0 elsewhere.
 * Time is measured from the launch line. Used as a soft constraint (relaxed barrier).
 */
class ClearanceConstraint final : public ocs2::StateConstraint {
 public:
  ClearanceConstraint(const RoadsterParameters& roadster, const StuntParameters& stunt);

  ~ClearanceConstraint() override = default;

  ClearanceConstraint* clone() const override { return new ClearanceConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t time) const override { return 1; }

  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation&) const override;

  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComp) const override;

  /** Minimum body height [m] at a time after launch. */
  ocs2::scalar_t minHeight(ocs2::scalar_t time) const;

 private:
  ClearanceConstraint(const ClearanceConstraint& other) = default;

  ocs2::scalar_t speed_;
  ocs2::scalar_t xStart_;
  ocs2::scalar_t xEnd_;
  ocs2::scalar_t baseHeight_;
  ocs2::scalar_t extraHeight_;
  ocs2::scalar_t sharpness_ = 8.0;  // [1/m] slope of the bump edges
};

}  // namespace roadster_stunt

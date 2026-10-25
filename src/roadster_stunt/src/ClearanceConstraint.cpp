#include "roadster_stunt/ClearanceConstraint.h"

#include <cmath>

#include "roadster_stunt/RoadsterDefinition.h"

namespace roadster_stunt {

namespace {
ocs2::scalar_t sigmoid(ocs2::scalar_t v) {
  return 1.0 / (1.0 + std::exp(-v));
}
}  // namespace

ClearanceConstraint::ClearanceConstraint(const RoadsterParameters& roadster, const StuntParameters& stunt)
    : ocs2::StateConstraint(ocs2::ConstraintOrder::Linear),
      speed_(stunt.launchSpeed_),
      xStart_(stunt.rowStart_ - stunt.rowMargin_),
      xEnd_(stunt.rowStart_ + stunt.rowLength_ + stunt.rowMargin_),
      baseHeight_(roadster.rideHeight_),
      extraHeight_(stunt.rowHeight_ + stunt.clearanceRadius_ - roadster.rideHeight_) {}

ocs2::scalar_t ClearanceConstraint::minHeight(ocs2::scalar_t time) const {
  const ocs2::scalar_t x = speed_ * time;
  const ocs2::scalar_t bump = sigmoid(sharpness_ * (x - xStart_)) * sigmoid(sharpness_ * (xEnd_ - x));
  return baseHeight_ + extraHeight_ * bump;
}

ocs2::vector_t ClearanceConstraint::getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation&) const {
  ocs2::vector_t h(1);
  h(0) = state(Z) - minHeight(time);
  return h;
}

ocs2::VectorFunctionLinearApproximation ClearanceConstraint::getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                                    const ocs2::PreComputation& preComp) const {
  ocs2::VectorFunctionLinearApproximation h(1, state.size());
  h.f = getValue(time, state, preComp);
  h.dfdx.setZero();
  h.dfdx(0, Z) = 1.0;
  return h;
}

}  // namespace roadster_stunt

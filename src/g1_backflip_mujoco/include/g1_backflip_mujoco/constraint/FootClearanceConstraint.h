#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "g1_backflip_mujoco/BackflipReferenceManager.h"

namespace g1_backflip {

/**
 * Keeps a frame above the ground: h = z_frame - minHeight >= 0 (relaxed barrier).
 *  - feet, while in the air: without it the plan lets the feet sink through the floor at the end of the flight
 *    (nothing else acts on a free foot), and the real robot then lands early;
 *  - knees, always: otherwise the MPC may keep the legs tucked into the landing and put a knee on the floor.
 */
class FootClearanceConstraint final : public ocs2::StateConstraint {
 public:
  FootClearanceConstraint(const BackflipReferenceManager& referenceManager, const ocs2::EndEffectorKinematics<ocs2::scalar_t>& kinematics,
                          int contactIndex, ocs2::scalar_t minHeight)
      : StateConstraint(ocs2::ConstraintOrder::Linear),
        referenceManagerPtr_(&referenceManager),
        kinematicsPtr_(kinematics.clone()),
        contactIndex_(contactIndex),
        minHeight_(minHeight) {}

  FootClearanceConstraint* clone() const override { return new FootClearanceConstraint(*this); }

  /** contactIndex < 0: always active (e.g. a knee); otherwise only while that contact is in the air. */
  bool isActive(ocs2::scalar_t time) const override {
    return contactIndex_ < 0 || !referenceManagerPtr_->getContactFlags(time)[contactIndex_];
  }
  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return 1; }

  ocs2::vector_t getValue(ocs2::scalar_t /*time*/, const ocs2::vector_t& state, const ocs2::PreComputation& /*preComp*/) const override {
    ocs2::vector_t h(1);
    h(0) = kinematicsPtr_->getPosition(state).front().z() - minHeight_;
    return h;
  }

  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t /*time*/, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& /*preComp*/) const override {
    const auto position = kinematicsPtr_->getPositionLinearApproximation(state).front();
    ocs2::VectorFunctionLinearApproximation h;
    h.f = position.f.tail<1>().array() - minHeight_;
    h.dfdx = position.dfdx.bottomRows<1>();
    h.dfdu.setZero(1, 0);
    return h;
  }

 private:
  FootClearanceConstraint(const FootClearanceConstraint& rhs)
      : StateConstraint(rhs),
        referenceManagerPtr_(rhs.referenceManagerPtr_),
        kinematicsPtr_(rhs.kinematicsPtr_->clone()),
        contactIndex_(rhs.contactIndex_),
        minHeight_(rhs.minHeight_) {}

  const BackflipReferenceManager* referenceManagerPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> kinematicsPtr_;
  int contactIndex_;
  ocs2::scalar_t minHeight_;
};

}  // namespace g1_backflip

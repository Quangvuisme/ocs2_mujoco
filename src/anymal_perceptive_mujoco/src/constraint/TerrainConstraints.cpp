#include "anymal_perceptive_mujoco/constraint/TerrainConstraints.h"

namespace anymal_perceptive {

// ---------------------------------------------------------------------------------------------
// FootPlacementConstraint
// ---------------------------------------------------------------------------------------------
FootPlacementConstraint::FootPlacementConstraint(const PerceptiveReferenceManager& referenceManager,
                                                 const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                                                 size_t contactPointIndex, size_t numVertices)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      numVertices_(numVertices) {}

FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint& rhs)
    : StateConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      numVertices_(rhs.numVertices_) {}

bool FootPlacementConstraint::isActive(ocs2::scalar_t time) const {
  return referenceManagerPtr_->isFootPlacementActive(contactPointIndex_, time);
}

ocs2::vector_t FootPlacementConstraint::getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                 const ocs2::PreComputation& /*preComp*/) const {
  const Foothold* f = referenceManagerPtr_->getFoothold(contactPointIndex_, time);
  if (f == nullptr || !f->constrained || static_cast<size_t>(f->A.rows()) != numVertices_) {
    return ocs2::vector_t::Ones(numVertices_);
  }
  return f->A * endEffectorKinematicsPtr_->getPosition(state).front() + f->b;
}

ocs2::VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(
    ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& /*preComp*/) const {
  auto approx = ocs2::VectorFunctionLinearApproximation::Zero(numVertices_, state.size(), 0);
  const Foothold* f = referenceManagerPtr_->getFoothold(contactPointIndex_, time);
  if (f == nullptr || !f->constrained || static_cast<size_t>(f->A.rows()) != numVertices_) {
    approx.f.setOnes();
    return approx;
  }
  const auto position = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  approx.f = f->A * position.f + f->b;
  approx.dfdx = f->A * position.dfdx;
  return approx;
}

// ---------------------------------------------------------------------------------------------
// FootClearanceConstraint
// ---------------------------------------------------------------------------------------------
FootClearanceConstraint::FootClearanceConstraint(const PerceptiveReferenceManager& referenceManager,
                                                 const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                                                 size_t contactPointIndex)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex) {}

FootClearanceConstraint::FootClearanceConstraint(const FootClearanceConstraint& rhs)
    : StateConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

bool FootClearanceConstraint::isActive(ocs2::scalar_t time) const {
  const auto& s = referenceManagerPtr_->settings();
  const auto& rm = *referenceManagerPtr_;
  return !rm.getContactFlags(time)[contactPointIndex_] &&
         !rm.getContactFlags(time - s.swingConstraintLiftOffMargin)[contactPointIndex_] &&
         !rm.getContactFlags(time + s.swingConstraintTouchDownMargin)[contactPointIndex_];
}

ocs2::vector_t FootClearanceConstraint::getValue(ocs2::scalar_t /*time*/, const ocs2::vector_t& state,
                                                 const ocs2::PreComputation& /*preComp*/) const {
  const auto& s = referenceManagerPtr_->settings();
  const Eigen::Vector3d p = endEffectorKinematicsPtr_->getPosition(state).front();
  const double h = referenceManagerPtr_->terrain().inflatedHeightAt(p.head<2>());
  ocs2::vector_t value(1);
  value(0) = p.z() - h - s.footFrameHeight - s.swingClearance;
  return value;
}

ocs2::VectorFunctionLinearApproximation FootClearanceConstraint::getLinearApproximation(
    ocs2::scalar_t /*time*/, const ocs2::vector_t& state, const ocs2::PreComputation& /*preComp*/) const {
  const auto& s = referenceManagerPtr_->settings();
  const auto position = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  Eigen::Vector2d gradient;
  const double h = referenceManagerPtr_->terrain().inflatedHeightAt(position.f.head<2>(), &gradient);
  auto approx = ocs2::VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
  approx.f(0) = position.f.z() - h - s.footFrameHeight - s.swingClearance;
  const Eigen::RowVector3d dgdp(-gradient.x(), -gradient.y(), 1.0);
  approx.dfdx = dgdp * position.dfdx;
  return approx;
}

}  // namespace anymal_perceptive

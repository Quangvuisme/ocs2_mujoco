#include "g1_perceptive_mujoco/constraint/TerrainConstraints.h"

namespace g1_perceptive {

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
                                                 size_t contactPointIndex, double extraClearance)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      extraClearance_(extraClearance) {}

FootClearanceConstraint::FootClearanceConstraint(const FootClearanceConstraint& rhs)
    : StateConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      extraClearance_(rhs.extraClearance_) {}

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
  value(0) = p.z() - h - s.footFrameHeight - s.swingClearance - extraClearance_;
  return value;
}

ocs2::VectorFunctionLinearApproximation FootClearanceConstraint::getLinearApproximation(
    ocs2::scalar_t /*time*/, const ocs2::vector_t& state, const ocs2::PreComputation& /*preComp*/) const {
  const auto& s = referenceManagerPtr_->settings();
  const auto position = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  Eigen::Vector2d gradient;
  const double h = referenceManagerPtr_->terrain().inflatedHeightAt(position.f.head<2>(), &gradient);
  auto approx = ocs2::VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
  approx.f(0) = position.f.z() - h - s.footFrameHeight - s.swingClearance - extraClearance_;
  const double g = referenceManagerPtr_->settings().clearanceGradientScale;
  const Eigen::RowVector3d dgdp(-g * gradient.x(), -g * gradient.y(), 1.0);
  approx.dfdx = dgdp * position.dfdx;
  return approx;
}

// ---------------------------------------------------------------------------------------------
// ShankClearanceConstraint
// ---------------------------------------------------------------------------------------------
ShankClearanceConstraint::ShankClearanceConstraint(const PerceptiveReferenceManager& referenceManager,
                                                   const ocs2::EndEffectorKinematics<ocs2::scalar_t>& kneeFootKinematics,
                                                   std::vector<double> fractions, double radius, int swingContactIndex)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      kinematicsPtr_(kneeFootKinematics.clone()),
      fractions_(std::move(fractions)),
      radius_(radius),
      swingContactIndex_(swingContactIndex) {
  if (kinematicsPtr_->getIds().size() != 2) {
    throw std::runtime_error("[ShankClearanceConstraint] the kinematics must have exactly two frames: knee, foot");
  }
}

ShankClearanceConstraint::ShankClearanceConstraint(const ShankClearanceConstraint& rhs)
    : StateConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      kinematicsPtr_(rhs.kinematicsPtr_->clone()),
      fractions_(rhs.fractions_),
      radius_(rhs.radius_),
      swingContactIndex_(rhs.swingContactIndex_) {}

bool ShankClearanceConstraint::isActive(ocs2::scalar_t time) const {
  return swingContactIndex_ < 0 || !referenceManagerPtr_->getContactFlags(time)[static_cast<size_t>(swingContactIndex_)];
}

ocs2::vector_t ShankClearanceConstraint::getValue(ocs2::scalar_t /*time*/, const ocs2::vector_t& state,
                                                  const ocs2::PreComputation& /*preComp*/) const {
  const auto positions = kinematicsPtr_->getPosition(state);
  const Eigen::Vector3d& knee = positions[0];
  const Eigen::Vector3d& foot = positions[1];
  const auto& map = referenceManagerPtr_->terrain();
  ocs2::vector_t value(fractions_.size());
  for (size_t k = 0; k < fractions_.size(); ++k) {
    const Eigen::Vector3d p = foot + fractions_[k] * (knee - foot);
    value(k) = p.z() - map.inflatedHeightAt(p.head<2>()) - radius_;
  }
  return value;
}

ocs2::VectorFunctionLinearApproximation ShankClearanceConstraint::getLinearApproximation(
    ocs2::scalar_t /*time*/, const ocs2::vector_t& state, const ocs2::PreComputation& /*preComp*/) const {
  const auto approximations = kinematicsPtr_->getPositionLinearApproximation(state);
  const auto& knee = approximations[0];
  const auto& foot = approximations[1];
  const auto& map = referenceManagerPtr_->terrain();
  auto approx = ocs2::VectorFunctionLinearApproximation::Zero(fractions_.size(), state.size(), 0);
  for (size_t k = 0; k < fractions_.size(); ++k) {
    const double s = fractions_[k];
    const Eigen::Vector3d p = foot.f + s * (knee.f - foot.f);
    Eigen::Vector2d gradient;
    const double h = map.inflatedHeightAt(p.head<2>(), &gradient);
    approx.f(k) = p.z() - h - radius_;
    const double g = referenceManagerPtr_->settings().clearanceGradientScale;
    const Eigen::RowVector3d dgdp(-g * gradient.x(), -g * gradient.y(), 1.0);
    approx.dfdx.row(k) = dgdp * ((1.0 - s) * foot.dfdx + s * knee.dfdx);
  }
  return approx;
}

}  // namespace g1_perceptive

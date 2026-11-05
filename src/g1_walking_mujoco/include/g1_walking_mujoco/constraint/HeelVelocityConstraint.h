#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "g1_walking_mujoco/reference_manager/SwitchedModelReferenceManager.h"

namespace g1_walking {

/**
 * Stance constraint of the heel contact point of a foot whose toe point already has a zero velocity
 * constraint (ZeroVelocityConstraintCppAd):
 *
 *   g = [n1, n2]^T v_heel = 0,   a = (p_toe - p_heel) / |p_toe - p_heel|,  n1 = (e_z x a) / |e_z x a|,  n2 = a x n1
 *
 * For a rigid foot the heel velocity along the toe-heel axis a always equals that of the toe, so a full
 * 3D heel constraint would repeat a toe row and make the stance constraints rank deficient (the DDP
 * constraint projection then fails). Only the two components perpendicular to a are constrained.
 * The axis is taken as constant in the linearisation (its derivative multiplies v_heel = 0).
 */
class HeelVelocityConstraint final : public ocs2::StateInputConstraint {
 public:
  /**
   * @param [in] heelKinematics, toeKinematics : kinematics of the heel / toe contact points of the same foot.
   * @param [in] heelContactIndex : contact index of the heel (its contact flag activates the constraint).
   */
  HeelVelocityConstraint(const SwitchedModelReferenceManager& referenceManager,
                         const ocs2::EndEffectorKinematics<ocs2::scalar_t>& heelKinematics,
                         const ocs2::EndEffectorKinematics<ocs2::scalar_t>& toeKinematics, size_t heelContactIndex)
      : StateInputConstraint(ocs2::ConstraintOrder::Linear),
        referenceManagerPtr_(&referenceManager),
        heelKinematicsPtr_(heelKinematics.clone()),
        toeKinematicsPtr_(toeKinematics.clone()),
        heelContactIndex_(heelContactIndex) {}

  ~HeelVelocityConstraint() override = default;
  HeelVelocityConstraint* clone() const override { return new HeelVelocityConstraint(*this); }

  bool isActive(ocs2::scalar_t time) const override { return referenceManagerPtr_->getContactFlags(time)[heelContactIndex_]; }

  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return 2; }

  ocs2::vector_t getValue(ocs2::scalar_t /*time*/, const ocs2::vector_t& state, const ocs2::vector_t& input,
                          const ocs2::PreComputation& /*preComp*/) const override {
    return projection(state) * heelKinematicsPtr_->getVelocity(state, input).front();
  }

  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t /*time*/, const ocs2::vector_t& state,
                                                                 const ocs2::vector_t& input,
                                                                 const ocs2::PreComputation& /*preComp*/) const override {
    const Eigen::Matrix<ocs2::scalar_t, 2, 3> N = projection(state);
    const auto v = heelKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
    ocs2::VectorFunctionLinearApproximation approx;
    approx.f = N * v.f;
    approx.dfdx = N * v.dfdx;
    approx.dfdu = N * v.dfdu;
    return approx;
  }

 private:
  HeelVelocityConstraint(const HeelVelocityConstraint& rhs)
      : StateInputConstraint(rhs),
        referenceManagerPtr_(rhs.referenceManagerPtr_),
        heelKinematicsPtr_(rhs.heelKinematicsPtr_->clone()),
        toeKinematicsPtr_(rhs.toeKinematicsPtr_->clone()),
        heelContactIndex_(rhs.heelContactIndex_) {}

  /** Rows n1^T, n2^T: the directions perpendicular to the toe-heel axis. */
  Eigen::Matrix<ocs2::scalar_t, 2, 3> projection(const ocs2::vector_t& state) const {
    const vector3_t pHeel = heelKinematicsPtr_->getPosition(state).front();
    const vector3_t pToe = toeKinematicsPtr_->getPosition(state).front();
    vector3_t a = pToe - pHeel;
    a = a.norm() > 1e-6 ? vector3_t(a.normalized()) : vector3_t::UnitX();
    vector3_t n1 = vector3_t::UnitZ().cross(a);
    n1 = n1.norm() > 1e-6 ? vector3_t(n1.normalized()) : vector3_t::UnitY();
    const vector3_t n2 = a.cross(n1);
    Eigen::Matrix<ocs2::scalar_t, 2, 3> N;
    N.row(0) = n1.transpose();
    N.row(1) = n2.transpose();
    return N;
  }

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> heelKinematicsPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> toeKinematicsPtr_;
  const size_t heelContactIndex_;
};

}  // namespace g1_walking

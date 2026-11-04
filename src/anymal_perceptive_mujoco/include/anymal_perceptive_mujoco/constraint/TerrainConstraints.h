#pragma once

#include <memory>
#include <vector>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "anymal_perceptive_mujoco/reference_manager/PerceptiveReferenceManager.h"

namespace anymal_perceptive {

/**
 * Keeps the foot of a future stance phase inside the convex region around its planned foothold:
 *     A * p_foot + b >= 0   (one row per polygon edge)
 * Active during stance phases that start after the current one (see
 * PerceptiveReferenceManager::isFootPlacementActive). Meant to be wrapped in a StateSoftConstraint.
 */
class FootPlacementConstraint final : public ocs2::StateConstraint {
 public:
  FootPlacementConstraint(const PerceptiveReferenceManager& referenceManager,
                          const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics, size_t contactPointIndex,
                          size_t numVertices);
  ~FootPlacementConstraint() override = default;
  FootPlacementConstraint* clone() const override { return new FootPlacementConstraint(*this); }

  bool isActive(ocs2::scalar_t time) const override;
  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return numVertices_; }
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComp) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  FootPlacementConstraint(const FootPlacementConstraint& rhs);

  const PerceptiveReferenceManager* referenceManagerPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
  const size_t contactPointIndex_;
  const size_t numVertices_;
};

/**
 * Keeps a swinging foot above the (inflated) elevation map:
 *     p_z - h_inflated(p_x, p_y) - footFrameHeight - clearance >= 0
 * The gradient of the map pushes the foot away from step risers and box edges. Active only in the
 * middle of the swing, away from lift-off and touch-down. Meant to be wrapped in a StateSoftConstraint.
 */
class FootClearanceConstraint final : public ocs2::StateConstraint {
 public:
  FootClearanceConstraint(const PerceptiveReferenceManager& referenceManager,
                          const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics, size_t contactPointIndex);
  ~FootClearanceConstraint() override = default;
  FootClearanceConstraint* clone() const override { return new FootClearanceConstraint(*this); }

  bool isActive(ocs2::scalar_t time) const override;
  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return 1; }
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComp) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  FootClearanceConstraint(const FootClearanceConstraint& rhs);

  const PerceptiveReferenceManager* referenceManagerPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
  const size_t contactPointIndex_;
};

/**
 * Keeps points on the shank (between the foot and the knee) above the inflated elevation map:
 *     p_z - h_inflated(p_x, p_y) - radius >= 0,   p = foot + s (knee - foot),  s in `fractions`
 * Stops the lower legs from scraping step noses and risers. The kinematics must provide two frames:
 * the knee (index 0) and the foot (index 1). Meant to be wrapped in a StateSoftConstraint.
 */
class ShankClearanceConstraint final : public ocs2::StateConstraint {
 public:
  ShankClearanceConstraint(const PerceptiveReferenceManager& referenceManager,
                           const ocs2::EndEffectorKinematics<ocs2::scalar_t>& kneeFootKinematics, std::vector<double> fractions,
                           double radius);
  ~ShankClearanceConstraint() override = default;
  ShankClearanceConstraint* clone() const override { return new ShankClearanceConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t /*time*/) const override { return fractions_.size(); }
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComp) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  ShankClearanceConstraint(const ShankClearanceConstraint& rhs);

  const PerceptiveReferenceManager* referenceManagerPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> kinematicsPtr_;
  const std::vector<double> fractions_;
  const double radius_;
};

}  // namespace anymal_perceptive

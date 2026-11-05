#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include "g1_perceptive_mujoco/controller/LegPostureIk.h"

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace g1_perceptive {

namespace {
/** Generalized coordinates with the root at the origin (identity orientation) and the given joints. */
Eigen::VectorXd fixedBaseConfiguration(const pinocchio::Model& model, const ocs2::vector_t& joints) {
  Eigen::VectorXd q = pinocchio::neutral(model);
  q.tail(joints.size()) = joints;
  return q;
}
}  // namespace

LegPostureIk::LegPostureIk(const ocs2::PinocchioInterface& pinocchioInterface, ocs2::vector_t defaultJoints,
                           std::vector<std::string> footFrames)
    : pinocchioInterface_(pinocchioInterface), defaultJoints_(std::move(defaultJoints)), cachedJoints_(defaultJoints_) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  for (const auto& name : footFrames) {
    if (!model.existFrame(name)) {
      throw std::invalid_argument("[LegPostureIk] frame not found: " + name);
    }
    footFrameIds_.push_back(model.getFrameId(name));
  }
  pinocchio::framesForwardKinematics(model, data, fixedBaseConfiguration(model, defaultJoints_));
  for (const auto id : footFrameIds_) {
    defaultFootRotations_.push_back(data.oMf[id].rotation());
    defaultFootPositions_.push_back(data.oMf[id].translation());
  }
}

const ocs2::vector_t& LegPostureIk::solve(double heightOffset) {
  if (std::abs(heightOffset - cachedOffset_) < 1e-4) {
    return cachedJoints_;
  }
  cachedJoints_ = iterate(defaultJoints_, std::vector<double>(footFrameIds_.size(), -heightOffset), 50, 1e-8);
  cachedOffset_ = heightOffset;
  return cachedJoints_;
}

ocs2::vector_t LegPostureIk::solveFeet(const std::vector<double>& footOffsets) {
  if (warmStart_.size() != defaultJoints_.size()) {
    warmStart_ = defaultJoints_;
  }
  warmStart_ = iterate(warmStart_, footOffsets, 20, 1e-6);
  return warmStart_;
}

ocs2::vector_t LegPostureIk::iterate(ocs2::vector_t joints, const std::vector<double>& footOffsets, int maxIterations,
                                     double tolerance) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const Eigen::Index nj = defaultJoints_.size();
  const Eigen::Index nv = model.nv;
  constexpr double kDamping = 1e-6;
  constexpr double kMaxStep = 0.3;  // [rad] per iteration
  for (int it = 0; it < maxIterations; ++it) {
    const Eigen::VectorXd q = fixedBaseConfiguration(model, joints);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    Eigen::VectorXd error(6 * footFrameIds_.size());
    Eigen::MatrixXd J(6 * footFrameIds_.size(), nj);
    for (size_t k = 0; k < footFrameIds_.size(); ++k) {
      Eigen::Vector3d targetPosition = defaultFootPositions_[k];
      targetPosition.z() += footOffsets[k];
      const pinocchio::SE3 target(defaultFootRotations_[k], targetPosition);
      const pinocchio::SE3 current = data.oMf[footFrameIds_[k]];
      error.segment<6>(6 * k) = pinocchio::log6(current.actInv(target)).toVector();  // in the local frame
      Eigen::MatrixXd Jk = Eigen::MatrixXd::Zero(6, nv);
      pinocchio::getFrameJacobian(model, data, footFrameIds_[k], pinocchio::LOCAL, Jk);
      J.middleRows<6>(6 * k) = Jk.rightCols(nj);
    }
    if (error.norm() < tolerance) {
      break;
    }
    const Eigen::MatrixXd JJt = J * J.transpose() + kDamping * Eigen::MatrixXd::Identity(J.rows(), J.rows());
    const Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(error);
    joints += dq.cwiseMax(-kMaxStep).cwiseMin(kMaxStep);
  }
  return joints;
}

}  // namespace g1_perceptive

/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/*
 * Copy of wb_humanoid_mpc humanoid_nmpc/humanoid_common_mpc/src/pinocchio_model/DynamicsHelperFunctions.cpp
 * (submodule commit 6fb34bb) with two fixes, compiled instead of the original by g1_backflip_mujoco
 * (see cmake/WbHumanoidMpc.cmake). Marked "g1_backflip_mujoco fix" below.
 *
 *  1. computeBaseAcceleration() treated the 6x6 base block of the mass matrix as block diagonal (linear / angular)
 *     and dropped the coupling m [r_com]x between them, which is only zero when the base origin is at the centre of
 *     mass. The base accelerations, and so the whole-body dynamics of the MPC, did not conserve momentum in flight.
 *     Now the full block is solved with a branch-free Schur complement (3x3 closed-form inverses only, CppAD safe).
 *  2. pinocchio 3 crba() fills only the upper triangle of data.M across joints. computeBaseAcceleration() and
 *     computeJointTorques() read lower-triangle entries (the angular base block, M.bottomRows()), which were zero.
 *     data.M is now symmetrised before use.
 */

#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

// Pinnochio
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>

namespace ocs2::humanoid {

template <typename SCALAR_T>
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface) {
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  updateFramePlacements(q, model, data);
}
template void updateFramePlacements(const ad_vector_t& q, PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface);
template void updateFramePlacements(const vector_t& q, PinocchioInterfaceTpl<scalar_t>& pinocchioInterface);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, const pinocchio::ModelTpl<SCALAR_T>& model, pinocchio::DataTpl<SCALAR_T>& data) {
  pinocchio::forwardKinematics(model, data, q);
  updateFramePlacements(model, data);
}
template void updateFramePlacements(const ad_vector_t& q,
                                    const pinocchio::ModelTpl<ad_scalar_t>& model,
                                    pinocchio::DataTpl<ad_scalar_t>& data);
template void updateFramePlacements(const vector_t& q, const pinocchio::ModelTpl<scalar_t>& model, pinocchio::DataTpl<scalar_t>& data);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeContactPositions(const VECTOR_T<SCALAR_T>& q,
                                                         PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                         const MpcRobotModelBase<SCALAR_T>& mpcRobotModel) {
  updateFramePlacements<SCALAR_T>(q, pinocchioInterface);
  return getContactPositions<SCALAR_T>(pinocchioInterface, mpcRobotModel);
}
template std::vector<VECTOR3_T<ad_scalar_t>> computeContactPositions(const VECTOR_T<ad_scalar_t>& q,
                                                                     PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                                     const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel);
template std::vector<VECTOR3_T<scalar_t>> computeContactPositions(const VECTOR_T<scalar_t>& q,
                                                                  PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                                  const MpcRobotModelBase<scalar_t>& mpcRobotModel);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getContactPositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                     const MpcRobotModelBase<SCALAR_T>& mpcRobotModel) {
  assert(mpcRobotModel.modelSettings.contactNames.size() == N_CONTACTS);
  std::vector<VECTOR3_T<SCALAR_T>> footPositions;
  footPositions.reserve(N_CONTACTS);
  const auto& data = pinocchioInterface.getData();
  std::vector<pinocchio::FrameIndex> contactFrameIndices = getContactFrameIndices(pinocchioInterface, mpcRobotModel);

  for (size_t i = 0; i < N_CONTACTS; i++) {
    const VECTOR3_T<SCALAR_T>& footPosition = data.oMf[getContactFrameIndex(pinocchioInterface, mpcRobotModel, i)].translation();
    footPositions.emplace_back(footPosition);
  }
  return footPositions;
}
template std::vector<VECTOR3_T<ad_scalar_t>> getContactPositions(const PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel);
template std::vector<VECTOR3_T<scalar_t>> getContactPositions(const PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                              const MpcRobotModelBase<scalar_t>& mpcRobotModel);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeFramePositions(const VECTOR_T<SCALAR_T>& q,
                                                       PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                       std::vector<std::string> frameNames) {
  updateFramePlacements<SCALAR_T>(q, pinocchioInterface);
  return getFramePositions<SCALAR_T>(pinocchioInterface, frameNames);
}
template std::vector<VECTOR3_T<ad_scalar_t>> computeFramePositions(const VECTOR_T<ad_scalar_t>& q,
                                                                   PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                                   std::vector<std::string> frameNames);
template std::vector<VECTOR3_T<scalar_t>> computeFramePositions(const VECTOR_T<scalar_t>& q,
                                                                PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                                std::vector<std::string> frameNames);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getFramePositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                   std::vector<std::string> frameNames) {
  std::vector<VECTOR3_T<SCALAR_T>> positions;
  positions.reserve(frameNames.size());
  const auto& data = pinocchioInterface.getData();
  for (size_t i = 0; i < frameNames.size(); i++) {
    const pinocchio::FrameIndex frameIndex = pinocchioInterface.getModel().getFrameId(frameNames[i]);
    const VECTOR3_T<SCALAR_T>& position = data.oMf[frameIndex].translation();
    positions.emplace_back(position);
  }
  return positions;
}
template std::vector<VECTOR3_T<ad_scalar_t>> getFramePositions(const PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                               std::vector<std::string> frameNames);
template std::vector<VECTOR3_T<scalar_t>> getFramePositions(const PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                            std::vector<std::string> frameNames);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t computeGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                     const vector_t& q,
                                     size_t measuredMode) {
  updateFramePlacements<scalar_t>(q, pinocchioInterface);
  return getGroundHeightEstimate(pinocchioInterface, mpcRobotModel, measuredMode);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t getGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                 size_t measuredMode) {
  contact_flag_t measuredContactFlags = modeNumber2StanceLeg(measuredMode);

  std::vector<vector3_t> contactPositions = getContactPositions<scalar_t>(pinocchioInterface, mpcRobotModel);

  static scalar_t terrainHeight = 0.0;

  // Use right foot if in contact
  if (measuredContactFlags[0] && measuredContactFlags[1]) {
    vector3_t footPosition1 = contactPositions[0];
    vector3_t footPosition2 = contactPositions[1];
    terrainHeight = 0.5 * (footPosition1[2] + footPosition2[2]);
  } else if (measuredContactFlags[0]) {
    vector3_t footPosition = contactPositions[0];
    terrainHeight = footPosition[2];
  } else if (measuredContactFlags[1]) {
    vector3_t footPosition = contactPositions[1];
    terrainHeight = footPosition[2];
  }
  return terrainHeight;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
VECTOR6_T<SCALAR_T> computeBaseAcceleration(const MATRIX_T<SCALAR_T>& M,
                                            const VECTOR_T<SCALAR_T>& nle,
                                            const VECTOR_T<SCALAR_T>& qdd_joints,
                                            const VECTOR_T<SCALAR_T>& externalForcesInJointSpace) {
  // g1_backflip_mujoco fix: full 6x6 base block [A B; B' C] (A linear, C angular, B coupling), only the upper triangle
  // of M is trusted. Schur complement with closed-form 3x3 inverses: no pivoting, so no CppAD branching.
  using Matrix3 = Eigen::Matrix<SCALAR_T, 3, 3>;
  const auto symmetricFromUpper = [](const Matrix3& upper) {
    Matrix3 m = upper;
    m(1, 0) = upper(0, 1);
    m(2, 0) = upper(0, 2);
    m(2, 1) = upper(1, 2);
    return m;
  };
  const Matrix3 A = symmetricFromUpper(M.topLeftCorner(3, 3));
  const Matrix3 C = symmetricFromUpper(M.block(3, 3, 3, 3));
  const Matrix3 B = M.block(0, 3, 3, 3);
  auto M_bj = M.block(0, 6, 6, qdd_joints.size());

  const VECTOR6_T<SCALAR_T> r = -nle.head(6) - M_bj * qdd_joints + externalForcesInJointSpace.head(6);
  const Matrix3 Ainv = A.inverse();
  const Matrix3 Sinv = (C - B.transpose() * Ainv * B).inverse();

  VECTOR6_T<SCALAR_T> baseAccelerations;
  baseAccelerations.tail(3) = Sinv * (r.tail(3) - B.transpose() * (Ainv * r.head(3)));
  baseAccelerations.head(3) = Ainv * (r.head(3) - B * baseAccelerations.tail(3));

  return baseAccelerations;
}
template VECTOR6_T<scalar_t> computeBaseAcceleration(const MATRIX_T<scalar_t>& M,
                                                     const VECTOR_T<scalar_t>& nle,
                                                     const VECTOR_T<scalar_t>& qdd_joints,
                                                     const VECTOR_T<scalar_t>& externalForcesInJointSpace);
template VECTOR6_T<ad_scalar_t> computeBaseAcceleration(const MATRIX_T<ad_scalar_t>& M,
                                                        const VECTOR_T<ad_scalar_t>& nle,
                                                        const VECTOR_T<ad_scalar_t>& qdd_joints,
                                                        const VECTOR_T<ad_scalar_t>& externalForcesInJointSpace);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
VECTOR_T<SCALAR_T> computeJointTorques(const VECTOR_T<SCALAR_T>& q,
                                       const VECTOR_T<SCALAR_T>& qd,
                                       const VECTOR_T<SCALAR_T>& qdd_joints,
                                       const std::array<VECTOR6_T<SCALAR_T>, 2>& footWrenches,
                                       PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface) {
  const auto& model = pinocchioInterface.getModel();
  pinocchio::DataTpl<SCALAR_T>& data = pinocchioInterface.getData();

  pinocchio::crba(model, data, q);
  // g1_backflip_mujoco fix: crba fills only the upper triangle; M.bottomRows() below needs the lower one
  data.M.template triangularView<Eigen::StrictlyLower>() = data.M.transpose().template triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, q, qd);

  // Compute Jacobians for the foot frames
  MATRIX_T<SCALAR_T> J_foot_l = MATRIX_T<SCALAR_T>::Zero(6, qd.size());
  MATRIX_T<SCALAR_T> J_foot_r = MATRIX_T<SCALAR_T>::Zero(6, qd.size());

  ////////////////////////////////////////////////////////////////////////////

  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_l_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_l);
  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_r_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_r);

  // Project contact wrenches into the joint space

  VECTOR_T<SCALAR_T> externalForcesInJointSpace = J_foot_l.transpose() * footWrenches[0] + J_foot_r.transpose() * footWrenches[1];

  VECTOR6_T<SCALAR_T> baseAccelerations = computeBaseAcceleration(data.M, data.nle, qdd_joints, externalForcesInJointSpace);

  VECTOR_T<SCALAR_T> q_dd(qd.size());
  q_dd << baseAccelerations, qdd_joints;
  size_t n_joints = qdd_joints.size();

  VECTOR_T<SCALAR_T> jointTorques =
      data.M.bottomRows(n_joints) * q_dd + data.nle.tail(n_joints) - externalForcesInJointSpace.tail(n_joints);

  // return jointTorques;
  return jointTorques;
}
template VECTOR_T<scalar_t> computeJointTorques(const VECTOR_T<scalar_t>& q,
                                                const VECTOR_T<scalar_t>& qd,
                                                const VECTOR_T<scalar_t>& qdd_joints,
                                                const std::array<VECTOR6_T<scalar_t>, 2>& footWrenches,
                                                PinocchioInterfaceTpl<scalar_t>& pinocchioInterface);
template VECTOR_T<ad_scalar_t> computeJointTorques(const VECTOR_T<ad_scalar_t>& q,
                                                   const VECTOR_T<ad_scalar_t>& qd,
                                                   const VECTOR_T<ad_scalar_t>& qdd_joints,
                                                   const std::array<VECTOR6_T<ad_scalar_t>, 2>& footWrenches,
                                                   PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
VECTOR_T<SCALAR_T> computeJointTorquesRNEA(const VECTOR_T<SCALAR_T>& q,
                                           const VECTOR_T<SCALAR_T>& qd,
                                           const VECTOR_T<SCALAR_T>& qdd_joints,
                                           const std::array<VECTOR6_T<SCALAR_T>, 2>& footWrenches,
                                           PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface) {
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();

  pinocchio::container::aligned_vector<pinocchio::Force> fextDesired(model.njoints, pinocchio::Force::Zero());

  pinocchio::forwardKinematics(model, data, q, qd);
  pinocchio::updateFramePlacements(model, data);

  auto setExternalForce = [&](const std::string& frameName, size_t i) {
    const auto frameIndex = model.getFrameId(frameName);
    const auto jointIndex = model.frames[frameIndex].parentJoint;
    const VECTOR3_T<SCALAR_T> translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const MATRIX3_T<SCALAR_T> rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const VECTOR3_T<SCALAR_T> contactForce = rotationWorldFrameToJointFrame * footWrenches[i].head(3);
    const VECTOR3_T<SCALAR_T> contactTorque = rotationWorldFrameToJointFrame * footWrenches[i].tail(3);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce) + contactTorque;
  };

  setExternalForce("foot_l_contact", 0);
  setExternalForce("foot_r_contact", 1);

  pinocchio::crba(model, data, q);
  pinocchio::nonLinearEffects(model, data, q, qd);

  // Compute Jacobians for the foot frames
  MATRIX_T<SCALAR_T> J_foot_l = MATRIX_T<SCALAR_T>::Zero(6, qd.size());
  MATRIX_T<SCALAR_T> J_foot_r = MATRIX_T<SCALAR_T>::Zero(6, qd.size());

  ////////////////////////////////////////////////////////////////////////////

  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_l_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_l);
  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_r_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_r);

  // Project contact wrenches into the joint space

  VECTOR_T<SCALAR_T> externalForcesInJointSpace = J_foot_l.transpose() * footWrenches[0] + J_foot_r.transpose() * footWrenches[1];

  // Repalce q with external forces in joint space.

  VECTOR6_T<SCALAR_T> baseAccelerations = computeBaseAcceleration(data.M, data.nle, qdd_joints, externalForcesInJointSpace);

  VECTOR_T<SCALAR_T> q_dd(qd.size());
  q_dd << baseAccelerations, qdd_joints;

  vector_t torques = pinocchio::rnea(model, data, q, qd, q_dd, fextDesired);

  return torques.tail(qdd_joints.size());
}
template VECTOR_T<scalar_t> computeJointTorquesRNEA(const VECTOR_T<scalar_t>& q,
                                                    const VECTOR_T<scalar_t>& qd,
                                                    const VECTOR_T<scalar_t>& qdd_joints,
                                                    const std::array<VECTOR6_T<scalar_t>, 2>& footWrenches,
                                                    PinocchioInterfaceTpl<scalar_t>& pinocchioInterface);
// template VECTOR_T<ad_scalar_t> computeJointTorquesRNEA(const VECTOR_T<ad_scalar_t>& q,
//                                                        const VECTOR_T<ad_scalar_t>& qd,
//                                                        const VECTOR_T<ad_scalar_t>& qdd_joints,
//                                                        const std::array<VECTOR6_T<ad_scalar_t>, 2>& footWrenches,
//                                                        PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface);

}  // namespace ocs2::humanoid

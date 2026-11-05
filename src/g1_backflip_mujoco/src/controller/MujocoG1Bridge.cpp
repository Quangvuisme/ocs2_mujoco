#include "g1_backflip_mujoco/controller/MujocoG1Bridge.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace g1_backflip {

namespace {
/** ZYX Euler angles (yaw, pitch, roll) of a rotation matrix, R = Rz(yaw) Ry(pitch) Rx(roll). */
Eigen::Vector3d eulerZyx(const Eigen::Matrix3d& R) {
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  return {yaw, pitch, roll};
}
}  // namespace

Eigen::Matrix3d MujocoG1Bridge::pelvisToFlipBase() { return Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }

std::vector<MujocoG1Bridge::JointMap> MujocoG1Bridge::mapJoints(const mjModel* model, const std::vector<std::string>& names) {
  std::vector<JointMap> maps;
  for (const auto& name : names) {
    const int jointId = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (jointId < 0) {
      throw std::runtime_error("[MujocoG1Bridge] joint '" + name + "' not found in the MuJoCo model");
    }
    int actuator = -1;
    for (int a = 0; a < model->nu; ++a) {
      if (model->actuator_trntype[a] == mjTRN_JOINT && model->actuator_trnid[2 * a] == jointId) {
        actuator = a;
        break;
      }
    }
    if (actuator < 0) {
      throw std::runtime_error("[MujocoG1Bridge] no actuator drives joint '" + name + "'");
    }
    maps.push_back({model->jnt_qposadr[jointId], model->jnt_dofadr[jointId], actuator});
  }
  return maps;
}

MujocoG1Bridge::MujocoG1Bridge(const mjModel* model, const std::vector<std::string>& mpcJointNames,
                               const std::vector<std::string>& extraJointNames) {
  int freeJoint = -1;
  for (int j = 0; j < model->njnt; ++j) {
    if (model->jnt_type[j] == mjJNT_FREE) {
      freeJoint = j;
      break;
    }
  }
  if (freeJoint < 0) {
    throw std::runtime_error("[MujocoG1Bridge] no free joint in the MuJoCo model");
  }
  baseQposAdr_ = model->jnt_qposadr[freeJoint];
  baseDofAdr_ = model->jnt_dofadr[freeJoint];
  mpcJoints_ = mapJoints(model, mpcJointNames);
  extraJoints_ = mapJoints(model, extraJointNames);
}

Eigen::Vector3d MujocoG1Bridge::basePosition(const mjData* d) const {
  return {d->qpos[baseQposAdr_], d->qpos[baseQposAdr_ + 1], d->qpos[baseQposAdr_ + 2]};
}

ocs2::vector_t MujocoG1Bridge::extraJointPositions(const mjData* d) const {
  ocs2::vector_t q(extraJoints_.size());
  for (size_t i = 0; i < extraJoints_.size(); ++i) q(i) = d->qpos[extraJoints_[i].qposAdr];
  return q;
}

ocs2::vector_t MujocoG1Bridge::extraJointVelocities(const mjData* d) const {
  ocs2::vector_t v(extraJoints_.size());
  for (size_t i = 0; i < extraJoints_.size(); ++i) v(i) = d->qvel[extraJoints_[i].dofAdr];
  return v;
}

ocs2::vector_t MujocoG1Bridge::mpcState(const mjData* d) {
  const size_t nj = mpcJoints_.size();
  ocs2::vector_t x(12 + 2 * nj);

  const double* q = d->qpos + baseQposAdr_ + 3;
  const Eigen::Matrix3d Rpelvis = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
  const Eigen::Vector3d zyx = eulerZyx(Rpelvis * pelvisToFlipBase());
  if (!eulerInitialized_) {
    euler_ = zyx;
    eulerInitialized_ = true;
  } else {
    euler_(0) += std::remainder(zyx(0) - euler_(0), 2.0 * M_PI);
    euler_(1) = zyx(1);
    euler_(2) += std::remainder(zyx(2) - euler_(2), 2.0 * M_PI);
  }

  // MuJoCo free joint: linear velocity in the world frame, angular velocity in the pelvis frame
  const Eigen::Vector3d omegaWorld = Rpelvis * Eigen::Vector3d(d->qvel[baseDofAdr_ + 3], d->qvel[baseDofAdr_ + 4], d->qvel[baseDofAdr_ + 5]);
  x.segment<3>(0) = basePosition(d);  // flip_base and pelvis share the origin
  x.segment<3>(3) = euler_;
  for (size_t i = 0; i < nj; ++i) x(6 + i) = d->qpos[mpcJoints_[i].qposAdr];
  x.segment<3>(6 + nj) << d->qvel[baseDofAdr_], d->qvel[baseDofAdr_ + 1], d->qvel[baseDofAdr_ + 2];
  x.segment<3>(9 + nj) = ocs2::getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<double>(euler_, omegaWorld);
  for (size_t i = 0; i < nj; ++i) x(12 + nj + i) = d->qvel[mpcJoints_[i].dofAdr];
  return x;
}

void MujocoG1Bridge::setPose(mjData* d, const ocs2::vector_t& x) {
  const size_t nj = mpcJoints_.size();
  const Eigen::Vector3d e = x.segment<3>(3);
  const Eigen::Matrix3d Rbase = (Eigen::AngleAxisd(e(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(e(1), Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(e(2), Eigen::Vector3d::UnitX()))
                                    .toRotationMatrix();
  const Eigen::Quaterniond qPelvis(Rbase * pelvisToFlipBase().transpose());
  for (int k = 0; k < 3; ++k) d->qpos[baseQposAdr_ + k] = x(k);
  d->qpos[baseQposAdr_ + 3] = qPelvis.w();
  d->qpos[baseQposAdr_ + 4] = qPelvis.x();
  d->qpos[baseQposAdr_ + 5] = qPelvis.y();
  d->qpos[baseQposAdr_ + 6] = qPelvis.z();
  for (size_t i = 0; i < nj; ++i) d->qpos[mpcJoints_[i].qposAdr] = x(6 + i);
  for (const auto& j : extraJoints_) d->qpos[j.qposAdr] = 0.0;
  for (int k = 0; k < 6; ++k) d->qvel[baseDofAdr_ + k] = 0.0;
  for (const auto& j : mpcJoints_) d->qvel[j.dofAdr] = 0.0;
  for (const auto& j : extraJoints_) d->qvel[j.dofAdr] = 0.0;
  eulerInitialized_ = false;
}

void MujocoG1Bridge::setTorques(const mjModel* m, mjData* d, const ocs2::vector_t& tauMpc, const ocs2::vector_t& tauExtra) const {
  auto write = [&](const JointMap& j, double tau) {
    double u = std::isfinite(tau) ? tau : 0.0;
    if (m->actuator_ctrllimited[j.actuator]) {
      u = std::clamp(u, m->actuator_ctrlrange[2 * j.actuator], m->actuator_ctrlrange[2 * j.actuator + 1]);
    }
    d->ctrl[j.actuator] = u;
  };
  for (size_t i = 0; i < mpcJoints_.size(); ++i) write(mpcJoints_[i], tauMpc(i));
  for (size_t i = 0; i < extraJoints_.size(); ++i) write(extraJoints_[i], tauExtra(i));
}

}  // namespace g1_backflip

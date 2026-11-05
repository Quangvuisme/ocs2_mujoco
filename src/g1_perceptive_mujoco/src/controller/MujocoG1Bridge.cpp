#include "g1_perceptive_mujoco/controller/MujocoG1Bridge.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace g1_perceptive {

namespace {
Eigen::Vector3d quaternionToEulerZyx(double qw, double qx, double qy, double qz) {
  const double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
  const double sinp = 2.0 * (qw * qy - qz * qx);
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  const double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  return {yaw, pitch, roll};
}
}  // namespace

MujocoG1Bridge::MujocoG1Bridge(const mjModel* model, const std::vector<std::string>& jointNames) {
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

  for (const auto& name : jointNames) {
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
    jointQposAdr_.push_back(model->jnt_qposadr[jointId]);
    jointDofAdr_.push_back(model->jnt_dofadr[jointId]);
    actuatorId_.push_back(actuator);
  }
}

Eigen::Vector3d MujocoG1Bridge::basePosition(const mjData* d) const {
  return {d->qpos[baseQposAdr_], d->qpos[baseQposAdr_ + 1], d->qpos[baseQposAdr_ + 2]};
}

Eigen::Vector3d MujocoG1Bridge::baseEulerZyx(const mjData* d) const {
  const double* q = d->qpos + baseQposAdr_ + 3;
  return quaternionToEulerZyx(q[0], q[1], q[2], q[3]);
}

ocs2::vector_t MujocoG1Bridge::jointPositions(const mjData* d) const {
  ocs2::vector_t q(numJoints());
  for (size_t i = 0; i < numJoints(); ++i) {
    q(i) = d->qpos[jointQposAdr_[i]];
  }
  return q;
}

ocs2::vector_t MujocoG1Bridge::jointVelocities(const mjData* d) const {
  ocs2::vector_t v(numJoints());
  for (size_t i = 0; i < numJoints(); ++i) {
    v(i) = d->qvel[jointDofAdr_[i]];
  }
  return v;
}

ocs2::vector_t MujocoG1Bridge::rbdState(const mjData* d) {
  const size_t nj = numJoints();
  ocs2::vector_t rbd = ocs2::vector_t::Zero(2 * (6 + nj));

  Eigen::Vector3d zyx = baseEulerZyx(d);
  if (!yawInitialized_) {
    continuousYaw_ = zyx(0);
    yawInitialized_ = true;
  } else {
    double diff = zyx(0) - continuousYaw_;
    diff = std::remainder(diff, 2.0 * M_PI);  // in [-pi, pi]
    continuousYaw_ += diff;
  }
  zyx(0) = continuousYaw_;

  rbd.segment<3>(0) = zyx;
  rbd.segment<3>(3) = basePosition(d);
  rbd.segment(6, nj) = jointPositions(d);

  // MuJoCo free joint: linear velocity in world frame, angular velocity in body frame
  const double* q = d->qpos + baseQposAdr_ + 3;
  const Eigen::Matrix3d R = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).toRotationMatrix();
  const Eigen::Vector3d omegaBody(d->qvel[baseDofAdr_ + 3], d->qvel[baseDofAdr_ + 4], d->qvel[baseDofAdr_ + 5]);
  rbd.segment<3>(6 + nj) = R * omegaBody;
  rbd.segment<3>(9 + nj) << d->qvel[baseDofAdr_], d->qvel[baseDofAdr_ + 1], d->qvel[baseDofAdr_ + 2];
  rbd.segment(12 + nj, nj) = jointVelocities(d);
  return rbd;
}

void MujocoG1Bridge::setPose(mjData* d, const Eigen::Vector3d& basePosition, double yaw,
                                 const ocs2::vector_t& jointPositions) const {
  d->qpos[baseQposAdr_] = basePosition.x();
  d->qpos[baseQposAdr_ + 1] = basePosition.y();
  d->qpos[baseQposAdr_ + 2] = basePosition.z();
  d->qpos[baseQposAdr_ + 3] = std::cos(0.5 * yaw);
  d->qpos[baseQposAdr_ + 4] = 0.0;
  d->qpos[baseQposAdr_ + 5] = 0.0;
  d->qpos[baseQposAdr_ + 6] = std::sin(0.5 * yaw);
  for (size_t i = 0; i < numJoints(); ++i) {
    d->qpos[jointQposAdr_[i]] = jointPositions(i);
  }
}

void MujocoG1Bridge::setJointTorques(const mjModel* m, mjData* d, const ocs2::vector_t& tau) const {
  for (size_t i = 0; i < numJoints(); ++i) {
    const int a = actuatorId_[i];
    double u = std::isfinite(tau(i)) ? tau(i) : 0.0;
    if (m->actuator_ctrllimited[a]) {
      u = std::clamp(u, m->actuator_ctrlrange[2 * a], m->actuator_ctrlrange[2 * a + 1]);
    }
    d->ctrl[a] = u;
  }
}

}  // namespace g1_perceptive

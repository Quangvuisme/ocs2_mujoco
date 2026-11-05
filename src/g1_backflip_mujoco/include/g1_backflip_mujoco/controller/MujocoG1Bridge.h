#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include <ocs2_core/Types.h>

namespace g1_backflip {

/**
 * Maps between the MuJoCo model (free joint on the pelvis, 29 joints) and the whole-body MPC model of wb_humanoid_mpc
 * (free-flyer on the URDF root "flip_base", the pelvis turned +90 deg about z; 23 MPC joints). Joints are matched by
 * name; the joints that are not in the MPC (wrists) are "extra" joints.
 *
 * MPC state: [position, flip_base ZYX Euler, q_mpc, linear velocity (world), Euler rates, dq_mpc]. The yaw and the
 * roll (robot pitch, 0 -> -2 pi in the backflip) are unwrapped, i.e. continuous.
 */
class MujocoG1Bridge {
 public:
  MujocoG1Bridge(const mjModel* model, const std::vector<std::string>& mpcJointNames, const std::vector<std::string>& extraJointNames);

  size_t numMpcJoints() const { return mpcJoints_.size(); }
  size_t numExtraJoints() const { return extraJoints_.size(); }

  /** Measured MPC state (perfect state estimation). Call every step (Euler unwrapping). */
  ocs2::vector_t mpcState(const mjData* data);

  ocs2::vector_t extraJointPositions(const mjData* data) const;
  ocs2::vector_t extraJointVelocities(const mjData* data) const;

  /** Sets the pose from an MPC state (velocities zero, extra joints zero). */
  void setPose(mjData* data, const ocs2::vector_t& mpcState);

  /** Writes torques of the MPC joints and of the extra joints (clamped to ctrlrange). */
  void setTorques(const mjModel* model, mjData* data, const ocs2::vector_t& tauMpc, const ocs2::vector_t& tauExtra) const;

  Eigen::Vector3d basePosition(const mjData* data) const;
  /** Unwrapped flip_base ZYX Euler angles of the last mpcState() call: (yaw, robot roll, robot pitch). */
  const Eigen::Vector3d& lastEulerZyx() const { return euler_; }
  void resetUnwrapping() { eulerInitialized_ = false; }

  /** Rotation pelvis -> flip_base (+90 deg about z). */
  static Eigen::Matrix3d pelvisToFlipBase();

 private:
  struct JointMap {
    int qposAdr, dofAdr, actuator;
  };
  static std::vector<JointMap> mapJoints(const mjModel* model, const std::vector<std::string>& names);

  int baseQposAdr_ = 0;
  int baseDofAdr_ = 0;
  std::vector<JointMap> mpcJoints_, extraJoints_;
  Eigen::Vector3d euler_ = Eigen::Vector3d::Zero();
  bool eulerInitialized_ = false;
};

}  // namespace g1_backflip

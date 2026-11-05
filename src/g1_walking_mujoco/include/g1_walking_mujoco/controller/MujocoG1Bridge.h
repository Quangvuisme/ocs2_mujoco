#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include <ocs2_core/Types.h>

namespace g1_walking {

/**
 * Maps between the MuJoCo model and the OCS2/Pinocchio model by joint name, so the joint order
 * of either model does not matter.
 *
 * RBD state (OCS2 CentroidalModelRbdConversions layout):
 *   [yaw pitch roll, x y z, q_joints, omega_world, v_world, dq_joints]
 */
class MujocoG1Bridge {
 public:
  /** @param jointNames: actuated joint names in OCS2/Pinocchio order. */
  MujocoG1Bridge(const mjModel* model, const std::vector<std::string>& jointNames);

  size_t numJoints() const { return jointQposAdr_.size(); }

  /** Measured RBD state (perfect state estimation from the simulator). Keeps yaw continuous. */
  ocs2::vector_t rbdState(const mjData* data);

  /** Joint positions / velocities in OCS2 order. */
  ocs2::vector_t jointPositions(const mjData* data) const;
  ocs2::vector_t jointVelocities(const mjData* data) const;

  /** Sets the base pose (position, ZYX Euler) and OCS2-ordered joint positions (velocities untouched). */
  void setPose(mjData* data, const Eigen::Vector3d& basePosition, double yaw, const ocs2::vector_t& jointPositions) const;

  /** Writes OCS2-ordered joint torques to the actuators (clamped to ctrlrange). */
  void setJointTorques(const mjModel* model, mjData* data, const ocs2::vector_t& tau) const;

  /** Base position / ZYX Euler angles of the floating base. */
  Eigen::Vector3d basePosition(const mjData* data) const;
  Eigen::Vector3d baseEulerZyx(const mjData* data) const;

  /** Resets the yaw unwrapping (after a simulation reset). */
  void resetYaw() { yawInitialized_ = false; }

 private:
  int baseQposAdr_ = 0;
  int baseDofAdr_ = 0;
  std::vector<int> jointQposAdr_;
  std::vector<int> jointDofAdr_;
  std::vector<int> actuatorId_;

  double continuousYaw_ = 0.0;
  bool yawInitialized_ = false;
};

}  // namespace g1_walking

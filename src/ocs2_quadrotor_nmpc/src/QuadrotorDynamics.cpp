#include "ocs2_quadrotor_nmpc/QuadrotorDynamics.h"

#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>

#include <cmath>

namespace ocs2_cartpole_quadrotor {

ocs2::vector_t QuadrotorDynamics::computeFlowMap(ocs2::scalar_t time,
                                                  const ocs2::vector_t& state,
                                                  const ocs2::vector_t& input,
                                                  const ocs2::PreComputation&) {
  // Angular velocities to Euler angle Derivatives transformation
  Eigen::Matrix<ocs2::scalar_t, 3, 1> eulerAngle = state.segment<3>(3);
  Eigen::Matrix<ocs2::scalar_t, 3, 3> T =
      ocs2::getMappingFromLocalAngularVelocityToEulerAnglesXyzDerivative<ocs2::scalar_t>(eulerAngle);

  // Euler angles xyz
  ocs2::scalar_t qph = state(3);   // roll
  ocs2::scalar_t qth = state(4);   // pitch
  ocs2::scalar_t qps = state(5);   // yaw

  // Position derivatives (velocities)
  ocs2::scalar_t dqxQ = state(6);  // vx
  ocs2::scalar_t dqyQ = state(7);  // vy
  ocs2::scalar_t dqzQ = state(8);  // vz

  // Angular velocity xyz
  ocs2::scalar_t dqph = state(9);   // wx
  ocs2::scalar_t dqth = state(10);  // wy
  ocs2::scalar_t dqps = state(11);  // wz

  // Euler angle derivatives
  Eigen::Matrix<ocs2::scalar_t, 3, 1> eulerAngleDerivatives = T * state.segment<3>(9);

  // Applied force and moments
  ocs2::scalar_t Fz = input(0);  // Thrust
  ocs2::scalar_t Mx = input(1);  // Roll moment
  ocs2::scalar_t My = input(2);  // Pitch moment
  ocs2::scalar_t Mz = input(3);  // Yaw moment

  ocs2::scalar_t t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13;

  t2 = 1.0 / param_.quadrotorMass_;
  t3 = cos(qth);
  t4 = sin(qth);
  t5 = 1.0 / param_.Thxxyy_;
  t6 = cos(qps);
  t7 = sin(qps);
  t8 = dqph * dqph;
  t9 = qth * 2.0;
  t10 = sin(t9);
  t11 = 1.0 / t3;
  t12 = param_.Thzz_ * param_.Thzz_;
  t13 = t3 * t3;

  ocs2::vector_t stateDerivative(STATE_DIM);
  
  // Position derivatives
  stateDerivative(0) = dqxQ;
  stateDerivative(1) = dqyQ;
  stateDerivative(2) = dqzQ;
  
  // Euler angle derivatives
  stateDerivative(3) = eulerAngleDerivatives(0);
  stateDerivative(4) = eulerAngleDerivatives(1);
  stateDerivative(5) = eulerAngleDerivatives(2);
  
  // Linear acceleration (from thrust and gravity)
  stateDerivative(6) = Fz * t2 * t4;
  stateDerivative(7) = -Fz * t2 * t3 * sin(qph);
  stateDerivative(8) = t2 * (param_.quadrotorMass_ * param_.gravity_ - Fz * t3 * cos(qph)) * (-1.0);
  
  // Angular acceleration
  stateDerivative(9) = -t5 * t11 *
                       (-Mx * t6 + My * t7 + param_.Thzz_ * dqps * dqth -
                        param_.Thxxyy_ * dqph * dqth * t4 * (2.0) +
                        param_.Thzz_ * dqph * dqth * t4);
  stateDerivative(10) = t5 *
                        (Mx * t7 + My * t6 - param_.Thxxyy_ * t8 * t10 * (1.0 / 2.0) +
                         param_.Thzz_ * t8 * t10 * (1.0 / 2.0) + param_.Thzz_ * dqph * dqps * t3);
  stateDerivative(11) = (t5 * t11 *
                         (Mz * param_.Thxxyy_ * t3 + dqph * dqth * t12 - dqph * dqth * t12 * t13 +
                          dqps * dqth * t4 * t12 -
                          param_.Thxxyy_ * param_.Thzz_ * dqph * dqth * (2.0) -
                          Mx * param_.Thzz_ * t4 * t6 + My * param_.Thzz_ * t4 * t7 +
                          param_.Thxxyy_ * param_.Thzz_ * dqph * dqth * t13)) /
                        param_.Thzz_;
                        
  return stateDerivative;
}

ocs2::VectorFunctionLinearApproximation QuadrotorDynamics::linearApproximation(
    ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u,
    const ocs2::PreComputation& preComp) {
  ocs2::VectorFunctionLinearApproximation dynamics;
  dynamics.f = computeFlowMap(t, x, u, preComp);

  // Jacobian of angular velocity mapping
  Eigen::Matrix<ocs2::scalar_t, 3, 1> eulerAngle = x.segment<3>(3);
  Eigen::Matrix<ocs2::scalar_t, 3, 1> angularVelocity = x.segment<3>(9);
  jacobianOfAngularVelocityMapping_ =
      ocs2::JacobianOfAngularVelocityMapping(eulerAngle, angularVelocity).transpose();

  // Euler angles xyz
  ocs2::scalar_t qph = x(3);
  ocs2::scalar_t qth = x(4);
  ocs2::scalar_t qps = x(5);

  // Angular velocities
  ocs2::scalar_t dqph = x(9);
  ocs2::scalar_t dqth = x(10);
  ocs2::scalar_t dqps = x(11);

  // Applied force and moments
  ocs2::scalar_t Fz = u(0);
  ocs2::scalar_t Mx = u(1);
  ocs2::scalar_t My = u(2);
  ocs2::scalar_t Mz = u(3);

  {  // Derivative with respect to state
    ocs2::scalar_t t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16,
        t17, t18, t19, t20, t26, t21, t22, t27, t23, t24, t25;

    t2 = 1.0 / param_.quadrotorMass_;
    t3 = cos(qth);
    t4 = sin(qph);
    t5 = cos(qph);
    t6 = sin(qth);
    t7 = 1.0 / param_.Thxxyy_;
    t8 = cos(qps);
    t9 = sin(qps);
    t10 = 1.0 / t3;
    t11 = param_.Thxxyy_ * 2.0;
    t12 = param_.Thzz_ - t11;
    t13 = qth * 2.0;
    t14 = cos(t13);
    t15 = My * t9;
    t16 = sin(t13);
    t17 = 1.0 / (t3 * t3);
    t18 = qth * 3.0;
    t19 = sin(t18);
    t20 = My * t8;
    t21 = Mx * t9;
    t22 = t20 + t21;
    t23 = t3 * t3;
    t24 = t6 * t6;
    t25 = param_.Thzz_ * dqps * t6;

    ocs2::matrix_t& A = dynamics.dfdx;
    A.setZero(STATE_DIM, STATE_DIM);
    A.block<3, 3>(0, 6).setIdentity();
    A.block<3, 3>(3, 3) = jacobianOfAngularVelocityMapping_.block<3, 3>(0, 0);
    A.block<3, 3>(3, 9) = jacobianOfAngularVelocityMapping_.block<3, 3>(0, 3);

    A(6, 4) = Fz * t2 * t3;
    A(7, 3) = -Fz * t2 * t3 * t5;
    A(7, 4) = Fz * t2 * t4 * t6;
    A(8, 3) = -Fz * t2 * t3 * t4;
    A(8, 4) = -Fz * t2 * t5 * t6;
    A(9, 4) = -t6 * t7 * t17 *
                  (t15 - Mx * t8 + param_.Thzz_ * dqps * dqth -
                   param_.Thxxyy_ * dqph * dqth * t6 * 2.0 +
                   param_.Thzz_ * dqph * dqth * t6) -
              dqph * dqth * t7 * t12;
    A(9, 5) = -t7 * t10 * t22;
    A(9, 9) = -dqth * t6 * t7 * t10 * t12;
    A(9, 10) = -t7 * t10 *
               (param_.Thzz_ * dqps - param_.Thxxyy_ * dqph * t6 * 2.0 +
                param_.Thzz_ * dqph * t6);
    A(9, 11) = -param_.Thzz_ * dqth * t7 * t10;
    A(10, 4) = -dqph * t7 *
               (t25 + param_.Thxxyy_ * dqph * t14 - param_.Thzz_ * dqph * t14);
    A(10, 5) = -t7 * (t15 - Mx * t8);
    A(10, 9) = t7 * (-param_.Thxxyy_ * dqph * t16 + param_.Thzz_ * dqps * t3 +
                     param_.Thzz_ * dqph * t16);
    A(10, 11) = param_.Thzz_ * dqph * t3 * t7;
    A(11, 4) =
        t7 * t17 *
        (Mx * t8 * -4.0 + My * t9 * 4.0 + param_.Thzz_ * dqps * dqth * 4.0 -
         param_.Thxxyy_ * dqph * dqth * t6 * 9.0 -
         param_.Thxxyy_ * dqph * dqth * t19 +
         param_.Thzz_ * dqph * dqth * t6 * 5.0 +
         param_.Thzz_ * dqph * dqth * t19) *
        (1.0 / 4.0);
    A(11, 5) = t6 * t7 * t10 * t22;
    A(11, 9) = dqth * t7 * t10 *
               (param_.Thzz_ - t11 + param_.Thxxyy_ * t23 - param_.Thzz_ * t23);
    A(11, 10) = t7 * t10 *
                (t25 - param_.Thxxyy_ * dqph - param_.Thxxyy_ * dqph * t24 +
                 param_.Thzz_ * dqph * t24);
    A(11, 11) = param_.Thzz_ * dqth * t6 * t7 * t10;
  }

  {  // Derivative with respect to input
    ocs2::scalar_t t2, t3, t4, t5, t6, t7, t8;

    t2 = 1.0 / param_.quadrotorMass_;
    t3 = cos(qth);
    t4 = 1.0 / param_.Thxxyy_;
    t5 = 1.0 / t3;
    t6 = sin(qps);
    t7 = cos(qps);
    t8 = sin(qth);

    ocs2::matrix_t& B = dynamics.dfdu;
    B.setZero(STATE_DIM, INPUT_DIM);
    B(6, 0) = t2 * t8;
    B(7, 0) = -t2 * t3 * sin(qph);
    B(8, 0) = t2 * t3 * cos(qph);
    B(9, 1) = t4 * t5 * t7;
    B(9, 2) = -t4 * t5 * t6;
    B(10, 1) = t4 * t6;
    B(10, 2) = t4 * t7;
    B(11, 1) = -t4 * t5 * t7 * t8;
    B(11, 2) = t4 * t5 * t6 * t8;
    B(11, 3) = 1.0 / param_.Thzz_;
  }
  
  return dynamics;
}

}  // namespace ocs2_cartpole_quadrotor



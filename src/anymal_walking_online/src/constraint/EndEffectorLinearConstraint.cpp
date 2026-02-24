#include "anymal_walking_online/constraint/EndEffectorLinearConstraint.h"


namespace anymal_walking_online {
    EndEffectorLinearConstraint::EndEffectorLinearConstraint(
        const ocs2::EndEffectorKinematics<ocs2::scalar_t> &endEffectorKinematics,
        size_t numConstraints, Config config)
        : StateInputConstraint(ocs2::ConstraintOrder::Linear),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          numConstraints_(numConstraints),
          config_(std::move(config)) {
        if (endEffectorKinematicsPtr_->getIds().size() != 1) {
            throw std::runtime_error("[EndEffectorLinearConstraint] this class only accepts a single end-effector!");
        }
    }


    EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorLinearConstraint &rhs)
        : StateInputConstraint(rhs),
          endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
          numConstraints_(rhs.numConstraints_),
          config_(rhs.config_) {
    }


    void EndEffectorLinearConstraint::configure(Config &&config) {
        assert(config.b.rows() == numConstraints_);
        assert(config.Ax.size() > 0 || config.Av.size() > 0);
        assert((config.Ax.size() > 0 && config.Ax.rows() == numConstraints_) || config.Ax.size() == 0);
        assert((config.Ax.size() > 0 && config.Ax.cols() == 3) || config.Ax.size() == 0);
        assert((config.Av.size() > 0 && config.Av.rows() == numConstraints_) || config.Av.size() == 0);
        assert((config.Av.size() > 0 && config.Av.cols() == 3) || config.Av.size() == 0);
        config_ = std::move(config);
    }


    ocs2::vector_t EndEffectorLinearConstraint::getValue(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                                                   const ocs2::PreComputation &preComp) const {
        ocs2::vector_t f = config_.b;
        if (config_.Ax.size() > 0) {
            f.noalias() += config_.Ax * endEffectorKinematicsPtr_->getPosition(state).front();
        }
        if (config_.Av.size() > 0) {
            f.noalias() += config_.Av * endEffectorKinematicsPtr_->getVelocity(state, input).front();
        }
        return f;
    }


    ocs2::VectorFunctionLinearApproximation EndEffectorLinearConstraint::getLinearApproximation(
        ocs2::scalar_t time, const ocs2::vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const {
       ocs2:: VectorFunctionLinearApproximation linearApproximation =
                ocs2::VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());

        linearApproximation.f = config_.b;

        if (config_.Ax.size() > 0) {
            const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
            linearApproximation.f.noalias() += config_.Ax * positionApprox.f;
            linearApproximation.dfdx.noalias() += config_.Ax * positionApprox.dfdx;
        }

        if (config_.Av.size() > 0) {
            const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
            linearApproximation.f.noalias() += config_.Av * velocityApprox.f;
            linearApproximation.dfdx.noalias() += config_.Av * velocityApprox.dfdx;
            linearApproximation.dfdu.noalias() += config_.Av * velocityApprox.dfdu;
        }

        return linearApproximation;
    }
} // namespace anymal_walking_online

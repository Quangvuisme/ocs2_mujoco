#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>


namespace anymal_walking_online {
    /**
     * Defines a linear constraint on an end-effector position (xee) and linear velocity (vee).
     * g(xee, vee) = Ax * xee + Av * vee + b
     * - For defining constraint of type g(xee), set Av to matrix_t(0, 0)
     * - For defining constraint of type g(vee), set Ax to matrix_t(0, 0)
     */
    class EndEffectorLinearConstraint final : public ocs2::StateInputConstraint {
    public:
        /**
         * Coefficients of the linear constraints of the form:
         * g(xee, vee) = Ax * xee + Av * vee + b
         */
        struct Config {
            ocs2::vector_t b;
            ocs2::matrix_t Ax;
            ocs2::matrix_t Av;
        };

        /**
         * Constructor
         * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
         * @param [in] numConstraints: The number of constraints {1, 2, 3}
         * @param [in] config: The constraint coefficients, g(xee, vee) = Ax * xee + Av * vee + b
         */
        EndEffectorLinearConstraint(const ocs2::EndEffectorKinematics<ocs2::scalar_t> &endEffectorKinematics, size_t numConstraints,
                                    Config config = Config());

        ~EndEffectorLinearConstraint() override = default;

        EndEffectorLinearConstraint *clone() const override { return new EndEffectorLinearConstraint(*this); }

        /** Sets a new constraint coefficients. */
        void configure(Config &&config);

        /** Sets a new constraint coefficients. */
        void configure(const Config &config) { this->configure(Config(config)); }

        /** Gets the underlying end-effector kinematics interface. */
        ocs2::EndEffectorKinematics<ocs2::scalar_t> &getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

        size_t getNumConstraints(ocs2::scalar_t time) const override { return numConstraints_; }

        ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                          const ocs2::PreComputation &preComp) const override;

        ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t &state,
                                                                 const ocs2::vector_t &input,
                                                                 const ocs2::PreComputation &preComp) const override;

    private:
        EndEffectorLinearConstraint(const EndEffectorLinearConstraint &rhs);

        std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t> > endEffectorKinematicsPtr_;
        const size_t numConstraints_;
        Config config_;
    };
} // namespace anymal_walking_online

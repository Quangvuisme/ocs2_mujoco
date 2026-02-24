#include <quadruped_nmpc/constraint/ZeroForceConstraint.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>


namespace quadruped_nmpc {
    ZeroForceConstraint::ZeroForceConstraint(const SwitchedModelReferenceManager &referenceManager,
                                             size_t contactPointIndex,
                                             ocs2::CentroidalModelInfo info)
        : StateInputConstraint(ocs2::ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          contactPointIndex_(contactPointIndex),
          info_(std::move(info)) {
    }


    bool ZeroForceConstraint::isActive(ocs2::scalar_t time) const {
        return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
    }


    ocs2::vector_t ZeroForceConstraint::getValue(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
                                           const ocs2::PreComputation &preComp) const {
        return ocs2::centroidal_model::getContactForces(input, contactPointIndex_, info_);
    }


    ocs2::VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(
        ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const {
        ocs2::VectorFunctionLinearApproximation approx;
        approx.f = getValue(time, state, input, preComp);
        approx.dfdx = ocs2::matrix_t::Zero(3, state.size());
        approx.dfdu = ocs2::matrix_t::Zero(3, input.size());
        approx.dfdu.middleCols<3>(3 * contactPointIndex_).diagonal() = ocs2::vector_t::Ones(3);
        return approx;
    }
} // namespace quadruped_nmpc

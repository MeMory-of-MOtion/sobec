///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/utils/exception.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "sobec/crocomplements/residual-dcm-position.hpp"

namespace sobec {
using namespace crocoddyl;

template <typename Scalar>
ResidualModelDCMPositionTpl<Scalar>::ResidualModelDCMPositionTpl(boost::shared_ptr<StateMultibody> state,
                                                                 const Vector3s& cref, const double alpha,
                                                                 const std::size_t nu)
    : Base(state, 3, nu, true, true, false), cref_(cref), alpha_(alpha), pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelDCMPositionTpl<Scalar>::ResidualModelDCMPositionTpl(boost::shared_ptr<StateMultibody> state,
                                                                 const Vector3s& cref, const double alpha)
    : Base(state, 3, true, true, false), cref_(cref), alpha_(alpha), pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelDCMPositionTpl<Scalar>::~ResidualModelDCMPositionTpl() {}

template <typename Scalar>
void ResidualModelDCMPositionTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                               const Eigen::Ref<const VectorXs>& x,
                                               const Eigen::Ref<const VectorXs>&) {
  // Compute the residual residual give the reference DCMPosition position
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());

  pinocchio::centerOfMass(pin_model_, *d->pinocchio, q, v);
  data->r = d->pinocchio->com[0] + alpha_ * d->pinocchio->vcom[0] - cref_;
}

template <typename Scalar>
void ResidualModelDCMPositionTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                   const Eigen::Ref<const VectorXs>&,
                                                   const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());

  // Compute the derivatives of the frame placement
  const std::size_t nv = state_->get_nv();
  pinocchio::getCenterOfMassVelocityDerivatives(pin_model_, *d->pinocchio, d->dvcom_dq);
  data->Rx.leftCols(nv) = d->pinocchio->Jcom + alpha_ * d->dvcom_dq; 
  data->Rx.rightCols(nv) = alpha_ * d->pinocchio->Jcom;
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelDCMPositionTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void ResidualModelDCMPositionTpl<Scalar>::print(std::ostream& os) const {
  const Eigen::IOFormat fmt(2, Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]");
  os << "ResidualModelDCMPosition {cref=" << cref_.transpose().format(fmt) << "}";
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector3s& ResidualModelDCMPositionTpl<Scalar>::get_reference() const {
  return cref_;
}

template <typename Scalar>
void ResidualModelDCMPositionTpl<Scalar>::set_reference(const Vector3s& cref) {
  cref_ = cref;
}

}  // namespace sobec

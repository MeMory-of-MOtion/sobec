///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/residuals/com-velocity.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"

namespace crocoddyl {

template <typename Scalar>
ResidualModelCoMVelocityTpl<Scalar>::ResidualModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state,
                                                                 const Vector3s& vref, const std::size_t nu)
    : Base(state, 3, nu, true, true, false), pin_model_(*state->get_pinocchio()), vref_(vref) {}

template <typename Scalar>
ResidualModelCoMVelocityTpl<Scalar>::ResidualModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state,
                                                                 const Vector3s& vref)
    : Base(state, 3, true, true, false), pin_model_(*state->get_pinocchio()), vref_(vref) {}

template <typename Scalar>
ResidualModelCoMVelocityTpl<Scalar>::~ResidualModelCoMVelocityTpl() {}

template <typename Scalar>
void ResidualModelCoMVelocityTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                               const Eigen::Ref<const VectorXs>&x, const Eigen::Ref<const VectorXs>&) {
  // Compute the residual residual give the reference CoM velocity
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());
  
  pinocchio::centerOfMass(pin_model_,*d->pinocchio,q,v);
  data->r = d->pinocchio->vcom[0] - vref_;
}

template <typename Scalar>
void ResidualModelCoMVelocityTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                   const Eigen::Ref<const VectorXs>&x,
                                                   const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());

  const std::size_t nv = state_->get_nv();

  pinocchio::getCenterOfMassVelocityDerivatives(pin_model_,*d->pinocchio, d->dvcom_dq);
  data->Rx.leftCols(nv) = d->dvcom_dq;
  data->Rx.rightCols(nv) = d->pinocchio->Jcom;
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelCoMVelocityTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector3s& ResidualModelCoMVelocityTpl<Scalar>::get_reference() const {
  return vref_;
}

template <typename Scalar>
void ResidualModelCoMVelocityTpl<Scalar>::set_reference(const Vector3s& vref) {
  vref_ = vref;
}

}  // namespace crocoddyl

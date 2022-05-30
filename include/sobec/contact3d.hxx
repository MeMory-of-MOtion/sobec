///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/contact3d.hpp"

namespace sobec {

template <typename Scalar>
ContactModel3DTpl<Scalar>::ContactModel3DTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id,
                                             const Vector3s& xref, const std::size_t nu, const Vector2s& gains,
                                             const pinocchio::ReferenceFrame type)
    : Base(state, id, Vector3s::Zero(), nu, Vector2s::Zero()), xref_(xref), gains_(gains), type_(type) {}

template <typename Scalar>
ContactModel3DTpl<Scalar>::ContactModel3DTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id,
                                             const Vector3s& xref, const Vector2s& gains,
                                             const pinocchio::ReferenceFrame type)
    : Base(state, id, Vector3s::Zero(), Vector2s::Zero()), xref_(xref), gains_(gains), type_(type) {}

template <typename Scalar>
ContactModel3DTpl<Scalar>::~ContactModel3DTpl() {}

template <typename Scalar>
void ContactModel3DTpl<Scalar>::calc(const boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>& data,
                                     const Eigen::Ref<const VectorXs>& x) {
  Data* d = static_cast<Data*>(data.get());
  pinocchio::updateFramePlacement(*state_->get_pinocchio().get(), *d->pinocchio, id_);
  pinocchio::getFrameJacobian(*state_->get_pinocchio().get(), *d->pinocchio, id_, pinocchio::LOCAL, d->fJf);
  d->v = pinocchio::getFrameVelocity(*state_->get_pinocchio().get(), *d->pinocchio, id_, pinocchio::LOCAL);
  d->vv = d->v.linear();
  d->vw = d->v.angular();
  d->Jc = d->fJf.template topRows<3>();
  d->a0_temp_ =
      pinocchio::getFrameClassicalAcceleration(*state_->get_pinocchio().get(), *d->pinocchio, id_, pinocchio::LOCAL)
          .linear();
  d->oRf = d->pinocchio->oMf[id_].rotation();
  // BAUMGARTE P
  if (gains_[0] != 0.) {
    d->a0_temp_ += gains_[0] * d->oRf.transpose() * (d->pinocchio->oMf[id_].translation() - xref_);
  }
  // BAUMGARTE V
  if (gains_[1] != 0.) {
    d->a0_temp_ += gains_[1] * d->vv;
  }
  d->a0 = d->a0_temp_;
  if (type_ == pinocchio::LOCAL_WORLD_ALIGNED || type_ == pinocchio::WORLD) {
    d->a0 = d->oRf * d->a0_temp_;
    d->Jc = d->oRf * d->fJf.template topRows<3>();
  }
}

template <typename Scalar>
void ContactModel3DTpl<Scalar>::calcDiff(const boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>& data,
                                         const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  const pinocchio::JointIndex joint = state_->get_pinocchio()->frames[d->frame].parent;
  pinocchio::getJointAccelerationDerivatives(*state_->get_pinocchio().get(), *d->pinocchio, joint, pinocchio::LOCAL,
                                             d->v_partial_dq, d->a_partial_dq, d->a_partial_dv, d->a_partial_da);
  const std::size_t nv = state_->get_nv();
  pinocchio::skew(d->vv, d->vv_skew);
  pinocchio::skew(d->vw, d->vw_skew);

  d->fXjdv_dq.noalias() = d->fXj * d->v_partial_dq;
  d->fXjda_dq.noalias() = d->fXj * d->a_partial_dq;
  d->fXjda_dv.noalias() = d->fXj * d->a_partial_dv;

  d->da0_dx_temp_.leftCols(nv) = d->fXjda_dq.template topRows<3>();
  d->da0_dx_temp_.leftCols(nv).noalias() += d->vw_skew * d->fXjdv_dq.template topRows<3>();
  d->da0_dx_temp_.leftCols(nv).noalias() -= d->vv_skew * d->fXjdv_dq.template bottomRows<3>();
  d->da0_dx_temp_.rightCols(nv) = d->fXjda_dv.template topRows<3>();
  d->da0_dx_temp_.rightCols(nv).noalias() += d->vw_skew * d->fJf.template topRows<3>();
  d->da0_dx_temp_.rightCols(nv).noalias() -= d->vv_skew * d->fJf.template bottomRows<3>();

  if (gains_[0] != 0.) {
    pinocchio::skew(d->oRf.transpose() * (d->pinocchio->oMf[id_].translation() - xref_), d->tmp_skew_);
    d->da0_dx_temp_.leftCols(nv).noalias() +=
        gains_[0] * (d->tmp_skew_ * d->fJf.template bottomRows<3>() + d->fJf.template topRows<3>());
  }

  if (gains_[1] != 0.) {
    d->da0_dx_temp_.leftCols(nv).noalias() += gains_[1] * d->fXjdv_dq.template topRows<3>();
    d->da0_dx_temp_.rightCols(nv).noalias() += gains_[1] * d->fJf.template topRows<3>();
  }

  d->da0_dx = d->da0_dx_temp_;

  if (type_ == pinocchio::LOCAL_WORLD_ALIGNED || type_ == pinocchio::WORLD) {
    // Need to recompute classical acceleration + BG correction (may not be equal to drift a0 anymore)
    d->a0_temp_ =
        pinocchio::getFrameClassicalAcceleration(*state_->get_pinocchio().get(), *d->pinocchio, id_, pinocchio::LOCAL)
            .linear();
    if (gains_[0] != 0.) {
      d->a0_temp_ += gains_[0] * d->oRf.transpose() * (d->pinocchio->oMf[id_].translation() - xref_);
    }
    if (gains_[1] != 0.) {
      d->a0_temp_ += gains_[1] * d->vv;
    }
    // Skew term due to LWA frame (is zero when classical acceleration = 0, which is the case in ContactFwdDynamics)
    pinocchio::skew(d->oRf * d->a0_temp_, d->tmp_skew_);
    d->da0_dx.leftCols(nv).noalias() =
        d->oRf * d->da0_dx_temp_.leftCols(nv) - d->tmp_skew_ * d->oRf * d->fJf.template bottomRows<3>();
    d->da0_dx.rightCols(nv).noalias() = d->oRf * d->da0_dx_temp_.rightCols(nv);
  }
}

template <typename Scalar>
void ContactModel3DTpl<Scalar>::updateForce(const boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>& data,
                                            const VectorXs& force) {
  if (force.size() != 3) {
    throw_pretty("Invalid argument: "
                 << "lambda has wrong dimension (it should be 3)");
  }
  Data* d = static_cast<Data*>(data.get());
  d->oRf = d->pinocchio->oMf[id_].rotation();
  if (type_ == pinocchio::LOCAL) {
    data->f = d->jMf.act(pinocchio::ForceTpl<Scalar>(force, Vector3s::Zero()));
  }
  if (type_ == pinocchio::LOCAL_WORLD_ALIGNED || type_ == pinocchio::WORLD) {
    data->f = d->jMf.act(pinocchio::ForceTpl<Scalar>(d->oRf.transpose() * force, Vector3s::Zero()));
    // Compute skew term to be added to rnea derivatives
    pinocchio::skew(d->oRf.transpose() * force, d->tmp_skew_);
    d->drnea_skew_term_ = -d->fJf.topRows(3).transpose() * d->tmp_skew_ * d->fJf.bottomRows(3);
  }
}

template <typename Scalar>
boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>> ContactModel3DTpl<Scalar>::createData(
    pinocchio::DataTpl<Scalar>* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void ContactModel3DTpl<Scalar>::print(std::ostream& os) const {
  os << "ContactModel3D {frame=" << state_->get_pinocchio()->frames[id_].name << "}";
}

template <typename Scalar>
const typename crocoddyl::MathBaseTpl<Scalar>::Vector3s& ContactModel3DTpl<Scalar>::get_reference() const {
  return xref_;
}

template <typename Scalar>
const typename crocoddyl::MathBaseTpl<Scalar>::Vector2s& ContactModel3DTpl<Scalar>::get_gains() const {
  return gains_;
}

template <typename Scalar>
void ContactModel3DTpl<Scalar>::set_reference(const Vector3s& reference) {
  xref_ = reference;
}

template <typename Scalar>
void ContactModel3DTpl<Scalar>::set_type(const pinocchio::ReferenceFrame type) {
  type_ = type;
}

template <typename Scalar>
const pinocchio::ReferenceFrame ContactModel3DTpl<Scalar>::get_type() const {
  return type_;
}

}  // namespace sobec

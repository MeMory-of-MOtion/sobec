///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "contact6d.hpp"

namespace sobec {
namespace newcontacts {

template <typename Scalar>
ContactModel6DTpl<Scalar>::ContactModel6DTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id,
                                             const SE3& xref, const std::size_t nu, const Vector2s& gains,
                                             const pinocchio::ReferenceFrame type)
    : Base(state, id, SE3::Identity(), nu, Vector2s::Zero()), xref_(xref), gains_(gains), type_(type) {}

template <typename Scalar>
ContactModel6DTpl<Scalar>::ContactModel6DTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id,
                                             const SE3& xref, const Vector2s& gains,
                                             const pinocchio::ReferenceFrame type)
    : Base(state, id, SE3::Identity(), Vector2s::Zero()), xref_(xref), gains_(gains), type_(type) {}

template <typename Scalar>
ContactModel6DTpl<Scalar>::~ContactModel6DTpl() {}

template <typename Scalar>
void ContactModel6DTpl<Scalar>::calc(const boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>& data,
                                     const Eigen::Ref<const VectorXs>& x) {
  Data* d = static_cast<Data*>(data.get());
  pinocchio::updateFramePlacement(*state_->get_pinocchio().get(), *d->pinocchio, id_);
  pinocchio::getFrameJacobian(*state_->get_pinocchio().get(), *d->pinocchio, id_, pinocchio::LOCAL, d->fJf);
  d->a = pinocchio::getFrameAcceleration(*state_->get_pinocchio().get(), *d->pinocchio, id_);
  d->a0_temp_ = d->a.toVector();
  d->Jc = d->fJf;
  d->lwaMl.rotation(d->pinocchio->oMf[id_].rotation());
  // BAUMGARTE P
  if (gains_[0] != 0.) {
    d->rMf = xref_.inverse() * d->pinocchio->oMf[id_];
    d->a0_temp_ += gains_[0] * pinocchio::log6(d->rMf).toVector();
  }
  // BAUMGARTE V
  if (gains_[1] != 0.) {
    d->v = pinocchio::getFrameVelocity(*state_->get_pinocchio().get(), *d->pinocchio, id_);
    d->a0_temp_ += gains_[1] * d->v.toVector();
  }
  d->a0 = d->a0_temp_;
  if (type_ == pinocchio::LOCAL_WORLD_ALIGNED || type_ == pinocchio::WORLD) {
    d->a0 = d->lwaMl.toActionMatrix() * d->a0_temp_;
    d->Jc = d->lwaMl.toActionMatrix() * d->fJf;
  }
}

template <typename Scalar>
void ContactModel6DTpl<Scalar>::calcDiff(const boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>& data,
                                         const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  const pinocchio::JointIndex joint = state_->get_pinocchio()->frames[d->frame].parent;
  pinocchio::getJointAccelerationDerivatives(*state_->get_pinocchio().get(), *d->pinocchio, joint, pinocchio::LOCAL,
                                             d->v_partial_dq, d->a_partial_dq, d->a_partial_dv, d->a_partial_da);
  const std::size_t nv = state_->get_nv();
  d->da0_dx_temp_.leftCols(nv).noalias() = d->fXj * d->a_partial_dq;
  d->da0_dx_temp_.rightCols(nv).noalias() = d->fXj * d->a_partial_dv;
  if (gains_[0] != 0.) {
    pinocchio::Jlog6(d->rMf, d->rMf_Jlog6);
    d->da0_dx_temp_.leftCols(nv).noalias() += gains_[0] * d->rMf_Jlog6 * d->fJf;
  }
  if (gains_[1] != 0.) {
    d->da0_dx_temp_.leftCols(nv).noalias() += gains_[1] * d->fXj * d->v_partial_dq;
    d->da0_dx_temp_.rightCols(nv).noalias() += gains_[1] * d->fXj * d->a_partial_da;
  }

  d->da0_dx = d->da0_dx_temp_;

  if (type_ == pinocchio::LOCAL_WORLD_ALIGNED || type_ == pinocchio::WORLD) {
    // Need to recompute classical acceleration + BG correction (may not be
    // equal to drift a0 anymore)
    d->a0_temp_ = pinocchio::getFrameAcceleration(*state_->get_pinocchio().get(), *d->pinocchio, id_).toVector();
    // BAUMGARTE P
    if (gains_[0] != 0.) {
      d->rMf = xref_.inverse() * d->pinocchio->oMf[id_];
      d->a0_temp_ += gains_[0] * pinocchio::log6(d->rMf).toVector();
    }
    // BAUMGARTE V
    if (gains_[1] != 0.) {
      d->v = pinocchio::getFrameVelocity(*state_->get_pinocchio().get(), *d->pinocchio, id_);
      d->a0_temp_ += gains_[1] * d->v.toVector();
    }
    // Skew term due to LWA frame (is zero when classical acceleration = 0,
    // which is the case in ContactFwdDynamics)
    d->lwaMl.rotation(d->pinocchio->oMf[id_].rotation());
    d->oRf = d->lwaMl.rotation();
    pinocchio::skew(d->oRf * d->a0_temp_.tail(3), d->tmp_skew_ang_);
    pinocchio::skew(d->oRf * d->a0_temp_.head(3), d->tmp_skew_lin_);
    d->da0_dx.leftCols(nv).noalias() = d->lwaMl.toActionMatrix() * d->da0_dx_temp_.leftCols(nv);
    d->da0_dx.leftCols(nv).topRows(3).noalias() -= d->tmp_skew_lin_ * d->oRf * d->fJf.template bottomRows<3>();
    d->da0_dx.leftCols(nv).bottomRows(3).noalias() -= d->tmp_skew_ang_ * d->oRf * d->fJf.template bottomRows<3>();
    d->da0_dx.rightCols(nv).noalias() = d->lwaMl.toActionMatrix() * d->da0_dx_temp_.rightCols(nv);
  }
}

template <typename Scalar>
void ContactModel6DTpl<Scalar>::updateForce(const boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>& data,
                                            const VectorXs& force) {
  if (force.size() != 6) {
    throw_pretty("Invalid argument: "
                 << "lambda has wrong dimension (it should be 6)");
  }
  Data* d = static_cast<Data*>(data.get());
  d->oRf = d->pinocchio->oMf[id_].rotation();
  d->lwaMl.rotation(d->oRf);
  if (type_ == pinocchio::LOCAL) {
    data->f = d->jMf.act(pinocchio::ForceTpl<Scalar>(force));
  }
  if (type_ == pinocchio::LOCAL_WORLD_ALIGNED || type_ == pinocchio::WORLD) {
    data->f = d->jMf.act(d->lwaMl.actInv(pinocchio::ForceTpl<Scalar>(force)));
    // Compute skew term to be added to rnea derivatives
    pinocchio::skew(d->oRf.transpose() * force.tail(3), d->tmp_skew_ang_);
    pinocchio::skew(d->oRf.transpose() * force.head(3), d->tmp_skew_lin_);
    d->tmp_skew_.topRows(3) = d->tmp_skew_lin_ * d->fJf.bottomRows(3);
    d->tmp_skew_.bottomRows(3) = d->tmp_skew_ang_ * d->fJf.bottomRows(3);
    d->drnea_skew_term_ = -d->fJf.transpose() * d->tmp_skew_;
  }
}

template <typename Scalar>
boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>> ContactModel6DTpl<Scalar>::createData(
    pinocchio::DataTpl<Scalar>* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void ContactModel6DTpl<Scalar>::print(std::ostream& os) const {
  os << "ContactModel6D {frame=" << state_->get_pinocchio()->frames[id_].name << "}";
}

template <typename Scalar>
const pinocchio::SE3Tpl<Scalar>& ContactModel6DTpl<Scalar>::get_reference() const {
  return xref_;
}

template <typename Scalar>
const typename crocoddyl::MathBaseTpl<Scalar>::Vector2s& ContactModel6DTpl<Scalar>::get_gains() const {
  return gains_;
}

template <typename Scalar>
void ContactModel6DTpl<Scalar>::set_reference(const SE3& reference) {
  xref_ = reference;
}

template <typename Scalar>
void ContactModel6DTpl<Scalar>::set_type(const pinocchio::ReferenceFrame type) {
  type_ = type;
}

template <typename Scalar>
const pinocchio::ReferenceFrame& ContactModel6DTpl<Scalar>::get_type() const {
  return type_;
}

}  // namespace newcontacts
}  // namespace sobec

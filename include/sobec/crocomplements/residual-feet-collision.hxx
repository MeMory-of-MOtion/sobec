///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/utils/exception.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "sobec/crocomplements/residual-feet-collision.hpp"

namespace sobec {
using namespace crocoddyl;
template <typename Scalar>
ResidualModelFeetCollisionTpl<Scalar>::ResidualModelFeetCollisionTpl(boost::shared_ptr<StateMultibody> state,
                                                                     const pinocchio::FrameIndex frame_id1,
                                                                     const pinocchio::FrameIndex frame_id2,
                                                                     const std::size_t nu)
    : Base(state, 1, nu, true, false, false),
      frame_id1(frame_id1),
      frame_id2(frame_id2),
      pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelFeetCollisionTpl<Scalar>::ResidualModelFeetCollisionTpl(boost::shared_ptr<StateMultibody> state,
                                                                     const pinocchio::FrameIndex frame_id1,
                                                                     const pinocchio::FrameIndex frame_id2)
    : Base(state, 1, true, false, false),
      frame_id1(frame_id1),
      frame_id2(frame_id2),
      pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelFeetCollisionTpl<Scalar>::~ResidualModelFeetCollisionTpl() {}

template <typename Scalar>
void ResidualModelFeetCollisionTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                 const Eigen::Ref<const VectorXs>& /*x*/,
                                                 const Eigen::Ref<const VectorXs>&) {
  // Compute the residual residual give the reference CoM velocity

  Data* d = static_cast<Data*>(data.get());

  pinocchio::updateFramePlacement(pin_model_, *d->pinocchio, frame_id1);
  pinocchio::updateFramePlacement(pin_model_, *d->pinocchio, frame_id2);

  const typename MathBase::Vector3s& p1 = d->pinocchio->oMf[frame_id1].translation();
  const typename MathBase::Vector3s& p2 = d->pinocchio->oMf[frame_id2].translation();
  d->p1p2 = p1 - p2;
  d->r[0] = d->p1p2.template head<2>().norm();
}

template <typename Scalar>
void ResidualModelFeetCollisionTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                     const Eigen::Ref<const VectorXs>& /*x*/,
                                                     const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  // const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic>
  // q = x.head(state_->get_nq()); const Eigen::VectorBlock<const
  // Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());

  const std::size_t nv = state_->get_nv();
  pinocchio::getFrameJacobian(pin_model_, *d->pinocchio, frame_id1, pinocchio::LOCAL_WORLD_ALIGNED, d->J1);
  pinocchio::getFrameJacobian(pin_model_, *d->pinocchio, frame_id2, pinocchio::LOCAL_WORLD_ALIGNED, d->J2);

  d->dJ = d->J1.template topRows<2>() - d->J2.template topRows<2>();

  data->Rx.leftCols(nv) = d->dJ.row(0) * (d->p1p2[0] / d->r[0]);
  data->Rx.leftCols(nv) += d->dJ.row(1) * (d->p1p2[1] / d->r[0]);
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelFeetCollisionTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
const typename pinocchio::FrameIndex& ResidualModelFeetCollisionTpl<Scalar>::get_frame_id1() const {
  return frame_id1;
}

template <typename Scalar>
void ResidualModelFeetCollisionTpl<Scalar>::set_frame_id1(const pinocchio::FrameIndex& fid) {
  frame_id1 = fid;
}

template <typename Scalar>
const typename pinocchio::FrameIndex& ResidualModelFeetCollisionTpl<Scalar>::get_frame_id2() const {
  return frame_id2;
}

template <typename Scalar>
void ResidualModelFeetCollisionTpl<Scalar>::set_frame_id2(const pinocchio::FrameIndex& fid) {
  frame_id2 = fid;
}

}  // namespace sobec

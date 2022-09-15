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

#include "sobec/crocomplements/residual-2D-surface.hpp"

namespace sobec {
using namespace crocoddyl;
template <typename Scalar>
ResidualModel2DSurfaceTpl<Scalar>::ResidualModel2DSurfaceTpl(
    boost::shared_ptr<StateMultibody> state,
    const pinocchio::FrameIndex frame_id,
    const Vector2s support_translation,
    const Scalar separation,
    const Scalar orientation,
    const std::size_t nu)
    : Base(state, 1, nu, true, false, false),
      frame_id(frame_id),
      support_translation_(support_translation),
      separation_(separation),
      orientation_(orientation),
      pin_model_(state->get_pinocchio()) {
		  A_.resize(1,2);
		  pointA_ = support_translation_;
		  pointA_(0) -= separation_ * sin(orientation_);
		  pointA_(1) += separation_ * cos(orientation_);
		  
		  pointB_ = pointA_;
		  pointB_(0) += cos(orientation_);
		  pointB_(1) += sin(orientation_);
		  A_(0,0) = pointA_(1) - pointB_(1);
		  A_(0,1) = pointB_(0) - pointA_(0);
		  b_ = pointB_(0) * pointA_(1) - pointA_(0) * pointB_(1);
	}

template <typename Scalar>
ResidualModel2DSurfaceTpl<Scalar>::ResidualModel2DSurfaceTpl(
    boost::shared_ptr<StateMultibody> state,
    const pinocchio::FrameIndex frame_id,
    const Vector2s support_translation,
    const Scalar separation,
    const Scalar orientation)
    : Base(state, 1, true, false, false),
      frame_id(frame_id),
      support_translation_(support_translation),
      separation_(separation),
      orientation_(orientation),
      pin_model_(state->get_pinocchio()) {
		  A_.resize(1,2);
		  pointA_ = support_translation_;
		  pointA_(0) -= separation_ * sin(orientation_);
		  pointA_(1) += separation_ * cos(orientation_);
		  
		  pointB_ = pointA_;
		  pointB_(0) += cos(orientation_);
		  pointB_(1) += sin(orientation_);
		  A_(0,0) = pointA_(1) - pointB_(1);
		  A_(0,1) = pointB_(0) - pointA_(0);
		  b_ = pointB_(0) * pointA_(1) - pointA_(0) * pointB_(1);
	}

template <typename Scalar>
ResidualModel2DSurfaceTpl<Scalar>::~ResidualModel2DSurfaceTpl() {}

template <typename Scalar>
void ResidualModel2DSurfaceTpl<Scalar>::calc(
    const boost::shared_ptr<ResidualDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& /*x*/,
    const Eigen::Ref<const VectorXs>&) {
  // Compute the residual residual give the reference CoM velocity

  Data* d = static_cast<Data*>(data.get());

  pinocchio::updateFramePlacement(*pin_model_.get(), *d->pinocchio, frame_id);
  data->r = A_ * d->pinocchio->oMf[frame_id].translation().head(2);
  data->r(0) -= b_;
}

template <typename Scalar>
void ResidualModel2DSurfaceTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ResidualDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& /*x*/,
    const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  const std::size_t nv = state_->get_nv();
   
  pinocchio::getFrameJacobian(*pin_model_.get(), *d->pinocchio, frame_id, pinocchio::LOCAL, d->fJf);
  data->Rx.leftCols(nv).noalias() = A_ * d->pinocchio->oMf[frame_id].rotation().topLeftCorner(2,3) * d->fJf.template topRows<3>();
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> >
ResidualModel2DSurfaceTpl<Scalar>::createData(DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this,
                                      data);
}

template <typename Scalar>
const typename pinocchio::FrameIndex&
ResidualModel2DSurfaceTpl<Scalar>::get_frame_id() const {
  return frame_id;
}

template <typename Scalar>
void ResidualModel2DSurfaceTpl<Scalar>::set_frame_id(
    const pinocchio::FrameIndex& fid) {
  frame_id = fid;
}

template <typename Scalar>
void ResidualModel2DSurfaceTpl<Scalar>::set_Ab(
    const Vector2s support_translation,
    const Scalar orientation) {
  pointA_ = support_translation;
  pointA_(0) -= separation_ * sin(orientation);
  pointA_(1) += separation_ * cos(orientation);
  
  pointB_ = pointA_;
  pointB_(0) += cos(orientation);
  pointB_(1) += sin(orientation);
  A_(0,0) = pointA_(1) - pointB_(1);
  A_(0,1) = pointB_(0) - pointA_(0);
  b_ = pointB_(0) * pointA_(1) - pointA_(0) * pointB_(1);
}

}  // namespace sobec

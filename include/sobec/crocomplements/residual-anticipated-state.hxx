///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/utils/exception.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>

#include "sobec/crocomplements/residual-anticipated-state.hpp"

namespace sobec {

using namespace crocoddyl;

template <typename Scalar>
ResidualModelAnticipatedStateTpl<Scalar>::ResidualModelAnticipatedStateTpl(boost::shared_ptr<typename Base::StateAbstract> state, 
                                                                           const std::size_t nu, 
                                                                           const double& anticipated_time)
    : Base(state, nu, nu, true, true, false), anticipated_time_(anticipated_time) {
}

template <typename Scalar>
ResidualModelAnticipatedStateTpl<Scalar>::ResidualModelAnticipatedStateTpl(boost::shared_ptr<typename Base::StateAbstract> state,
                                                     const double& anticipated_time)
    : Base(state, state->get_nv(), true, true, false), anticipated_time_(anticipated_time) {
}

template <typename Scalar>
ResidualModelAnticipatedStateTpl<Scalar>::~ResidualModelAnticipatedStateTpl() {}

template <typename Scalar>
void ResidualModelAnticipatedStateTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                         const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>&) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());
  const std::size_t nv = state_->get_nv();
  
  Data *d = static_cast<Data *>(data.get());
  VectorXs xout = VectorXs::Zero(nv * 2);
  
  state_->diff(state_->zero(), x, xout);//.tail(nu_); //+ anticipated_time_ * v.tail(nu_); // q; //+ time_matrix_ * v.tail(nu_);
  data->r = xout.head(nv).tail(nu_) + anticipated_time_ * v.tail(nu_);
}

template <typename Scalar>
void ResidualModelAnticipatedStateTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                             const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>&) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  Data *d = static_cast<Data *>(data.get());
  const std::size_t nv = state_->get_nv();
  
  MatrixXs Jx = MatrixXs::Zero(state_->get_ndx(),state_->get_ndx());
  
  state_->Jdiff(state_->zero(), x, Jx, Jx, second);
  data->Rx.rightCols(nu_).diagonal() = VectorXs::Constant(nu_,anticipated_time_);
  data->Rx.leftCols(nv).rightCols(nu_) = Jx.topLeftCorner(nv,nv).bottomRightCorner(nu_,nu_);
  
}

template <typename Scalar>
void ResidualModelAnticipatedStateTpl<Scalar>::print(std::ostream& os) const {
  os << "ResidualModelAnticipatedState";
}

}  // namespace sobec

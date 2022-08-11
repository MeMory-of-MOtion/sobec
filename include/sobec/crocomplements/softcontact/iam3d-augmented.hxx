///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <crocoddyl/core/utils/exception.hpp>
#include <iostream>

#include "iam3d-augmented.hpp"

namespace sobec {
using namespace crocoddyl;
template <typename Scalar>
IAMSoftContact3DAugmentedTpl<Scalar>::IAMSoftContact3DAugmentedTpl(
    boost::shared_ptr<DAMSoftContact3DAugmentedFwdDynamics> model,
    const Scalar& time_step,
    const bool& with_cost_residual)
    : Base(model->get_state(), model->get_nu(),
           model->get_nr() + model->get_nc()),
      differential_(model),
      time_step_(time_step),
      time_step2_(time_step * time_step),
      with_cost_residual_(with_cost_residual) {
  // Downcast DAM state (abstract --> multibody)
  boost::shared_ptr<StateMultibody> state =
      boost::static_pointer_cast<StateMultibody>(model->get_state());
  pin_model_ = state->get_pinocchio();
  // Instantiate stateLPF using pinocchio model of DAM state
  nc_ = 3;
  state_ = boost::make_shared<StateSoftContact>(pin_model_, nc_);
  ny_ = boost::static_pointer_cast<StateSoftContact>(state_)->get_ny();
  // Check stuff
  if (time_step_ < Scalar(0.)) {
    time_step_ = Scalar(1e-3);
    time_step2_ = time_step_ * time_step_;
    std::cerr << "Warning: dt should be positive, set to 1e-3" << std::endl;
  }
}

template <typename Scalar>
IAMSoftContact3DAugmentedTpl<Scalar>::~IAMSoftContact3DAugmentedTpl() {}

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::calc(
    const boost::shared_ptr<ActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& y, 
    const Eigen::Ref<const VectorXs>& u) {
  const std::size_t& nv = differential_->get_state()->get_nv();
  const std::size_t& nx = differential_->get_state()->get_nx();
  const std::size_t& nu_ = differential_->get_nu();

  if (static_cast<std::size_t>(y.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }

  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);
  boost::shared_ptr<DADSoftContact3DAugmentedFwdDynamics> diff_data_soft = boost::static_pointer_cast<DADSoftContact3DAugmentedFwdDynamics>(d->differential);
  // Extract x=(q,v) and f from augmented state y
  const Eigen::Ref<const VectorXs>& x = y.head(nx);   // get q,v_q
  const Eigen::Ref<const VectorXs>& f = y.tail(nc_);  // get f

  if (static_cast<std::size_t>(d->Fy.rows()) !=
      boost::static_pointer_cast<StateSoftContact>(state_)->get_ndy()) {
    throw_pretty(
        "Invalid argument: "
        << "Fy.rows() has wrong dimension (it should be " +
               std::to_string(
                   boost::static_pointer_cast<StateSoftContact>(state_)->get_ndy()) +
               ")");
  }
  if (static_cast<std::size_t>(d->Fy.cols()) !=
      boost::static_pointer_cast<StateSoftContact>(state_)->get_ndy()) {
    throw_pretty(
        "Invalid argument: "
        << "Fy.cols() has wrong dimension (it should be " +
               std::to_string(
                   boost::static_pointer_cast<StateSoftContact>(state_)->get_ndy()) +
               ")");
  }
  if (static_cast<std::size_t>(d->Fu.cols()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "Fw.cols() has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }
  if (static_cast<std::size_t>(d->r.size()) !=
      differential_->get_nr() + nc_) {
    throw_pretty("Invalid argument: "
                 << "r has wrong dimension (it should be " +
                        std::to_string(differential_->get_nr()+ nc_) +
                        ")");
  }
  if (static_cast<std::size_t>(d->Ly.size()) !=
      boost::static_pointer_cast<StateSoftContact>(state_)->get_ndy()) {
    throw_pretty(
        "Invalid argument: "
        << "Ly has wrong dimension (it should be " +
               std::to_string(
                   boost::static_pointer_cast<StateSoftContact>(state_)->get_ndy()) +
               ")");
  }
  if (static_cast<std::size_t>(d->Lu.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "Lw has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }

  // Compute acceleration and cost (DAM, i.e. CT model)
  // a_q, cost = DAM(q, v_q, f, tau_q)
  differential_->calc(diff_data_soft, x, f, u);

  // Computing the next state x+ = x + dx and cost+ = dt*cost
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(nv);
  const VectorXs& a = diff_data_soft->xout;
  const VectorXs& fdot = diff_data_soft->fout;
  d->dy.head(nv).noalias() = v * time_step_ + a * time_step2_;
  d->dy.segment(nv, nv).noalias() = a * time_step_;
  d->dy.tail(nc_).noalias() = fdot * time_step_;
  state_->integrate(y, d->dy, d->ynext);
  d->cost = time_step_ * diff_data_soft->cost;
  if (with_cost_residual_) {
    d->r.head(differential_->get_nr()) = diff_data_soft->r;
    d->r.tail(nc_) = diff_data_soft->f_residual;
  }
}  // calc

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::calc(
    const boost::shared_ptr<ActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& y) {
  const std::size_t& nx = differential_->get_state()->get_nx();

  if (static_cast<std::size_t>(y.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);
  boost::shared_ptr<DADSoftContact3DAugmentedFwdDynamics> diff_data_soft = boost::static_pointer_cast<DADSoftContact3DAugmentedFwdDynamics>(d->differential);
  // Extract x=(q,v) and tau from augmented state y
  const Eigen::Ref<const VectorXs>& x = y.head(nx);  // get q,v_q
  const Eigen::Ref<const VectorXs>& f = y.tail(nc_);  // get q,v_q
  // Compute acceleration and cost (DAM, i.e. CT model)
  differential_->calc(diff_data_soft, x, f);
  d->dy.setZero();
  // d->ynext = y;
  d->cost = diff_data_soft->cost;
  // Update RESIDUAL
  if (with_cost_residual_) {
    d->r.head(differential_->get_nr()) = diff_data_soft->r;
    d->r.tail(nc_) = diff_data_soft->f_residual;
  }
}  // calc


template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& y, 
    const Eigen::Ref<const VectorXs>& u) {
  const std::size_t& nv = differential_->get_state()->get_nv();
  const std::size_t& nx = differential_->get_state()->get_nx();
  const std::size_t& ndx = differential_->get_state()->get_ndx();

  if (static_cast<std::size_t>(y.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }

  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);
  boost::shared_ptr<DADSoftContact3DAugmentedFwdDynamics> diff_data_soft = boost::static_pointer_cast<DADSoftContact3DAugmentedFwdDynamics>(d->differential);
  // Extract x=(q,v) and f from augmented state y
  const Eigen::Ref<const VectorXs>& x = y.head(nx);   // get q,v_q
  const Eigen::Ref<const VectorXs>& f = y.tail(nc_);  // get f

  // Get partials of CT model a_q ('f'), cost w.r.t. (q,v,tau)
  differential_->calcDiff(diff_data_soft, x, f, u);
  const MatrixXs& da_dx = diff_data_soft->Fx;
  const MatrixXs& da_du = diff_data_soft->Fu;

  // Fill out blocks
  d->Fy.topLeftCorner(nv, ndx).noalias() = da_dx * time_step2_;
  d->Fy.block(nv, 0, nv, ndx).noalias() = da_dx * time_step_;
  d->Fy.block(0, nv, nv, nv).diagonal().array() += Scalar(time_step_);
  d->Fu.topRows(nv).noalias() = time_step2_ * da_du;
  d->Fu.block(nv, 0, nv, nu_).noalias() = time_step_ * da_du;

  // New block from augmented dynamics (top right corner)
  d->Fy.topRightCorner(nv, nc_) = diff_data_soft->aba_df * time_step2_;
  d->Fy.block(nv, ndx, nv, nc_) = diff_data_soft->aba_df * time_step_;
  // New block from augmented dynamics (bottom right corner)
  d->Fy.bottomRightCorner(nc_, nc_) = diff_data_soft->dfdt_df*time_step_;
  d->Fy.bottomRightCorner(nc_, nc_).diagonal().array() += Scalar(1.);
  // New block from augmented dynamics (bottom left corner)
  d->Fy.bottomLeftCorner(nc_, ndx) = diff_data_soft->dfdt_dx * time_step_;

  d->Fu.bottomRows(nc_) = diff_data_soft->dfdt_du * time_step_;
  
  state_->JintegrateTransport(y, d->dy, d->Fy, second);
  state_->Jintegrate(y, d->dy, d->Fy, d->Fy, first, addto);
  d->Fy.bottomRightCorner(nc_, nc_).diagonal().array() -= Scalar(1.);  // remove identity from Ftau (due to stateLPF.Jintegrate)
  state_->JintegrateTransport(y, d->dy, d->Fu, second);

  // d->Lx.noalias() = time_step_ * diff_data_soft->Lx;
  d->Ly.head(ndx) = diff_data_soft->Lx*time_step_;
  d->Ly.tail(nc_) = diff_data_soft->Lf*time_step_;
  d->Lyy.topLeftCorner(ndx, ndx) = diff_data_soft->Lxx*time_step_;
  d->Lyy.bottomRightCorner(nc_, nc_) = diff_data_soft->Lff*time_step_;
  d->Lyu.topLeftCorner(ndx, nu_) = diff_data_soft->Lxu*time_step_;
  d->Lu = diff_data_soft->Lu*time_step_;
  d->Luu = diff_data_soft->Luu*time_step_;
}

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& y) {
  const std::size_t& nx = differential_->get_state()->get_nx();
  const std::size_t& ndx = differential_->get_state()->get_ndx();

  if (static_cast<std::size_t>(y.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);
  boost::shared_ptr<DADSoftContact3DAugmentedFwdDynamics> diff_data_soft = boost::static_pointer_cast<DADSoftContact3DAugmentedFwdDynamics>(d->differential);
  // Extract x=(q,v) and f from augmented state y
  const Eigen::Ref<const VectorXs>& x = y.head(nx);   // get q,v_q
  const Eigen::Ref<const VectorXs>& f = y.tail(nc_);  // get f
  // Get partials of CT model a_q ('f'), cost w.r.t. (q,v,tau)
  differential_->calcDiff(diff_data_soft, x, f);
  state_->Jintegrate(y, d->dy, d->Fy, d->Fy);
  // d->Fu.setZero();
  // d(cost+)/dy
  d->Ly.head(ndx).noalias() = diff_data_soft->Lx;
  d->Ly.tail(nc_).noalias() = diff_data_soft->Lf;
  d->Lyy.topLeftCorner(ndx, ndx).noalias() = diff_data_soft->Lxx;
  d->Lyy.bottomRightCorner(nc_, nc_).noalias() = diff_data_soft->Lff;
}

template <typename Scalar>
boost::shared_ptr<ActionDataAbstractTpl<Scalar> >
IAMSoftContact3DAugmentedTpl<Scalar>::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}

template <typename Scalar>
bool IAMSoftContact3DAugmentedTpl<Scalar>::checkData(
    const boost::shared_ptr<ActionDataAbstract>& data) {
  boost::shared_ptr<Data> d = boost::dynamic_pointer_cast<Data>(data);
  boost::shared_ptr<DADSoftContact3DAugmentedFwdDynamics> diff_data_soft = boost::static_pointer_cast<DADSoftContact3DAugmentedFwdDynamics>(d->differential);
  if (data != NULL) {
    return differential_->checkData(diff_data_soft);
  } else {
    return false;
  }
}

template <typename Scalar>
const boost::shared_ptr<DAMSoftContact3DAugmentedFwdDynamicsTpl<Scalar> >&
IAMSoftContact3DAugmentedTpl<Scalar>::get_differential() const {
  return differential_;
}

template <typename Scalar>
const Scalar& IAMSoftContact3DAugmentedTpl<Scalar>::get_dt() const {
  return time_step_;
}


template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::set_dt(const Scalar& dt) {
  if (dt < 0.) {
    throw_pretty("Invalid argument: "
                 << "dt has positive value");
  }
  time_step_ = dt;
  time_step2_ = dt * dt;
}


template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::set_differential(
    boost::shared_ptr<DAMSoftContact3DAugmentedFwdDynamics> model) {
  const std::size_t& nu = model->get_nu();
  if (nu_ != nu) {
    nu_ = nu;
    unone_ = VectorXs::Zero(nu_);
  }
  nr_ = model->get_nr() + nc_;
  state_ = boost::static_pointer_cast<StateSoftContact>(
      model->get_state());  // cast StateAbstract from DAM as StateSoftContact for IAM
  differential_ = model;
  Base::set_u_lb(differential_->get_u_lb());
  Base::set_u_ub(differential_->get_u_ub());
}

// template <typename Scalar>
// void IAMSoftContact3DAugmentedTpl<Scalar>::quasiStatic(
//     const boost::shared_ptr<ActionDataAbstract>& data, Eigen::Ref<VectorXs> u,
//     const Eigen::Ref<const VectorXs>& x, 
//     const std::size_t& maxiter,
//     const Scalar& tol) {
//   if (static_cast<std::size_t>(u.size()) != nu_) {
//     throw_pretty("Invalid argument: "
//                  << "u has wrong dimension (it should be " +
//                         std::to_string(nu_) + ")");
//   }
//   if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
//     throw_pretty("Invalid argument: "
//                  << "x has wrong dimension (it should be " +
//                         std::to_string(state_->get_nx()) + ")");
//   }

//   // Static casting the data
//   boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);

//   differential_->quasiStatic(d->differential, u, x, maxiter, tol);
// }

}  // namespace sobec
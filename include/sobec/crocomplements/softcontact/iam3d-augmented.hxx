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

#include "action.hpp"

namespace sobec {
using namespace crocoddyl;
template <typename Scalar>
IAMSoftContact3DAugmentedTpl<Scalar>::IAMSoftContact3DAugmentedTpl(
    boost::shared_ptr<DifferentialActionModelAbstract> model,
    const Scalar& time_step,
    const bool& with_cost_residual)
    : Base(model->get_state(), model->get_nu(),
           model->get_nr() + 2 * lpf_joint_names.size()),
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
  const std::size_t& nu = differential_->get_nu();

  if (static_cast<std::size_t>(y.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " +
                        std::to_string(nu) + ")");
  }

  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);
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
  if (static_cast<std::size_t>(d->Fw.cols()) != nu) {
    throw_pretty("Invalid argument: "
                 << "Fw.cols() has wrong dimension (it should be " +
                        std::to_string(nu) + ")");
  }
  if (static_cast<std::size_t>(d->r.size()) !=
      differential_->get_nr() + 2 * nc_) {
    throw_pretty("Invalid argument: "
                 << "r has wrong dimension (it should be " +
                        std::to_string(differential_->get_nr() + 2 * nc_) +
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
  if (static_cast<std::size_t>(d->Lw.size()) != nu) {
    throw_pretty("Invalid argument: "
                 << "Lw has wrong dimension (it should be " +
                        std::to_string(nu) + ")");
  }

  // Compute acceleration and cost (DAM, i.e. CT model)
  // a_q, cost = DAM(q, v_q, f, tau_q)
  differential_->calc(d->differential, x, f, u);

  // Computing the next state x+ = x + dx and cost+ = dt*cost
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(nv);
  const VectorXs& a = d->differential->xout;
  const VectorXs& fdot = d->differential->fout;
  d->dy.head(nv).noalias() = v * time_step_ + a * time_step2_;
  d->dy.segment(nv, nv).noalias() = a * time_step_;
  d->dy.tail(nc).noalias() = fdot * time_step_;
  data.xnext = self.stateSoft.integrate(y, data.dx)
  data.cost = self.dt*data.differential.cost
  if(self.withCostResidual):
      data.r = data.differential.r


  // dq(a_q, dt) & dv_q(a_q, dt)
  d->dy.head(nv).noalias() = v * time_step_ + a * time_step2_;
  

  // Update dtau from LPF ids
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  d->dy.tail(nc_) = ((1 - alpha_) * (w - tau))(lpf_torque_ids_);
#else
  for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
    d->dy.tail(nc_)(i) = ((1 - alpha_) * (w - tau))(lpf_torque_ids_[i]);
  }
#endif

  // integrate using stateLPF rule : tau+ = tau + dtau(tau, w)
  state_->integrate(y, d->dy, d->ynext);
  // get cost+ from cost
  d->cost = time_step_ * d->differential->cost;

  // Add hard-coded cost on unfiltered torque a[r(w)] only at LPF joints
  // Torque REG
  if (tauReg_weight_ > 0) {
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
    tauReg_residual_ = w(lpf_torque_ids_) - tauReg_reference_;
#else
    for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
      tauReg_residual_(i) = w(lpf_torque_ids_[i]) - tauReg_reference_(i);
    }
#endif
    // add to cost
    d->cost += Scalar(0.5 * time_step_ * tauReg_weight_ *
                      tauReg_residual_.transpose() * tauReg_residual_);
  }
  // Torque LIM
  if (tauLim_weight_ > 0) {
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
    activation_model_tauLim_->calc(
        d->activation,
        w(lpf_torque_ids_));  // Compute limit cost torque residual of w
    tauLim_residual_ = w(lpf_torque_ids_);
#else
    for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
      tauLim_residual_(i) = w(lpf_torque_ids_[i]);
    }
    activation_model_tauLim_->calc(d->activation, tauLim_residual_);
#endif
    // add to cost
    d->cost += Scalar(0.5 * time_step_ * tauLim_weight_ *
                      d->activation->a_value);  // tau lim
  }

  // Update RESIDUAL
  if (with_cost_residual_) {
    d->r.head(differential_->get_nr()) = d->differential->r;

    // Add unfiltered torque residuals
    if (tauReg_weight_ > 0) {
      d->r.segment(differential_->get_nr(), nc_) = tauReg_residual_;
    }
    if (tauLim_weight_ > 0) {
      d->r.tail(nc_) = tauLim_residual_;
    }
  }
}  // calc

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::calc(
    const boost::shared_ptr<ActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& y) {
  const std::size_t& nv = differential_->get_state()->get_nv();
  const std::size_t& nx = differential_->get_state()->get_nx();

  if (static_cast<std::size_t>(y.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }

  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);
  // Extract x=(q,v) and tau from augmented state y
  const Eigen::Ref<const VectorXs>& x = y.head(nx);  // get q,v_q

  // Compute acceleration and cost (DAM, i.e. CT model)
  differential_->calc(d->differential, x);
  d->dy.setZero();
  // d->ynext = y;
  d->cost = d->differential->cost;
  // Update RESIDUAL
  if (with_cost_residual_) {
    d->r.head(differential_->get_nr()) = d->differential->r;
  }
}  // calc

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& y, const Eigen::Ref<const VectorXs>& w) {
  const std::size_t& nv = differential_->get_state()->get_nv();
  const std::size_t& nx = differential_->get_state()->get_nx();
  const std::size_t& ndx = differential_->get_state()->get_ndx();

  if (static_cast<std::size_t>(y.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  if (static_cast<std::size_t>(w.size()) != nu) {
    throw_pretty("Invalid argument: "
                 << "w has wrong dimension (it should be " +
                        std::to_string(nu) + ")");
  }

  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);

  // Computing the derivatives for the time-continuous model (i.e. differential
  // model)
  // Extract x=(q,v) and tau from augmented state y
  const Eigen::Ref<const VectorXs>& x = y.head(nx);  // get q,v_q
  // d->tau_tmp = w;  // Initialize torques from unfiltered input
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  d->tau_tmp(non_lpf_torque_ids_) =
      w(non_lpf_torque_ids_);                   // NON-LPF dimensions
  d->tau_tmp(lpf_torque_ids_) = y.tail(nc_);  // LPF dimensions
#else
  for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
    d->tau_tmp(lpf_torque_ids_[i]) = y.tail(nc_)(i);
  }
  for (std::size_t i = 0; i < non_lpf_torque_ids_.size(); i++) {
    d->tau_tmp(non_lpf_torque_ids_[i]) = w(non_lpf_torque_ids_[i]);
  }
#endif
  const Eigen::Ref<const VectorXs>& tau = d->tau_tmp;

  // TAU INTEGRATION
  if (!tau_plus_integration_) {
    // Get partials of CT model a_q ('f'), cost w.r.t. (q,v,tau)
    differential_->calcDiff(d->differential, x, tau);

    // Get cost lim w derivatives
    if (tauLim_weight_ > 0) {
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
      activation_model_tauLim_->calcDiff(d->activation, w(lpf_torque_ids_));
#else
      for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
        tauLim_residual_(i) = w(lpf_torque_ids_[i]);
      }
      activation_model_tauLim_->calcDiff(d->activation, tauLim_residual_);
#endif
    }  // tauLim_weight_ != 0

    // Fill RUNNING MODELS partials of (y+,cost+) w.r.t. (y,w)
    const MatrixXs& da_dx = d->differential->Fx;
    const MatrixXs& da_du = d->differential->Fu;
    // d(x+)/dy
    d->Fy.block(0, 0, nv, ndx).noalias() = da_dx * time_step2_;
    d->Fy.block(nv, 0, nv, ndx).noalias() = da_dx * time_step_;
    d->Fy.block(0, nv, nv, nv).diagonal().array() += Scalar(time_step_);

    // Partial blocks of LPF dimensions due y+ dependency on tau
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
    d->Fy.block(0, ndx, nv, nc_).noalias() =
        da_du(Eigen::all, lpf_torque_ids_) * time_step2_;
    d->Fy.block(nv, ndx, nv, nc_).noalias() =
        da_du(Eigen::all, lpf_torque_ids_) * time_step_;
#else
    for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
      d->Fy.block(0, ndx, nv, nc_).col(i).noalias() =
          da_du.col(lpf_torque_ids_[i]) * time_step2_;
      d->Fy.block(nv, ndx, nv, nc_).col(i).noalias() =
          da_du.col(lpf_torque_ids_[i]) * time_step_;
    }
#endif
    // LPF partial block
    d->Fy.bottomRightCorner(nc_, nc_).diagonal().array() = Scalar(alpha_);

    state_->JintegrateTransport(y, d->dy, d->Fy, second);
    state_->Jintegrate(
        y, d->dy, d->Fy, d->Fy, first,
        addto);  // add identity to Fx = d(x+dx)/dx = d(q,v)/d(q,v)
    // !!! remove identity from Ftau (due to stateLPF.Jintegrate) !!!
    d->Fy.bottomRightCorner(nc_, nc_).diagonal().array() -= Scalar(1.);

    // d(y+)/dw : the "non-LPF" dimensions cause the next state to depend
    // directly on the input w since in those dimensions we have tau = w . So we
    // fill the partials w.r.t. w accordingly
    d->Fw.setZero();
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
    d->Fw.topRows(nv)(Eigen::all, non_lpf_torque_ids_).noalias() =
        da_du(Eigen::all, non_lpf_torque_ids_) * time_step2_;
    d->Fw.block(nv, 0, nv, nu)(Eigen::all, non_lpf_torque_ids_).noalias() =
        da_du(Eigen::all, non_lpf_torque_ids_) * time_step_;
    d->Fw.bottomRows(nc_)(Eigen::all, lpf_torque_ids_).diagonal().array() +=
        Scalar(1 - alpha_);
#else
    for (std::size_t i = 0; i < non_lpf_torque_ids_.size(); i++) {
      d->Fw.topRows(nv).col(non_lpf_torque_ids_[i]).noalias() =
          da_du.col(non_lpf_torque_ids_[i]) * time_step2_;
      d->Fw.block(nv, 0, nv, nu).col(non_lpf_torque_ids_[i]).noalias() =
          da_du.col(non_lpf_torque_ids_[i]) * time_step_;
    }
    for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
      d->Fw.bottomRows(nc_)(i, lpf_torque_ids_[i]) += Scalar(1 - alpha_);
    }
#endif
    // LPF partial
    // d->Fw.bottomRows(nc_).diagonal().array() += Scalar(1 - alpha_);
    state_->JintegrateTransport(y, d->dy, d->Fw, second);

    // d(cost+)/dy
    d->Ly.head(ndx).noalias() = time_step_ * d->differential->Lx;
    // Partial blocks for LPF dimensions
    d->Lyy.topLeftCorner(ndx, ndx).noalias() =
        time_step_ * d->differential->Lxx;
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
    d->Ly.tail(nc_).noalias() =
        time_step_ * d->differential->Lu(lpf_torque_ids_);
    d->Lyy.block(0, ndx, ndx, nc_).noalias() =
        time_step_ * d->differential->Lxu(Eigen::all, lpf_torque_ids_);
    d->Lyy.block(ndx, 0, nc_, ndx).noalias() =
        time_step_ *
        d->differential->Lxu.transpose()(lpf_torque_ids_, Eigen::all);
    d->Lyy.bottomRightCorner(nc_, nc_).noalias() =
        time_step_ * d->differential->Luu(lpf_torque_ids_, lpf_torque_ids_);
#else
    for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
      d->Ly.tail(nc_)(i) =
          time_step_ * d->differential->Lu(lpf_torque_ids_[i]);
      d->Lyy.block(0, ndx, ndx, nc_).col(i).noalias() =
          time_step_ * d->differential->Lxu.col(lpf_torque_ids_[i]);
      d->Lyy.block(ndx, 0, nc_, ndx).row(i).noalias() =
          time_step_ * d->differential->Lxu.transpose().row(lpf_torque_ids_[i]);
      for (std::size_t j = 0; j < lpf_torque_ids_.size(); j++) {
        d->Lyy.bottomRightCorner(nc_, nc_)(i, j) =
            time_step_ *
            d->differential->Luu(lpf_torque_ids_[i], lpf_torque_ids_[j]);
      }
    }
#endif

    // Partials blocks for non-LPF dimensions
    d->Lw.setZero();
    d->Lyw.setZero();
    d->Lww.setZero();
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
    d->Lw(non_lpf_torque_ids_).noalias() =
        time_step_ * d->differential->Lu(non_lpf_torque_ids_);
    d->Lyw.topRows(ndx)(Eigen::all, non_lpf_torque_ids_).noalias() =
        time_step_ * d->differential->Lxu(Eigen::all, non_lpf_torque_ids_);
    d->Lyw.bottomRows(nc_)(Eigen::all, non_lpf_torque_ids_).noalias() =
        time_step_ * d->differential->Luu(lpf_torque_ids_, non_lpf_torque_ids_);
    d->Lww(non_lpf_torque_ids_, non_lpf_torque_ids_).noalias() =
        time_step_ *
        d->differential->Luu(non_lpf_torque_ids_, non_lpf_torque_ids_);
#else
    for (std::size_t i = 0; i < non_lpf_torque_ids_.size(); i++) {
      d->Lw(non_lpf_torque_ids_[i]) =
          time_step_ * d->differential->Lu(non_lpf_torque_ids_[i]);
      d->Lyw.topRows(ndx).col(non_lpf_torque_ids_[i]).noalias() =
          time_step_ * d->differential->Lxu.col(non_lpf_torque_ids_[i]);
      for (std::size_t j = 0; j < non_lpf_torque_ids_.size(); j++) {
        d->Lww(non_lpf_torque_ids_[i], non_lpf_torque_ids_[j]) =
            time_step_ * d->differential->Luu(non_lpf_torque_ids_[i],
                                              non_lpf_torque_ids_[j]);
      }
      for (std::size_t j = 0; j < lpf_torque_ids_.size(); j++) {
        d->Lyw.bottomRows(nc_)(j, non_lpf_torque_ids_[i]) =
            time_step_ *
            d->differential->Luu(lpf_torque_ids_[j], non_lpf_torque_ids_[i]);
      }
    }
#endif

    // Partials of hard-coded cost+(tauReg) & cost+(tauLim) w.r.t. (y,w)
    if (tauReg_weight_ > 0) {
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
      d->Lw(lpf_torque_ids_) +=
          time_step_ * tauReg_weight_ *
          d->r.segment(differential_->get_nr(), nc_);  // tau reg
      d->Lww.diagonal().array()(lpf_torque_ids_) +=
          Scalar(time_step_ * tauReg_weight_);  // tau reg
#else
      for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
        d->Lw(lpf_torque_ids_[i]) +=
            time_step_ * tauReg_weight_ *
            d->r(differential_->get_nr() + i);  // tau reg
        d->Lww.diagonal().array()(lpf_torque_ids_[i]) +=
            Scalar(time_step_ * tauReg_weight_);  // tau reg
      }
#endif
    }  // tauReg !=0
    if (tauLim_weight_ > 0) {
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
      d->Lw(lpf_torque_ids_) +=
          time_step_ * tauLim_weight_ * d->activation->Ar;  // tau lim
      d->Lww.diagonal()(lpf_torque_ids_) +=
          time_step_ * tauLim_weight_ *
          d->activation->Arr.diagonal();  // tau lim
#else
      for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
        d->Lw(lpf_torque_ids_[i]) +=
            time_step_ * tauLim_weight_ * d->activation->Ar(i);  // tau lim
        d->Lww.diagonal()(lpf_torque_ids_[i]) +=
            time_step_ * tauLim_weight_ *
            d->activation->Arr.diagonal()(i);  // tau lim
      }
#endif
    }  // tauLim !=0
  }    // tau integration

  //   // TAU PLUS INTEGRATION
  //   else {
  //     // get tau_q+ from (tau_q, w)
  //     const Eigen::Ref<const VectorXs>& tau_plus =
  //         alpha_ * tau + (1 - alpha_) * w;
  //     // Get partials of CT model a_q ('f'), cost w.r.t. (q,v,tau+)
  //     differential_->calcDiff(d->differential, x, tau_plus);
  //     // Get cost lim w
  //     if (!is_terminal_) {
  //       activation_model_tauLim_->calcDiff(d->activation, w);
  //     }
  //     // Fill out partials of IAM
  //     if (enable_integration_) {
  //       const MatrixXs& da_dx = d->differential->Fx;
  //       const MatrixXs& da_du = d->differential->Fu;
  //       d->Fy.block(0, 0, nv, ndx).noalias() = da_dx * time_step2_;
  //       d->Fy.block(nv, 0, nv, ndx).noalias() = da_dx * time_step_;
  // #if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  //       d->Fy.block(0, ndx, nv, nc_).noalias() =
  //           alpha_ * alpha_ * da_du(Eigen::all, lpf_torque_ids_) *
  //           time_step2_;
  //       d->Fy.block(nv, ndx, nv, nc_).noalias() =
  //           alpha_ * da_du(Eigen::all, lpf_torque_ids_) * time_step_;
  // #else
  //       for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
  //         d->Fy.block(0, ndx, nv, nc_).col(i).noalias() =
  //             alpha_ * alpha_ * da_du.col(lpf_torque_ids_[i]) * time_step2_;
  //         d->Fy.block(nv, ndx, nv, nc_).col(i).noalias() =
  //             alpha_ * da_du.col(lpf_torque_ids_[i]) * time_step_;
  //       }
  // #endif
  //       d->Fy.block(0, nv, nv, nv).diagonal().array() +=
  //           Scalar(time_step_);  // dt*identity top row middle col (eq.
  //                                // Jsecond = d(xnext)/d(dx))
  //       // d->Fy.topLeftCorner(nx, nx).diagonal().array() += Scalar(1.); //
  //       // managed by Jintegrate (eq. Jsecond = d(xnext)/d(dx))
  //       d->Fy.bottomRightCorner(nc_, nc_).diagonal().array() =
  //       Scalar(alpha_); d->Fw.topRows(nv).noalias() = da_du * time_step2_ *
  //       (1 - alpha_);
  // #if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  //       d->Fw.block(nv, 0, nv, nc_).noalias() =
  //           da_du(Eigen::all, lpf_torque_ids_) * time_step_ * (1 - alpha_);
  // #else
  //       for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
  //         d->Fw.block(nv, 0, nv, nc_).col(i).noalias() =
  //             da_du.col(lpf_torque_ids_[i]) * time_step_ * (1 - alpha_);
  //       }
  // #endif
  //       d->Fw.bottomRows(nv).diagonal().array() = Scalar(1 - alpha_);
  //       state_->JintegrateTransport(y, d->dy, d->Fy, second);  // it this
  //       correct? state_->Jintegrate(y, d->dy, d->Fy, d->Fy, first,
  //                          addto);  // for d(x+dx)/d(x)
  //       d->Fy.bottomRightCorner(nc_, nc_).diagonal().array() -=
  //           Scalar(1.);  // remove identity from Ftau (due to
  //           stateLPF.Jintegrate)
  //       state_->JintegrateTransport(y, d->dy, d->Fw, second);  // it this
  //       correct? d->Ly.head(ndx).noalias() = time_step_ *
  //       d->differential->Lx;
  // #if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  //       d->Ly.tail(nc_).noalias() =
  //           alpha_ * time_step_ * d->differential->Lu(lpf_torque_ids_);

  // #else
  //       for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
  //         d->Ly.tail(nc_)(i) =
  //             alpha_ * time_step_ * d->differential->Lu(lpf_torque_ids_[i]);
  //       }
  // #endif
  //       d->Lw.noalias() = (1 - alpha_) * time_step_ * d->differential->Lu;
  //       d->Lyy.topLeftCorner(ndx, ndx).noalias() =
  //           time_step_ * d->differential->Lxx;
  // #if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  //       d->Lyy.block(0, ndx, ndx, nc_).noalias() =
  //           alpha_ * time_step_ *
  //           d->differential->Lxu(Eigen::all, lpf_torque_ids_);
  //       d->Lyy.block(ndx, 0, nc_, ndx).noalias() =
  //           alpha_ * time_step_ *
  //           d->differential->Lxu.transpose()(lpf_torque_ids_, Eigen::all);
  //       d->Lyy.bottomRightCorner(nc_, nc_).noalias() =
  //           alpha_ * alpha_ * time_step_ *
  //           d->differential->Luu(lpf_torque_ids_, lpf_torque_ids_);
  //       d->Lyw.topRows(ndx).noalias() =
  //           (1 - alpha_) * time_step_ *
  //           d->differential->Lxu(Eigen::all, lpf_torque_ids_);
  //       d->Lyw.bottomRows(nc_).noalias() =
  //           (1 - alpha_) * alpha_ * time_step_ *
  //           d->differential->Luu(lpf_torque_ids_, lpf_torque_ids_);
  // #else
  //       for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
  //         d->Lyy.block(0, ndx, ndx, nc_).col(i).noalias() =
  //             alpha_ * time_step_ *
  //             d->differential->Lxu.col(lpf_torque_ids_[i]);
  //         d->Lyy.block(ndx, 0, nc_, ndx).row(i).noalias() =
  //             alpha_ * time_step_ *
  //             d->differential->Lxu.transpose().row(lpf_torque_ids_[i]);
  //         d->Lyw.topRows(ndx).col(i).noalias() =
  //             (1 - alpha_) * time_step_ *
  //             d->differential->Lxu.col(lpf_torque_ids_[i]);
  //         for (std::size_t j = 0; j < lpf_torque_ids_.size(); j++) {
  //           d->Lyy.bottomRightCorner(nc_, nc_)(i, j) =
  //               alpha_ * alpha_ * time_step_ *
  //               d->differential->Luu(lpf_torque_ids_[i], lpf_torque_ids_[j]);
  //           d->Lyw.bottomRows(nc_)(i, j) =
  //               (1 - alpha_) * alpha_ * time_step_ *
  //               d->differential->Luu(lpf_torque_ids_[i], lpf_torque_ids_[j]);
  //         }
  //       }
  // #endif
  //       d->Lww.noalias() =
  //           (1 - alpha_) * (1 - alpha_) * time_step_ * d->differential->Luu;
  //       // Add partials related to unfiltered torque costs w_reg, w_lim (only
  //       for
  //       // running models)
  //       if (!is_terminal_) {
  //         // Torque reg and lim
  //         d->Lw.noalias() =
  //             time_step_ * tauReg_weight_ *
  //             d->r.segment(differential_->get_nr(), nu);  // tau reg
  //         d->Lw.noalias() +=
  //             time_step_ * tauLim_weight_ * d->activation->Ar;  // tau lim
  //         d->Lww.diagonal().array() = Scalar(time_step_ * tauReg_weight_); //
  //         reg d->Lww.diagonal() +=
  //             time_step_ * tauLim_weight_ * d->activation->Arr.diagonal(); //
  //             lim
  //       }

  //     } else {
  //       // state_->Jintegrate(y, d->dy, d->Fy, d->Fy);
  //       d->Fw.setZero();
  //       d->Ly.head(ndx).noalias() = d->differential->Lx;

  // #if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  //       d->Ly.tail(nc_).noalias() =
  //           alpha_ * d->differential->Lu(lpf_torque_ids_);
  //       d->Lw.noalias() = (1 - alpha_) * d->differential->Lu;
  //       d->Lyy.topLeftCorner(ndx, ndx).noalias() = d->differential->Lxx;
  //       d->Lyy.block(0, ndx, ndx, nc_).noalias() =
  //           alpha_ * d->differential->Lxu(Eigen::all, lpf_torque_ids_);
  //       d->Lyy.block(ndx, 0, nc_, ndx).noalias() =
  //           alpha_ *
  //           d->differential->Lxu.transpose()(lpf_torque_ids_, Eigen::all);
  //       d->Lyy.bottomRightCorner(nc_, nc_).noalias() =
  //           alpha_ * alpha_ *
  //           d->differential->Luu(lpf_torque_ids_, lpf_torque_ids_);
  //       d->Lyw.topRows(ndx).noalias() = (1 - alpha_) * d->differential->Lxu;
  //       d->Lyw.bottomRows(nc_).noalias() =
  //           (1 - alpha_) * alpha_ *
  //           d->differential->Luu(lpf_torque_ids_, lpf_torque_ids_);
  // #else
  //       for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
  //         d->Ly.tail(nc_)(i) = alpha_ *
  //         d->differential->Lu(lpf_torque_ids_[i]); d->Lw.noalias() = (1 -
  //         alpha_) * d->differential->Lu; d->Lyy.topLeftCorner(ndx,
  //         ndx).noalias() = d->differential->Lxx; d->Lyy.block(0, ndx, ndx,
  //         nc_).col(i).noalias() =
  //             alpha_ * d->differential->Lxu.col(lpf_torque_ids_[i]);
  //         d->Lyy.block(ndx, 0, nc_, ndx).row(i).noalias() =
  //             alpha_ *
  //             d->differential->Lxu.transpose().row(lpf_torque_ids_[i]);
  //         d->Lyw.topRows(ndx).noalias() = (1 - alpha_) *
  //         d->differential->Lxu; for (std::size_t j = 0; j <
  //         lpf_torque_ids_.size(); j++) {
  //           d->Lyy.bottomRightCorner(nc_, nc_)(i, j) =
  //               alpha_ * alpha_ *
  //               d->differential->Luu(lpf_torque_ids_[i], lpf_torque_ids_[j]);
  //           d->Lyw.bottomRows(nc_)(i, j) =
  //               (1 - alpha_) * alpha_ *
  //               d->differential->Luu(lpf_torque_ids_[i], lpf_torque_ids_[j]);
  //         }
  //       }
  // #endif
  //       d->Lww.noalias() = (1 - alpha_) * (1 - alpha_) *
  //       d->differential->Luu;
  //       // Add partials related to unfiltered torque costs w_reg, w_lim (only
  //       for
  //       // running models)
  //       if (!is_terminal_) {
  //         d->Lw.noalias() += tauReg_weight_ *
  //                            d->r.segment(differential_->get_nr(), nc_); //
  //                            reg
  //         d->Lw.noalias() += tauLim_weight_ * d->activation->Ar; // lim
  //         d->Lww.diagonal().array() += Scalar(tauReg_weight_);  // tau reg
  //         d->Lww.diagonal() +=
  //             tauLim_weight_ * d->activation->Arr.diagonal();  // tau lim
  //       }
  //     }
  //   }  // tau_plus_integration
}

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::calcDiff(
    const boost::shared_ptr<ActionDataAbstract>& data,
    const Eigen::Ref<const VectorXs>& y) {
  const std::size_t& nv = differential_->get_state()->get_nv();
  const std::size_t& nx = differential_->get_state()->get_nx();
  const std::size_t& ndx = differential_->get_state()->get_ndx();

  if (static_cast<std::size_t>(y.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);

  // Computing the derivatives for the time-continuous model (i.e. differential
  // model)
  // Extract x=(q,v) and tau from augmented state y
  const Eigen::Ref<const VectorXs>& x = y.head(nx);  // get q,v_q

  differential_->calcDiff(d->differential, x);

  state_->Jintegrate(y, d->dy, d->Fy, d->Fy);
  // d->Fw.setZero();
  // d(cost+)/dy
  d->Ly.head(ndx).noalias() = d->differential->Lx;
  d->Lyy.topLeftCorner(ndx, ndx).noalias() = d->differential->Lxx;
  // Partial blocks for LPF dimensions
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  d->Ly.tail(nc_).noalias() = d->differential->Lu(lpf_torque_ids_);
  d->Lyy.block(0, ndx, ndx, nc_).noalias() =
      d->differential->Lxu(Eigen::all, lpf_torque_ids_);
  d->Lyy.block(ndx, 0, nc_, ndx).noalias() =
      d->differential->Lxu.transpose()(lpf_torque_ids_, Eigen::all);
  d->Lyy.bottomRightCorner(nc_, nc_).noalias() =
      d->differential->Luu(lpf_torque_ids_, lpf_torque_ids_);
#else
  for (std::size_t i = 0; i < lpf_torque_ids_.size(); i++) {
    d->Ly.tail(nc_)(i) = d->differential->Lu(lpf_torque_ids_[i]);
    d->Lyy.block(0, ndx, ndx, nc_).col(i).noalias() =
        d->differential->Lxu.col(lpf_torque_ids_[i]);
    d->Lyy.block(ndx, 0, nc_, ndx).row(i).noalias() =
        d->differential->Lxu.transpose().row(lpf_torque_ids_[i]);
    for (std::size_t j = 0; j < lpf_torque_ids_.size(); j++) {
      d->Lyy.bottomRightCorner(nc_, nc_)(i, j) =
          d->differential->Luu(lpf_torque_ids_[i], lpf_torque_ids_[j]);
    }
  }
#endif
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
  if (data != NULL) {
    return differential_->checkData(d->differential);
  } else {
    return false;
  }
}

template <typename Scalar>
const boost::shared_ptr<DifferentialActionModelAbstractTpl<Scalar> >&
IAMSoftContact3DAugmentedTpl<Scalar>::get_differential() const {
  return differential_;
}

template <typename Scalar>
const Scalar& IAMSoftContact3DAugmentedTpl<Scalar>::get_dt() const {
  return time_step_;
}

template <typename Scalar>
const Scalar& IAMSoftContact3DAugmentedTpl<Scalar>::get_fc() const {
  return fc_;
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
void IAMSoftContact3DAugmentedTpl<Scalar>::set_fc(const Scalar& fc) {
  // Set the cut-off frequency
  if (fc <= 0.) {
    throw_pretty("Invalid argument: "
                 << "fc must be positive");
  } else {
    fc_ = fc;
  }
}

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::set_alpha(const Scalar& alpha) {
  // Set the cut-off frequency
  if (alpha < 0. || alpha > 1) {
    throw_pretty("Invalid argument: "
                 << "alpha must be in [0,1]");
  } else {
    alpha_ = alpha;
  }
}

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::compute_alpha(const Scalar& fc) {
  // Update alpha parameter
  if (fc > 0 && time_step_ != 0) {
    const Scalar& pi = 3.14159;
    // Exponential Moving Average (EMA) (IIR filter) >> quite sharp
    if (filter_ == 0) {
      alpha_ = exp(-2. * pi * fc * time_step_);
    }
    // Using the formula "fc = 1/2*pi*RC" ??? >> less sharp
    if (filter_ == 1) {
      double omega = 1 / (2. * pi * time_step_ * fc);
      alpha_ = omega / (omega + 1);
    }
    // Exact formula to get fc out of EMA's alpha >> inbetween sharp
    if (filter_ == 2) {
      double y = cos(2. * pi * time_step_ * fc);
      alpha_ = 1 - (y - 1 + sqrt(y * y - 4 * y + 3));
    }
  } else {
    alpha_ = 0;
  }
}

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::set_differential(
    boost::shared_ptr<DifferentialActionModelAbstract> model) {
  const std::size_t& nu = model->get_nu();
  if (nu_ != nu) {
    nu_ = nu;
    unone_ = VectorXs::Zero(nu_);
  }
  nr_ = model->get_nr() + 2 * nc_;
  state_ = boost::static_pointer_cast<StateSoftContact>(
      model->get_state());  // cast StateAbstract from DAM as StateSoftContact for IAM
  differential_ = model;
  Base::set_u_lb(differential_->get_u_lb());
  Base::set_u_ub(differential_->get_u_ub());
}

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::set_control_reg_cost(
    const Scalar& weight, const VectorXs& ref) {
  if (weight < 0.) {
    throw_pretty("cost weight is positive ");
  }
  if ((std::size_t)ref.size() != (std::size_t)(nc_)) {
    throw_pretty("cost ref must have size " << nc_);
  }
  tauReg_weight_ = weight;
  tauReg_reference_ = ref;
}

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::set_control_lim_cost(
    const Scalar& weight) {
  if (weight < 0.) {
    throw_pretty("cost weight is positive ");
  }
  tauLim_weight_ = weight;
}

template <typename Scalar>
void IAMSoftContact3DAugmentedTpl<Scalar>::quasiStatic(
    const boost::shared_ptr<ActionDataAbstract>& data, Eigen::Ref<VectorXs> u,
    const Eigen::Ref<const VectorXs>& x, const std::size_t& maxiter,
    const Scalar& tol) {
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " +
                        std::to_string(nu_) + ")");
  }
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " +
                        std::to_string(state_->get_nx()) + ")");
  }

  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);

  differential_->quasiStatic(d->differential, u, x, maxiter, tol);
}

}  // namespace sobec

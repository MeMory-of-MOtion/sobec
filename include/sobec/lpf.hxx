///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <cmath>

#include <crocoddyl/core/utils/exception.hpp>

#include "sobec/lpf.hpp"

namespace sobec {
using namespace crocoddyl;
template <typename Scalar>
IntegratedActionModelLPFTpl<Scalar>::IntegratedActionModelLPFTpl(
    boost::shared_ptr<DifferentialActionModelAbstract> model, const Scalar& time_step, const bool& with_cost_residual,
    const Scalar& fc, const bool& tau_plus_integration, const int& filter, const bool& is_terminal)
    : Base(model->get_state(), model->get_nu(), model->get_nr() + 2 * model->get_nu()),
      differential_(model),
      time_step_(time_step),
      time_step2_(time_step * time_step),
      with_cost_residual_(with_cost_residual),
      fc_(fc),
      tau_plus_integration_(tau_plus_integration),
      nw_(model->get_nu()),
      ny_(model->get_state()->get_nx() + model->get_nu()),
      enable_integration_(true),
      filter_(filter),
      is_terminal_(is_terminal) {
  Base::set_u_lb(differential_->get_u_lb());
  Base::set_u_ub(differential_->get_u_ub());
  if (time_step_ < Scalar(0.)) {
    time_step_ = Scalar(1e-3);
    time_step2_ = time_step_ * time_step_;
    std::cerr << "Warning: dt should be positive, set to 1e-3" << std::endl;
  }
  if (time_step == Scalar(0.) || is_terminal_ == true) {
    enable_integration_ = false;
  }
  // Compute alpha parameter from fc_
  compute_alpha(fc_);
  // Downcast DAM state (abstract --> multibody)
  boost::shared_ptr<StateMultibody> state = boost::static_pointer_cast<StateMultibody>(model->get_state());
  pin_model_ = state->get_pinocchio();
  // Instantiate stateLPF using pinocchio model of DAM state
  state_ = boost::make_shared<StateLPF>(pin_model_, model->get_nu());
  // init barrier for lim cost
  VectorXs wlb = state_->get_lb().tail(nw_);
  VectorXs wub = state_->get_ub().tail(nw_);
  activation_model_w_lim_ = boost::make_shared<ActivationModelQuadraticBarrier>(ActivationBounds(wlb, wub));
  // cost weights are zero by default
  wreg_ = Scalar(0.);
  wlim_ = Scalar(0.);
}

template <typename Scalar>
IntegratedActionModelLPFTpl<Scalar>::~IntegratedActionModelLPFTpl() {}

template <typename Scalar>
void IntegratedActionModelLPFTpl<Scalar>::calc(const boost::shared_ptr<ActionDataAbstract>& data,
                                               const Eigen::Ref<const VectorXs>& y,
                                               const Eigen::Ref<const VectorXs>& w) {
  const std::size_t& nv = differential_->get_state()->get_nv();
  const std::size_t& nq = differential_->get_state()->get_nq();
  const std::size_t& nx = differential_->get_state()->get_nx();
  const std::size_t& ny = state_->get_nx();
  const std::size_t& nu = differential_->get_nu();
  const std::size_t& nw = nu;

  if (static_cast<std::size_t>(y.size()) != ny) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " + std::to_string(ny) + ")");
  }
  if (static_cast<std::size_t>(w.size()) != nw) {
    throw_pretty("Invalid argument: "
                 << "w has wrong dimension (it should be " + std::to_string(nw) + ")");
  }
  
  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);

  // Extract x=(q,v) and tau from augmented state y
  const Eigen::Ref<const VectorXs>& x = y.head(nx);    // get q,v_q
  const Eigen::Ref<const VectorXs>& tau = y.tail(nu);  // get tau_q

  boost::shared_ptr<StateLPF> statelpf = boost::static_pointer_cast<StateLPF>(state_);

  if (static_cast<std::size_t>(x.size()) != nx) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(nx) + ")");
  }
  if (static_cast<std::size_t>(tau.size()) != nu) {
    throw_pretty("Invalid argument: "
                 << "tau has wrong dimension (it should be " + std::to_string(nu) + ")");
  }

  if (static_cast<std::size_t>(d->Fy.rows()) != statelpf->get_ndy()) {
    throw_pretty("Invalid argument: "
                 << "Fy.rows() has wrong dimension (it should be " + std::to_string(statelpf->get_ndy()) + ")");
  }
  if (static_cast<std::size_t>(d->Fy.cols()) != statelpf->get_ndy()) {
    throw_pretty("Invalid argument: "
                 << "Fy.cols() has wrong dimension (it should be " + std::to_string(statelpf->get_ndy()) + ")");
  }
  if (static_cast<std::size_t>(d->Fw.cols()) != statelpf->get_nw()) {
    throw_pretty("Invalid argument: "
                 << "Fw.cols() has wrong dimension (it should be " + std::to_string(statelpf->get_nw()) + ")");
  }
  if (static_cast<std::size_t>(d->r.size()) != differential_->get_nr() + 2 * statelpf->get_nw()) {
    throw_pretty("Invalid argument: "
                 << "r has wrong dimension (it should be " + std::to_string(differential_->get_nr() + 2 * statelpf->get_nw()) + ")");
  }
  if (static_cast<std::size_t>(d->Ly.size()) != statelpf->get_ndy()) {
    throw_pretty("Invalid argument: "
                 << "Ly has wrong dimension (it should be " + std::to_string(statelpf->get_ndy()) + ")");
  }
  if (static_cast<std::size_t>(d->Lw.size()) != statelpf->get_nw()) {
    throw_pretty("Invalid argument: "
                 << "Lw has wrong dimension (it should be " + std::to_string(statelpf->get_nw()) + ")");
  }

  // Compute acceleration and cost (DAM, i.e. CT model)
  if (!tau_plus_integration_) {  // TAU INTEGRATION
    // a,cost = DAM(x,tau)
    differential_->calc(d->differential, x, tau);  // get a_q, cost = DAM(q, v_q, tau_q)
  } else {                                         // TAU PLUS INTEGRATION
    // a,cost = DAM(x,tau+)
    const Eigen::Ref<const VectorXs>& tau_plus = alpha_ * tau + (1 - alpha_) * w;  // get tau_q+ from (tau_q, w)
    differential_->calc(d->differential, x, tau_plus);  // get a_q, cost = DAM(q, v_q, tau_q+)
  }                                                     // DAM.calc

  // To store residuals of hard-coded costs on w regularization and w limit
  VectorXs res_w_reg, res_w_lim;

  // INTEGRATE (only for running nodes)
  if (!is_terminal_) {
    // Computing the next state x+ = x + dx and cost+ = dt*cost
    const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(nv);
    const VectorXs& a = d->differential->xout;
    d->dy.head(nv).noalias() = v * time_step_ + a * time_step2_;  // get     dq(a_q, dt)
    d->dy.segment(nq, nv).noalias() = a * time_step_;             // get   dv_q(a_q, dt)
    d->dy.tail(nu).noalias() = (1 - alpha_) * (w - tau);          // get dtau_q(w, tau, alpha)
    state_->integrate(y, d->dy, d->ynext);                        // integrate using stateLPF rule : tau+ = tau + dtau(tau, w)
    d->cost = time_step_ * d->differential->cost;                 // get cost+ from cost
    // Add hard-coded cost on unfiltered torque a[r(w)]
    if(wreg_ != 0){
      res_w_reg = w - wref_;
      d->cost += Scalar(0.5 * time_step_ * wreg_ * res_w_reg.transpose() * res_w_reg);  // w reg
    }
    if(wlim_ !=0){
      activation_model_w_lim_->calc(d->activation, w);  // Compute limit cost torque residual of w
      res_w_lim = w;
      d->cost += Scalar(0.5 * time_step_ * wlim_ * d->activation->a_value);             // w lim
    }
  } // running model

  // For TERMINAL node, no integration & no cost on control w
  else {
    d->dy.setZero();
    d->ynext = y;
    d->cost = d->differential->cost;
  }  // terminal model

  // Update RESIDUAL
  if (with_cost_residual_) {
    d->r.head(differential_->get_nr() - 2 * nw) = d->differential->r;
    // Add unfiltered torque RESIDUAL(w) for RUNNING MODELS
    if (!is_terminal_) {
      if(wreg_ != 0){
        d->r.segment(differential_->get_nr(), nw) = res_w_reg;  // w reg
      }// wreg_ != 0
      if(wlim_ != 0){
        d->r.tail(nw) = res_w_lim;                              // w lim
      } // wlim_ != 0
    } // running residual
  } // update residual

}  // calc

template <typename Scalar>
void IntegratedActionModelLPFTpl<Scalar>::calcDiff(const boost::shared_ptr<ActionDataAbstract>& data,
                                                   const Eigen::Ref<const VectorXs>& y,
                                                   const Eigen::Ref<const VectorXs>& w) {
  const std::size_t& nv = differential_->get_state()->get_nv();
  const std::size_t& nq = differential_->get_state()->get_nq();
  const std::size_t& nx = differential_->get_state()->get_nx();
  const std::size_t& ndx = differential_->get_state()->get_ndx();
  const std::size_t& ny = state_->get_nx();
  const std::size_t& nu = differential_->get_nu();
  const std::size_t& ndy = ndx + nu; //state_->get_ndy();
  const std::size_t& nw = nu;

  // if (ndx + nu != ndy) {
  //   throw_pretty("Invalid state size: "
  //                << "ndy is wrong (it should be " + std::to_string(ndx + nu) + ")");
  // }
  if (static_cast<std::size_t>(y.size()) != ny) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " + std::to_string(ny) + ")");
  }
  if (static_cast<std::size_t>(w.size()) != nw) {
    throw_pretty("Invalid argument: "
                 << "w has wrong dimension (it should be " + std::to_string(nw) + ")");
  }

  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);

  // Computing the derivatives for the time-continuous model (i.e. differential model)
  const Eigen::Ref<const VectorXs>& x = y.head(nx);    // get q,v_q
  const Eigen::Ref<const VectorXs>& tau = y.tail(nu);  // get tau_q

  // TAU INTEGRATION
  if (!tau_plus_integration_) {
    // Get partials of CT model a_q ('f'), cost w.r.t. (q,v,tau)
    differential_->calcDiff(d->differential, x, tau);
    // Get cost lim w derivatives
    if (!is_terminal_) {
      if(wlim_ != 0){
        activation_model_w_lim_->calcDiff(d->activation, w);
      } // wlim_ != 0
    }

    // Fill RUNNING MODELS partials of (y+,cost+) w.r.t. (y,w)
    if (!is_terminal_) {
      const MatrixXs& da_dx = d->differential->Fx;
      const MatrixXs& da_du = d->differential->Fu;
      // d(x+)/dy
      d->Fy.block(0, 0, nv, ndx).noalias() = da_dx * time_step2_;
      d->Fy.block(nv, 0, nv, ndx).noalias() = da_dx * time_step_;
      d->Fy.block(0, nv, nv, nv).diagonal().array() += Scalar(time_step_);
      d->Fy.block(0, ndx, nv, nu).noalias() = da_du * time_step2_;
      d->Fy.block(nv, ndx, nv, nu).noalias() = da_du * time_step_;
      d->Fy.bottomRightCorner(nv, nu).diagonal().array() = Scalar(alpha_);
      state_->JintegrateTransport(y, d->dy, d->Fy, second);
      state_->Jintegrate(y, d->dy, d->Fy, d->Fy, first, addto);           // add identity to Fx = d(x+dx)/dx = d(q,v)/d(q,v)
      d->Fy.bottomRightCorner(nu, nu).diagonal().array() -= Scalar(1.);   // remove identity to Ftau (stateLPF)
      // d(x+)/dw
      d->Fw.setZero();
      d->Fw.bottomRows(nu).diagonal().array() = Scalar(1 - alpha_);
      state_->JintegrateTransport(y, d->dy, d->Fw, second);
      // d(cost+)/dy
      d->Ly.head(ndx).noalias() = time_step_ * d->differential->Lx;
      d->Ly.tail(nu).noalias() = time_step_ * d->differential->Lu;
      d->Lyy.topLeftCorner(ndx, ndx).noalias() = time_step_ * d->differential->Lxx;
      d->Lyy.block(0, ndx, ndx, nu).noalias() = time_step_ * d->differential->Lxu;
      d->Lyy.block(ndx, 0, nu, ndx).noalias() = time_step_ * d->differential->Lxu.transpose();
      d->Lyy.bottomRightCorner(nu, nu).noalias() = time_step_ * d->differential->Luu;
      // Partials of hard-coded cost+(wreg) & cost+(wlim) w.r.t. (y,w)
      if(wreg_ != 0){
        d->Lw.noalias() = time_step_ * wreg_ * d->r.segment(differential_->get_nr(), nw);  // w reg
        d->Lww.diagonal().array() = Scalar(time_step_ * wreg_);                            // w reg
      }//wreg !=0
      if(wlim_ != 0){
        d->Lw.noalias() += time_step_ * wlim_ * d->activation->Ar;                         // w lim
        d->Lww.diagonal() += time_step_ * wlim_ * d->activation->Arr.diagonal();           // w lim
      } //wlim !=0
    } // !is_terminal                                                                                                 // running model

    // Fill TERMINAL MODEL partials of ( a(yT), cost(yT) ) w.r.t. y
    else {
      state_->Jintegrate(y, d->dy, d->Fy, d->Fy);
      d->Fw.setZero();
      d->Ly.head(ndx).noalias() = d->differential->Lx;
      d->Ly.tail(nu).noalias() = d->differential->Lu;
      d->Lyy.topLeftCorner(ndx, ndx).noalias() = d->differential->Lxx;
      d->Lyy.block(0, ndx, ndx, nu).noalias() = d->differential->Lxu;
      d->Lyy.block(ndx, 0, nu, ndx).noalias() = d->differential->Lxu.transpose();
      d->Lyy.bottomRightCorner(nu, nu).noalias() = d->differential->Luu;
      d->Lw.setZero();
      d->Lww.setZero();
      d->Lyw.setZero();
    }  // terminal model

  }  // tau integration

  // TAU PLUS INTEGRATION
  else {
    // get tau_q+ from (tau_q, w)
    const Eigen::Ref<const VectorXs>& tau_plus = alpha_ * tau + (1 - alpha_) * w;
    // Get partials of CT model a_q ('f'), cost w.r.t. (q,v,tau+)
    differential_->calcDiff(d->differential, x, tau_plus);
    // Get cost lim w
    if (!is_terminal_) {
      activation_model_w_lim_->calcDiff(d->activation, w);
    }
    // Fill out partials of IAM
    if (enable_integration_) {
      const MatrixXs& da_dx = d->differential->Fx;
      const MatrixXs& da_du = d->differential->Fu;
      d->Fy.block(0, 0, nv, ndx).noalias() = da_dx * time_step2_;
      d->Fy.block(nv, 0, nv, ndx).noalias() = da_dx * time_step_;
      d->Fy.block(0, ndx, nv, nu).noalias() = alpha_ * alpha_ * da_du * time_step2_;
      d->Fy.block(nv, ndx, nv, nu).noalias() = alpha_ * da_du * time_step_;
      d->Fy.block(0, nq, nv, nv).diagonal().array() +=
          Scalar(time_step_);  // dt*identity top row middle col (eq. Jsecond = d(xnext)/d(dx))
      // d->Fy.topLeftCorner(nx, nx).diagonal().array() += Scalar(1.);     // managed by Jintegrate (eq. Jsecond =
      // d(xnext)/d(dx))
      d->Fy.bottomRightCorner(nv, nu).diagonal().array() = Scalar(alpha_);
      d->Fw.topRows(nv).noalias() = da_du * time_step2_ * (1 - alpha_);
      d->Fw.block(nv, 0, nv, nu).noalias() = da_du * time_step_ * (1 - alpha_);
      d->Fw.bottomRows(nv).diagonal().array() = Scalar(1 - alpha_);
      state_->JintegrateTransport(y, d->dy, d->Fy, second);      // it this correct?
      state_->Jintegrate(y, d->dy, d->Fy, d->Fy, first, addto);  // for d(x+dx)/d(x)
      state_->JintegrateTransport(y, d->dy, d->Fw, second);      // it this correct?
      d->Ly.head(ndx).noalias() = time_step_ * d->differential->Lx;
      d->Ly.tail(nu).noalias() = alpha_ * time_step_ * d->differential->Lu;
      d->Lw.noalias() = (1 - alpha_) * time_step_ * d->differential->Lu;
      d->Lyy.topLeftCorner(ndx, ndx).noalias() = time_step_ * d->differential->Lxx;
      d->Lyy.block(0, ndx, ndx, nu).noalias() = alpha_ * time_step_ * d->differential->Lxu;
      d->Lyy.block(ndx, 0, nu, ndx).noalias() = alpha_ * time_step_ * d->differential->Lxu.transpose();
      d->Lyy.bottomRightCorner(nu, nu).noalias() = alpha_ * alpha_ * time_step_ * d->differential->Luu;
      d->Lyw.topRows(ndx).noalias() = (1 - alpha_) * time_step_ * d->differential->Lxu;
      d->Lyw.bottomRows(nu).noalias() = (1 - alpha_) * alpha_ * time_step_ * d->differential->Luu;
      d->Lww.noalias() = (1 - alpha_) * (1 - alpha_) * time_step_ * d->differential->Luu;
      // Add partials related to unfiltered torque costs w_reg, w_lim (only for running models)
      if (!is_terminal_) {
        // Torque reg and lim
        d->Lw.noalias() = time_step_ * wreg_ * d->r.segment(differential_->get_nr(), nw);  // w reg
        d->Lw.noalias() += time_step_ * wlim_ * d->activation->Ar;                         // w lim
        d->Lww.diagonal().array() = Scalar(time_step_ * wreg_);                            // reg
        d->Lww.diagonal() += time_step_ * wlim_ * d->activation->Arr.diagonal();           // lim
      }

    } else {
      // state_->Jintegrate(y, d->dy, d->Fy, d->Fy);
      d->Fw.setZero();
      d->Ly.head(ndx).noalias() = d->differential->Lx;
      d->Ly.tail(nu).noalias() = alpha_ * d->differential->Lu;
      d->Lw.noalias() = (1 - alpha_) * d->differential->Lu;
      d->Lyy.topLeftCorner(ndx, ndx).noalias() = d->differential->Lxx;
      d->Lyy.block(0, ndx, ndx, nu).noalias() = alpha_ * d->differential->Lxu;
      d->Lyy.block(ndx, 0, nu, ndx).noalias() = alpha_ * d->differential->Lxu.transpose();
      d->Lyy.bottomRightCorner(nu, nu).noalias() = alpha_ * alpha_ * d->differential->Luu;
      d->Lyw.topRows(ndx).noalias() = (1 - alpha_) * d->differential->Lxu;
      d->Lyw.bottomRows(nu).noalias() = (1 - alpha_) * alpha_ * d->differential->Luu;
      d->Lww.noalias() = (1 - alpha_) * (1 - alpha_) * d->differential->Luu;
      // Add partials related to unfiltered torque costs w_reg, w_lim (only for running models)
      if (!is_terminal_) {
        d->Lw.noalias() += wreg_ * d->r.segment(differential_->get_nr(), nw);  // reg
        d->Lw.noalias() += wlim_ * d->activation->Ar;                          // lim
        d->Lww.diagonal().array() += Scalar(wreg_);                            // w reg
        d->Lww.diagonal() += wlim_ * d->activation->Arr.diagonal();            // w lim
      }
    }
  }  // tau_plus_integration
}

template <typename Scalar>
boost::shared_ptr<ActionDataAbstractTpl<Scalar> > IntegratedActionModelLPFTpl<Scalar>::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}

template <typename Scalar>
bool IntegratedActionModelLPFTpl<Scalar>::checkData(const boost::shared_ptr<ActionDataAbstract>& data) {
  boost::shared_ptr<Data> d = boost::dynamic_pointer_cast<Data>(data);
  if (data != NULL) {
    return differential_->checkData(d->differential);
  } else {
    return false;
  }
}

template <typename Scalar>
const boost::shared_ptr<DifferentialActionModelAbstractTpl<Scalar> >&
IntegratedActionModelLPFTpl<Scalar>::get_differential() const {
  return differential_;
}

template <typename Scalar>
const Scalar& IntegratedActionModelLPFTpl<Scalar>::get_dt() const {
  return time_step_;
}

template <typename Scalar>
const Scalar& IntegratedActionModelLPFTpl<Scalar>::get_fc() const {
  return fc_;
}

template <typename Scalar>
void IntegratedActionModelLPFTpl<Scalar>::set_dt(const Scalar& dt) {
  if (dt < 0.) {
    throw_pretty("Invalid argument: "
                 << "dt has positive value");
  }
  time_step_ = dt;
  time_step2_ = dt * dt;
}

template <typename Scalar>
void IntegratedActionModelLPFTpl<Scalar>::set_fc(const Scalar& fc) {
  // Set the cut-off frequency
  if (fc <= 0.) {
    throw_pretty("Invalid argument: "
                 << "fc must be positive");
  } else {
    fc_ = fc;
  }
}

template <typename Scalar>
void IntegratedActionModelLPFTpl<Scalar>::compute_alpha(const Scalar& fc) {
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
void IntegratedActionModelLPFTpl<Scalar>::set_differential(boost::shared_ptr<DifferentialActionModelAbstract> model) {
  const std::size_t& nu = model->get_nu();
  if (nu_ != nu) {
    nu_ = nu;
    unone_ = VectorXs::Zero(nu_);
  }
  nr_ = model->get_nr() + 2*nu;
  state_ =
      boost::static_pointer_cast<StateLPF>(model->get_state());  // cast StateAbstract from DAM as StateLPF for IAM
  differential_ = model;
  Base::set_u_lb(differential_->get_u_lb());
  Base::set_u_ub(differential_->get_u_ub());
}

template <typename Scalar>
void IntegratedActionModelLPFTpl<Scalar>::set_control_reg_cost(const Scalar& weight, 
                                                                const VectorXs& ref) {
  if (weight <= 0.) {
    throw_pretty("cost weight is positive ");
  }
  wreg_ = weight;
  wref_ = ref;
}


template <typename Scalar>
void IntegratedActionModelLPFTpl<Scalar>::set_control_lim_cost(const Scalar& weight) {
  if (weight <= 0.) {
    throw_pretty("cost weight is positive ");
  }
  wlim_ = weight;
}

template <typename Scalar>
void IntegratedActionModelLPFTpl<Scalar>::quasiStatic(const boost::shared_ptr<ActionDataAbstract>& data,
                                                      Eigen::Ref<VectorXs> u, const Eigen::Ref<const VectorXs>& x,
                                                      const std::size_t& maxiter, const Scalar& tol) {
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }

  // Static casting the data
  boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);

  differential_->quasiStatic(d->differential, u, x, maxiter, tol);
}

}  // namespace sobec
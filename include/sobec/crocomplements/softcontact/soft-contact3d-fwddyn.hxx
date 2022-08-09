///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/core/utils/math.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "soft-contact3d-fwddyn.hpp"

namespace sobec {

template <typename Scalar>
DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::DifferentialActionModelSoftContact3DFwdDynamicsTpl(
    boost::shared_ptr<StateMultibody> state, 
    boost::shared_ptr<ActuationModelAbstract> actuation,
    boost::shared_ptr<CostModelSum> costs,
    const pinocchio::FrameIndex frameId,
    const double Kp, 
    const double Kv,
    const Vector3s& oPc,
    const pinocchio::ReferenceFrame ref)
    : Base(state, actuation, costs) {
  if (this->get_costs()->get_nu() != this->get_nu()) {
    throw_pretty("Invalid argument: "
                 << "Costs doesn't have the same control dimension (it should be " + std::to_string(this->get_nu()) + ")");
  }
  Base::set_u_lb(Scalar(-1.) * this->get_pinocchio().effortLimit.tail(this->get_nu()));
  Base::set_u_ub(Scalar(+1.) * this->get_pinocchio().effortLimit.tail(this->get_nu()));
  // Soft contact model parameters
  if(Kp < 0.){
     throw_pretty("Invalid argument: "
                << "Kp must be positive "); 
  }
  if(Kv < 0.){
     throw_pretty("Invalid argument: "
                << "Kv must be positive "); 
  }
  Kp_ = Kp;
  Kv_ = Kv;
  oPc_ = oPc;
  frameId_ = frameId;
  ref_ = ref;
  with_force_cost_ = false;
  active_contact_ = true;
  nc_ = 3;
  parentId_ = this->get_pinocchio().frames[frameId_].parent;
  jMf_ = this->get_pinocchio().frames[frameId_].placement;
}

template <typename Scalar>
DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::~DifferentialActionModelSoftContact3DFwdDynamicsTpl() {}

template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::calc(
            const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
            const Eigen::Ref<const VectorXs>& x,
            const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != this->get_nu()) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(this->get_nu()) + ")");
  }

  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(this->get_state()->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(this->get_state()->get_nv());
  pinocchio::computeAllTerms(this->get_pinocchio(), d->pinocchio, q, v);
  pinocchio::updateFramePlacements(this->get_pinocchio(), d->pinocchio);
  d->oRf = d->pinocchio.oMf[frameId_].rotation();
  
  // Actuation calc
  this->get_actuation()->calc(d->multibody.actuation, x, u);
  
  // If contact is active, compute aq = ABA(q,v,tau,fext)
  if(active_contact_){
    // Computing the dynamics using ABA or manually for armature case
    // if (!with_armature_) {
    // Compute spring damper force expressed at joint level 
    d->lv = pinocchio::getFrameVelocity(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL).linear();
    d->f = -Kp_ * d->oRf.transpose() * ( d->pinocchio.oMf[frameId_].translation() - oPc_ ) - Kv_*d->lv;
    d->pinForce = pinocchio::ForceTpl<Scalar>(d->f, Vector3s::Zero());
    d->fext[parentId_] = jMf_.act(d->pinForce);
    // Save local force for later
    d->f_copy = d->f;
    d->fext_copy = d->fext;
    // rotate if not local 
    // std::cout << "[DAM soft 3D calc] hello from calc ! " << std::endl;
    if(ref_ != pinocchio::LOCAL){
        d->f = -Kp_ * ( d->pinocchio.oMf[frameId_].translation() - oPc_ ) - Kv_ * d->oRf * d->lv;
        d->pinForce = pinocchio::ForceTpl<Scalar>(d->oRf.transpose() * d->f, Vector3s::Zero());
        d->fext[parentId_] = jMf_.act(d->pinForce);
    }
    d->xout = pinocchio::aba(this->get_pinocchio(), d->pinocchio, q, v, d->multibody.actuation->tau, d->fext);
    pinocchio::updateGlobalPlacements(this->get_pinocchio(), d->pinocchio);
    // } 
    // else {
    //     pinocchio::computeAllTerms(this->get_pinocchio(), d->pinocchio, q, v);
    //     d->pinocchio.M.diagonal() += this->get_armature();
    //     pinocchio::cholesky::decompose(this->get_pinocchio(), d->pinocchio);
    //     d->Minv.setZero();
    //     pinocchio::cholesky::computeMinv(this->get_pinocchio(), d->pinocchio, d->Minv);
    //     d->u_drift = d->multibody.actuation->tau - d->pinocchio.nle;
    //     d->xout.noalias() = d->Minv * d->u_drift;
    // }
  } 
  // If contact NOT active : compute aq = ABA(q,v,tau)
  else {
    // Computing the dynamics using ABA or manually for armature case
    if (!with_armature_) {
        d->xout = pinocchio::aba(this->get_pinocchio(), d->pinocchio, q, v, d->multibody.actuation->tau);
        pinocchio::updateGlobalPlacements(this->get_pinocchio(), d->pinocchio);
    } else {
        pinocchio::computeAllTerms(this->get_pinocchio(), d->pinocchio, q, v);
        d->pinocchio.M.diagonal() += this->get_armature();
        pinocchio::cholesky::decompose(this->get_pinocchio(), d->pinocchio);
        d->Minv.setZero();
        pinocchio::cholesky::computeMinv(this->get_pinocchio(), d->pinocchio, d->Minv);
        d->u_drift = d->multibody.actuation->tau - d->pinocchio.nle;
        d->xout.noalias() = d->Minv * d->u_drift;
    }
  }

  // Computing the cost value and residuals
  this->get_costs()->calc(d->costs, x, u);
  d->cost = d->costs->cost;

  // Add hard-coded cost on contact force
  if(active_contact_ && with_force_cost_){
    d->f_residual = d->f - force_des_;
    d->cost += 0.5* force_weight_ * d->f_residual.transpose() * d->f_residual;
  }
}


template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::calc(
            const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
            const Eigen::Ref<const VectorXs>& x) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(this->get_state()->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(this->get_state()->get_nv());
  pinocchio::computeAllTerms(this->get_pinocchio(), d->pinocchio, q, v);
  this->get_costs()->calc(d->costs, x);
  d->cost = d->costs->cost;
}



template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
    const Eigen::Ref<const VectorXs>& x,
    const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != this->get_nu()) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(this->get_nu()) + ")");
  }

  const std::size_t nv = this->get_state()->get_nv();
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(this->get_state()->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(nv);
  
  Data* d = static_cast<Data*>(data.get());

  d->oRf = d->pinocchio.oMf[frameId_].rotation();

  // Actuation calcDiff
  this->get_actuation()->calcDiff(d->multibody.actuation, x, u);
  
  // If contact is active, compute ABA derivatives + force
  if(active_contact_){
    // std::cout << "[DAM soft 3D calc] hello from calcDiff ! " << std::endl;
    // Compute spring damper force derivatives in LOCAL
    pinocchio::framesForwardKinematics(this->get_pinocchio(), d->pinocchio, q);
    pinocchio::getFrameJacobian(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL, d->lJ);
    pinocchio::getFrameJacobian(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL_WORLD_ALIGNED, d->oJ);
    pinocchio::getFrameVelocityDerivatives(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL, d->lv_partial_dq, d->lv_partial_dv);
    d->df_dx.leftCols(nv) = 
        -Kp_ * (d->lJ.topRows(3) + pinocchio::skew(d->oRf.transpose() * (d->pinocchio.oMf[frameId_].translation() - oPc_)) * d->lJ.bottomRows(3)) - Kv_* d->lv_partial_dq.topRows(3);
    d->df_dx.rightCols(nv) = 
        -Kv_ * d->lv_partial_dv.topRows(3);
    // copy for later
    d->df_dx_copy = d->df_dx;
    // rotate force derivatives if not local 
    if(ref_ != pinocchio::LOCAL){
        d->df_dx.leftCols(nv) = d->oRf * d->df_dx_copy.leftCols(nv) - pinocchio::skew(d->oRf * d->f_copy) * d->oJ.bottomRows(3);
        d->df_dx.rightCols(nv) = d->oRf * d->df_dx_copy.rightCols(nv);
    }
    // Compute ABA derivatives (same in LOCAL and LWA for 3D contact)
    pinocchio::computeABADerivatives(this->get_pinocchio(), d->pinocchio, q, v, d->multibody.actuation->tau, d->fext_copy, 
                                                              d->aba_dq, d->aba_dv, d->aba_dtau);
    d->Fx.leftCols(nv) = d->aba_dq + d->aba_dtau * d->lJ.topRows(3).transpose() * d->df_dx_copy.leftCols(nv);
    d->Fx.rightCols(nv) = d->aba_dv + d->aba_dtau * d->lJ.topRows(3).transpose() * d->df_dx_copy.rightCols(nv);
    d->Fx += d->aba_dtau * d->multibody.actuation->dtau_dx;
    d->Fu = d->aba_dtau * d->multibody.actuation->dtau_du;
  }

  // Else ABA Derivatives
  else{
    // Computing the dynamics derivatives
    if (!with_armature_) {
        pinocchio::computeABADerivatives(this->get_pinocchio(), d->pinocchio, q, v, d->multibody.actuation->tau, d->Fx.leftCols(nv),
                                        d->Fx.rightCols(nv), d->pinocchio.Minv);
        d->Fx.noalias() += d->pinocchio.Minv * d->multibody.actuation->dtau_dx;
        d->Fu.noalias() = d->pinocchio.Minv * d->multibody.actuation->dtau_du;
    } else {
        pinocchio::computeRNEADerivatives(this->get_pinocchio(), d->pinocchio, q, v, d->xout);
        d->dtau_dx.leftCols(nv) = d->multibody.actuation->dtau_dx.leftCols(nv) - d->pinocchio.dtau_dq;
        d->dtau_dx.rightCols(nv) = d->multibody.actuation->dtau_dx.rightCols(nv) - d->pinocchio.dtau_dv;
        d->Fx.noalias() = d->Minv * d->dtau_dx;
        d->Fu.noalias() = d->Minv * d->multibody.actuation->dtau_du;
    }
  }


  // Computing the cost derivatives
  this->get_costs()->calcDiff(d->costs, x, u);
  
  // Add costs on force 
  d->Lx = d->costs->Lx;
  d->Lu = d->costs->Lu;
  d->Lxx = d->costs->Lxx;
  d->Lxu = d->costs->Lxu;
  d->Luu = d->costs->Luu;
  if(active_contact_ && with_force_cost_){
      d->f_residual = d->f - force_des_;
      d->Lx += force_weight_ * d->f_residual.transpose() * d->df_dx;
      d->Lxx += force_weight_ * d->df_dx.transpose() * d->df_dx;
  }
}


template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
    const Eigen::Ref<const VectorXs>& x) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  Data* d = static_cast<Data*>(data.get());
  this->get_costs()->calcDiff(d->costs, x);
}


template <typename Scalar>
boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> >
DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}

template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::set_force_cost(const Vector3s& force_des, 
                                                                                const Scalar force_weight) {
  if (force_weight < 0.) {
    throw_pretty("Invalid argument: "
                 << "Force weight should be positive");
  }
  force_des_ = force_des;
  force_weight_ = force_weight;
  with_force_cost_ = true;
}

template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::set_Kp(const Scalar inKp) {
  if (inKp < 0.) {
    throw_pretty("Invalid argument: "
                 << "Stiffness should be positive");
  }
  Kp_ = inKp;
}

template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::set_Kv(const Scalar inKv) {
  if (inKv < 0.) {
    throw_pretty("Invalid argument: "
                 << "Damping should be positive");
  }
  Kv_ = inKv;
}

template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::set_oPc(const Vector3s& inoPc) {
  if (inoPc.size() != 3) {
    throw_pretty("Invalid argument: "
                 << "Anchor point position should have size 3");
  }
  oPc_ = inoPc;
}

template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::set_force_des(const Vector3s& inForceDes) {
  if (inForceDes.size() != 3) {
    throw_pretty("Invalid argument: "
                 << "Desired force should have size 3");
  }
  force_des_ = inForceDes;
}

template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::set_force_weight(const Scalar inForceWeight) {
  if (inForceWeight < 0.) {
    throw_pretty("Invalid argument: "
                 << "Force cost weight should be positive");
  }
  force_weight_ = inForceWeight;
}

template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::set_ref(const pinocchio::ReferenceFrame inRef) {
  ref_ = inRef;
}

template <typename Scalar>
void DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::set_id(const pinocchio::FrameIndex inId) {
  frameId_ = inId;
}

template <typename Scalar>
const Scalar DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::get_Kp() const {
  return Kp_;
}

template <typename Scalar>
const Scalar DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::get_Kv() const {
  return Kv_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector3s& DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::get_oPc() const {
  return oPc_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector3s& DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::get_force_des() const {
  return force_des_;
}

template <typename Scalar>
const Scalar DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::get_force_weight() const {
  return force_weight_;
}

template <typename Scalar>
const pinocchio::ReferenceFrame DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::get_ref() const {
  return ref_;
}

template <typename Scalar>
const pinocchio::FrameIndex DifferentialActionModelSoftContact3DFwdDynamicsTpl<Scalar>::get_id() const {
  return frameId_;
}

}  // namespace sobec

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_LPF_HPP_
#define SOBEC_LPF_HPP_

#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/diff-action-base.hpp>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/multibody/model.hpp>

#include "state.hpp"

namespace sobec {
using namespace crocoddyl;

template <typename _Scalar>
class IntegratedActionModelLPFTpl : public ActionModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ActionModelAbstractTpl<Scalar> Base;
  typedef IntegratedActionDataLPFTpl<Scalar> Data;
  typedef ActionDataAbstractTpl<Scalar> ActionDataAbstract;
  typedef DifferentialActionModelAbstractTpl<Scalar>
      DifferentialActionModelAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef StateLPFTpl<Scalar> StateLPF;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef pinocchio::ModelTpl<Scalar> PinocchioModel;
  typedef ActivationModelQuadraticBarrierTpl<Scalar>
      ActivationModelQuadraticBarrier;
  typedef ActivationBoundsTpl<Scalar> ActivationBounds;

  IntegratedActionModelLPFTpl(
      boost::shared_ptr<DifferentialActionModelAbstract> model,
      std::vector<std::string> lpf_joint_names = {},
      const Scalar& time_step = Scalar(1e-3),
      const bool& with_cost_residual = true, const Scalar& fc = 0,
      const bool& tau_plus_integration = true, const int& filter = 0);
  virtual ~IntegratedActionModelLPFTpl();

  virtual void calc(const boost::shared_ptr<ActionDataAbstract>& data,
                    const Eigen::Ref<const VectorXs>& y,
                    const Eigen::Ref<const VectorXs>& w);

  virtual void calc(const boost::shared_ptr<ActionDataAbstract>& data,
                    const Eigen::Ref<const VectorXs>& y);

  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& y,
                        const Eigen::Ref<const VectorXs>& w);

  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& y);

  virtual boost::shared_ptr<ActionDataAbstract> createData();
  virtual bool checkData(const boost::shared_ptr<ActionDataAbstract>& data);

  virtual void quasiStatic(const boost::shared_ptr<ActionDataAbstract>& data,
                           Eigen::Ref<VectorXs> u,
                           const Eigen::Ref<const VectorXs>& x,
                           const std::size_t& maxiter = 100,
                           const Scalar& tol = Scalar(1e-9));

  const boost::shared_ptr<DifferentialActionModelAbstract>& get_differential()
      const;
  const Scalar& get_dt() const;
  const Scalar& get_fc() const;
  const Scalar& get_alpha() const { return alpha_; };

  const std::size_t& get_nw() const { return nw_; };
  const std::size_t& get_ntau() const { return ntau_; };
  const std::size_t& get_ny() const { return ny_; };

  const std::vector<std::string>& get_lpf_joint_names() const {
    return lpf_joint_names_;
  };

  const std::vector<int>& get_lpf_torque_ids() const {
    return lpf_torque_ids_;
  };
  const std::vector<int>& get_non_lpf_torque_ids() const {
    return non_lpf_torque_ids_;
  };

  void set_dt(const Scalar& dt);
  void set_fc(const Scalar& fc);
  void set_alpha(const Scalar& alpha);
  void set_differential(
      boost::shared_ptr<DifferentialActionModelAbstract> model);

  // hard-coded costs
  void set_control_reg_cost(const Scalar& cost_weight_w_reg,
                            const VectorXs& cost_ref_w_reg);
  void set_control_lim_cost(const Scalar& cost_weight_w_lim);

  void compute_alpha(const Scalar& fc);

 protected:
  using Base::has_control_limits_;  //!< Indicates whether any of the control
                                    //!< limits are active
  using Base::nr_;                  //!< Dimension of the cost residual
  using Base::nu_;                  //!< Control dimension
  using Base::u_lb_;                //!< Lower control limits
  using Base::u_ub_;                //!< Upper control limits
  using Base::unone_;               //!< Neutral state
  std::size_t nw_;                  //!< Input torque dimension (unfiltered)
  std::size_t ntau_;   //!< Filtered torque dimension ("lpf" dimension)
  std::size_t ny_;     //!< Augmented state dimension : nq+nv+ntau
  using Base::state_;  //!< Model of the state

 public:
  boost::shared_ptr<ActivationModelQuadraticBarrier>
      activation_model_tauLim_;  //!< for lim cost

 private:
  boost::shared_ptr<DifferentialActionModelAbstract> differential_;
  Scalar time_step_;
  Scalar time_step2_;
  Scalar alpha_;
  bool with_cost_residual_;
  Scalar fc_;
  // bool enable_integration_;
  Scalar tauReg_weight_;  //!< Cost weight for unfiltered torque regularization
  VectorXs tauReg_reference_;  //!< Cost reference for unfiltered torque
                               //!< regularization
  VectorXs tauReg_residual_,
      tauLim_residual_;  //!< Residuals for LPF torques reg and lim
  // bool gravity_reg_;                          //!< Use gravity torque for
  // unfiltered torque reg, or user-provided reference?
  Scalar tauLim_weight_;       //!< Cost weight for unfiltered torque limits
  bool tau_plus_integration_;  //!< Use tau+ = LPF(tau,w) in acceleration
                               //!< computation, or tau
  int filter_;                 //!< Type of LPF used>
  boost::shared_ptr<PinocchioModel> pin_model_;  //!< for reg cost
  bool is_terminal_;  //!< is it a terminal model or not ? (deactivate cost on w
                      //!< if true)
  std::vector<std::string>
      lpf_joint_names_;  //!< Vector of joint names that are low-pass filtered
  std::vector<int>
      lpf_joint_ids_;  //!< Vector of joint ids that are low-pass filtered
  std::vector<int>
      lpf_torque_ids_;  //!< Vector of torque ids that are low-passs filtered

  //   std::vector<std::string> non_lpf_joint_names_;  //!< Vector of joint
  //   names that are NOT low-pass filtered
  std::vector<int> non_lpf_joint_ids_;   //!< Vector of joint ids that are NOT
                                         //!< low-pass filtered
  std::vector<int> non_lpf_torque_ids_;  //!< Vector of torque ids that are NOT
                                         //!< low-passs filtered
};

template <typename _Scalar>
struct IntegratedActionDataLPFTpl : public ActionDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ActionDataAbstractTpl<Scalar> Base;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef pinocchio::DataTpl<Scalar> PinocchioData;
  typedef DifferentialActionDataAbstractTpl<Scalar>
      DifferentialActionDataAbstract;
  typedef ActivationDataQuadraticBarrierTpl<Scalar>
      ActivationDataQuadraticBarrier;  // for lim cost

  template <template <typename Scalar> class Model>
  explicit IntegratedActionDataLPFTpl(Model<Scalar>* const model)
      : Base(model), tau_tmp(model->get_nu()) {
    tau_tmp.setZero();
    differential = model->get_differential()->createData();
    const std::size_t& ndy = model->get_state()->get_ndx();
    dy = VectorXs::Zero(ndy);
    // for wlim cost
    activation = boost::static_pointer_cast<ActivationDataQuadraticBarrier>(
        model->activation_model_tauLim_->createData());
  }
  virtual ~IntegratedActionDataLPFTpl() {}

  boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> > differential;
  VectorXs dy;

  // PinocchioData pinocchio;                                       // for reg
  // cost
  boost::shared_ptr<ActivationDataQuadraticBarrier> activation;  // for lim cost

  using Base::cost;
  using Base::r;
  VectorXs tau_tmp;
  // use refs to "alias" base class member names
  VectorXs& ynext = Base::xnext;
  MatrixXs& Fy = Base::Fx;
  MatrixXs& Fw = Base::Fu;
  VectorXs& Ly = Base::Lx;
  VectorXs& Lw = Base::Lu;
  MatrixXs& Lyy = Base::Lxx;
  MatrixXs& Lyw = Base::Lxu;
  MatrixXs& Lww = Base::Luu;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/crocomplements/lowpassfilter/action.hxx"

#endif  // SOBEC_LPF_HPP_

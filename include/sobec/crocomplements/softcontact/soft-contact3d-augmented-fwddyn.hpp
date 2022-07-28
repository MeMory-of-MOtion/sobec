///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, CTU, INRIA,
// University of Oxford Copyright note valid unless otherwise stated in
// individual files. All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_SOFTCONTACT3D_FWDDYN_HPP_
#define SOBEC_SOFTCONTACT3D_FWDDYN_HPP_

#include <stdexcept>

#include "crocoddyl/core/actuation-base.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/diff-action-base.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"

// #include "sobec/crocomplements/contact/contact-fwddyn.hpp"
#include "sobec/fwd.hpp"

namespace sobec {

/**
 * @brief Differential action model for visco-elastic contact forward dynamics in multibody
 * systems.
 *
 * Maths here : https://www.overleaf.com/read/xdpymjfhqqhn
 *
 * \sa `DifferentialActionModelAbstractTpl`, `calc()`, `calcDiff()`,
 * `createData()`
 */
template <typename _Scalar>
class DifferentialActionModelSoftContact3DFwdDynamicsTpl
    : public crocoddyl::DifferentialActionModelFreeFwdDynamicsTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef crocoddyl::DifferentialActionModelFreeFwdDynamicsTpl<Scalar> Base;
  typedef crocoddyl::DifferentialActionDataFreeFwdDynamicsTpl<Scalar> Data;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef crocoddyl::CostModelSumTpl<Scalar> CostModelSum;
  typedef crocoddyl::StateMultibodyTpl<Scalar> StateMultibody;
  typedef crocoddyl::ActuationModelAbstractTpl<Scalar> ActuationModelAbstract;
  typedef crocoddyl::DifferentialActionDataAbstractTpl<Scalar> DifferentialActionDataAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::MatrixXs MatrixXs;

  /**
   * @brief Initialize the soft contact forward-dynamics action model
   *
   * It describes the dynamics evolution of a multibody system under
   * visco-elastic contact (linear spring damper force)
   *
   * @param[in] state            State of the multibody system
   * @param[in] actuation        Actuation model
   * @param[in] costs            Stack of cost functions
   * @param[in] frameId          Pinocchio frame id of the frame in contact
   * @param[in] Kp               Soft contact model stiffness
   * @param[in] Kv               Soft contact model damping
   * @param[in] oPc              Anchor point of the contact model in WORLD coordinates
   * @param[in] ref              Pinocchio reference frame in which the contact force is to be expressed
   * 
   */
  DifferentialActionModelFreeFwdDynamicsTpl(
      boost::shared_ptr<StateMultibody> state,
      boost::shared_ptr<ActuationModelAbstract> actuation,
      boost::shared_ptr<CostModelSum> costs,
      const pinocchio::FrameIndex frameId,
      const double Kp, 
      const double Kv,
      const Vector3s& oPc,
      const pinocchio::ReferenceFrame ref = pinocchio::LOCAL);
  virtual ~DifferentialActionModelFreeFwdDynamicsTpl();

  /**
   * @brief Compute the system acceleration, and cost value
   *
   * It computes the system acceleration using the free forward-dynamics.
   *
   * @param[in] data  Free forward-dynamics data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the contact dynamics, and cost function
   *
   * @param[in] data  Contact forward-dynamics data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u);
  
  void set_force_cost(const Vector3s& force_des, const Scalar force_weight);

  protected:
    Scalar Kp_; //!< Contact model stiffness
    Scalar Kv_;
    Vector3s oPc_;
    pinocchio::FrameIndex frameId_;
    pinocchio::ReferenceFrame ref_;
    bool with_force_cost_;
    bool active_contact_;
    std::size_t nc_;
    Vector3s force_des_;
    Scalar force_weight_;
    pinocchio::SE3Tpl<Scalar> jMf_;
};

template <typename _Scalar>
struct DifferentialActionDataSoftContact3DFwdDynamicsTpl : public DifferentialActionDataFreeFwdDynamicsTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef DifferentialActionDataAbstractTpl<Scalar> Base;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::Matrix3s Matrix3s;

  template <template <typename Scalar> class Model>
  explicit DifferentialActionDataSoftContact3DFwdDynamicsTpl(Model<Scalar>* const model)
      : Base(model),
        pinocchio(pinocchio::DataTpl<Scalar>(model->get_pinocchio())),
        multibody(&pinocchio, model->get_actuation()->createData()),
        costs(model->get_costs()->createData(&multibody)),
        Minv(model->get_state()->get_nv(), model->get_state()->get_nv()),
        u_drift(model->get_nu()),
        dtau_dx(model->get_nu(), model->get_state()->get_ndx()),
        tmp_xstatic(model->get_state()->get_nx()),
        lJ(6, model->get_state()->get_nv()),
        oJ(6, model->get_state()->get_nv()),
        lv_partial_dq(6, model->get_state()->get_ndx()),
        lv_partial_dv(6, model->get_state()->get_ndx()),
        aba_dq(model->get_state()->get_nv(), model->get_state()->get_ndx()),
        aba_dv(model->get_state()->get_nv(), model->get_state()->get_ndx()),
        aba_dtau(model->get_state()->get_nv(), model->get_state()->get_ndx()),
        df_dx(model->get_nc(), model->get_state()->get_ndx()),
        df_dx_copy(model->get_nc(), model->get_state()->get_ndx()),
        // f(model->get_nc(), model->get_state()->get_ndx()),
        // f_copy(model->get_nc(), model->get_state()->get_ndx()),
        pinForce(pinocchio::ForceTpl<Scalar>::Zero()),
        fext(model->get_state()->get_pinocchio()->njoints, pinocchio::ForceTpl<Scalar>::Zero()),
        fext_copy(model->get_state()->get_pinocchio()->njoints, pinocchio::ForceTpl<Scalar>::Zero()) {
    costs->shareMemory(this);
    Minv.setZero();
    u_drift.setZero();
    dtau_dx.setZero();
    tmp_xstatic.setZero();
    df_dx.setZero();
    df_dx_copy.setZero();
    f.setZero();
    f_copy.setZero();
    oRf.setZero();
    lv.setZero();
    f_residual.setZero();
  }

  pinocchio::DataTpl<Scalar> pinocchio;
  DataCollectorActMultibodyTpl<Scalar> multibody;
  boost::shared_ptr<CostDataSumTpl<Scalar> > costs;
  MatrixXs Minv;
  VectorXs u_drift;
  MatrixXs dtau_dx;
  VectorXs tmp_xstatic;

  Matrix3s oRf;
  Vector3s lv;
  MatrixXs d->lJ;
  MatrixXs d->oJ;
  MatrixXs lv_partial_dq;
  MatrixXs lv_partial_dv;
  MatrixXs aba_dq;
  MatrixXs aba_dv;
  MatrixXs aba_dtau;
  // force cost & derivatives
  Vector3d f_residual;
  MatrixXs df_dx; 
  MatrixXs df_dx_copy;
  Vector3s f;
  Vector3s f_copy;
  pinocchio::ForceTpl<Scalar> pinForce;
  pinocchio::container::aligned_vector<pinocchio::ForceTpl<Scalar> > fext;  //!< External spatial forces in body coordinates (joint level)
  pinocchio::container::aligned_vector<pinocchio::ForceTpl<Scalar> > fext_copy;  //!< External spatial forces in body coordinates (joint level)

  using Base::cost;
  using Base::Fu;
  using Base::Fx;
  using Base::Lu;
  using Base::Luu;
  using Base::Lx;
  using Base::Lxu;
  using Base::Lxx;
  using Base::r;
  using Base::xout;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include <sobec/crocomplements/contact/soft-contact3d-fwddyn.hxx>

#endif  // SOBEC_SOFTCONTACT3D_FWDDYN_HPP_

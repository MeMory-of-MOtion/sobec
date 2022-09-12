///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, CTU, INRIA,
// University of Oxford Copyright note valid unless otherwise stated in
// individual files. All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_SOFTCONTACT3D_AUGMENTED_FWDDYN_HPP_
#define SOBEC_SOFTCONTACT3D_AUGMENTED_FWDDYN_HPP_

#include <stdexcept>

#include "crocoddyl/core/actuation-base.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/diff-action-base.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "sobec/fwd.hpp"

namespace sobec {

/**
 * @brief Differential action model for visco-elastic contact forward dynamics
 * in multibody systems (augmented dynamics including contact force as a state,
 * designed specifically for force feedback MPC)
 *
 * Maths here : https://www.overleaf.com/read/xdpymjfhqqhn
 *
 * \sa `DifferentialActionModelFreeFwdDynamicsTpl`, `calc()`, `calcDiff()`,
 * `createData()`
 */
template <typename _Scalar>
class DAMSoftContact3DAugmentedFwdDynamicsTpl
    : public crocoddyl::DifferentialActionModelFreeFwdDynamicsTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef crocoddyl::DifferentialActionModelFreeFwdDynamicsTpl<Scalar> Base;
  typedef DADSoftContact3DAugmentedFwdDynamicsTpl<Scalar> Data;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef crocoddyl::CostModelSumTpl<Scalar> CostModelSum;
  typedef crocoddyl::StateMultibodyTpl<Scalar> StateMultibody;
  typedef crocoddyl::ActuationModelAbstractTpl<Scalar> ActuationModelAbstract;
  typedef crocoddyl::DifferentialActionDataAbstractTpl<Scalar>
      DifferentialActionDataAbstract;
  typedef crocoddyl::DifferentialActionDataFreeFwdDynamicsTpl<Scalar>
      DifferentialActionDataFreeFwdDynamics;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::Matrix3s Matrix3s;

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
   * @param[in] oPc              Anchor point of the contact model in WORLD
   * coordinates
   * @param[in] ref              Pinocchio reference frame in which the contact
   * force is to be expressed
   *
   */
  DAMSoftContact3DAugmentedFwdDynamicsTpl(
      boost::shared_ptr<StateMultibody> state,
      boost::shared_ptr<ActuationModelAbstract> actuation,
      boost::shared_ptr<CostModelSum> costs,
      const pinocchio::FrameIndex frameId, const double Kp, const double Kv,
      const Vector3s& oPc,
      const pinocchio::ReferenceFrame ref = pinocchio::LOCAL);
  virtual ~DAMSoftContact3DAugmentedFwdDynamicsTpl();

  /**
   * @brief Compute the system acceleration, and cost value
   *
   * It computes the system acceleration using the free forward-dynamics.
   *
   * @param[in] data  Free forward-dynamics data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] f     Force point \f$\mathbf{f}\in\mathbb{R}^{nc}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& f,
      const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the system acceleration, and cost value
   *
   * It computes the system acceleration using the free forward-dynamics.
   *
   * @param[in] data  Free forward-dynamics data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] f     Force point \f$\mathbf{f}\in\mathbb{R}^{nc}\f$
   */
  virtual void calc(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& f);

  /**
   * @brief Compute the derivatives of the contact dynamics, and cost function
   *
   * @param[in] data  Contact forward-dynamics data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] f     Force point \f$\mathbf{f}\in\mathbb{R}^{nc}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& f,
      const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the contact dynamics, and cost function
   *
   * @param[in] data  Contact forward-dynamics data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] f     Force point \f$\mathbf{f}\in\mathbb{R}^{nc}\f$
   */
  virtual void calcDiff(
      const boost::shared_ptr<DifferentialActionDataAbstract>& data,
      const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& f);

  /**
   * @brief Create the soft contact forward-dynamics data
   *
   * @return soft contact forward-dynamics data
   */
  virtual boost::shared_ptr<DifferentialActionDataAbstract> createData();

  void set_force_cost(const Vector3s& force_des, const Scalar force_weight);

  void set_force_des(const Vector3s& inForceDes);

  void set_force_weight(const Scalar inForceWeight);

  void set_Kp(const Scalar inKp);

  void set_Kv(const Scalar inKv);

  void set_oPc(const Vector3s& oPc);

  void set_ref(const pinocchio::ReferenceFrame inRef);

  void set_id(const pinocchio::FrameIndex inId);

  const Scalar get_Kp() const;

  const Scalar get_Kv() const;

  const Vector3s& get_oPc() const;

  const Vector3s& get_force_des() const;

  const Scalar get_force_weight() const;

  const pinocchio::ReferenceFrame& get_ref() const;

  const pinocchio::FrameIndex& get_id() const;

  std::size_t get_nc() { return nc_; };

  // armature
  const VectorXs& get_armature() const;
  void set_armature(const VectorXs& armature);

 protected:
  Scalar Kp_;                       //!< Contact model stiffness
  Scalar Kv_;                       //!< Contact model damping
  Vector3s oPc_;                    //!< Contact model anchor point
  pinocchio::FrameIndex frameId_;   //!< Frame id of the contact
  pinocchio::FrameIndex parentId_;  //!< Parent id of the contact
  pinocchio::ReferenceFrame ref_;   //!< Pinocchio reference frame
  bool with_force_cost_;            //!< Force cost ?
  bool active_contact_;             //!< Active contact ?
  std::size_t nc_;                  //!< Contact model dimension = 3
  Vector3s force_des_;              //!< Desired force 3D
  Scalar force_weight_;             //!< Force cost weight
  pinocchio::SE3Tpl<Scalar>
      jMf_;             //!< Placement of contact frame w.r.t. parent frame
  bool with_armature_;  //!< Indicate if we have defined an armature
  VectorXs armature_;   //!< Armature vector
};

template <typename _Scalar>
struct DADSoftContact3DAugmentedFwdDynamicsTpl
    : public crocoddyl::DifferentialActionDataFreeFwdDynamicsTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef crocoddyl::DifferentialActionDataFreeFwdDynamicsTpl<Scalar> Base;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::Matrix3s Matrix3s;

  template <template <typename Scalar> class Model>
  explicit DADSoftContact3DAugmentedFwdDynamicsTpl(Model<Scalar>* const model)
      : Base(model),
        dtau_dx(model->get_state()->get_nv(), model->get_state()->get_ndx()),
        lJ(6, model->get_state()->get_nv()),
        oJ(6, model->get_state()->get_nv()),
        aba_dq(model->get_state()->get_nv(), model->get_state()->get_nv()),
        aba_dv(model->get_state()->get_nv(), model->get_state()->get_nv()),
        aba_dx(model->get_state()->get_nv(), model->get_state()->get_ndx()),
        aba_dtau(model->get_state()->get_nv(), model->get_state()->get_nv()),
        aba_df(model->get_state()->get_nv(), 3),
        lv_dq(6, model->get_state()->get_nv()),
        lv_dv(6, model->get_state()->get_nv()),
        lv_dx(6, model->get_state()->get_ndx()),
        v_dv(6, model->get_state()->get_nv()),
        a_dq(6, model->get_state()->get_nv()),
        a_dv(6, model->get_state()->get_nv()),
        a_da(6, model->get_state()->get_nv()),
        da_dx(6, model->get_state()->get_ndx()),
        da_du(6, model->get_nu()),
        da_df(6, 3),
        pinForce(pinocchio::ForceTpl<Scalar>::Zero()),
        fext(model->get_pinocchio().njoints,
             pinocchio::ForceTpl<Scalar>::Zero()),
        fext_copy(model->get_pinocchio().njoints,
                  pinocchio::ForceTpl<Scalar>::Zero()),
        dfdt_dx(3, model->get_state()->get_ndx()),
        dfdt_du(3, model->get_nu()),
        dfdt_dx_copy(3, model->get_state()->get_ndx()),
        dfdt_du_copy(3, model->get_nu()),
        JtF(model->get_state()->get_nv(), model->get_state()->get_nv()) {
    costs->shareMemory(this);
    Minv.setZero();
    u_drift.setZero();
    dtau_dx.setZero();
    tmp_xstatic.setZero();
    oRf.setZero();
    lJ.setZero();
    oJ.setZero();
    aba_dq.setZero();
    aba_dv.setZero();
    aba_dx.setZero();
    aba_dtau.setZero();
    aba_df.setZero();
    lv.setZero();
    la.setZero();
    ov.setZero();
    oa.setZero();
    lv_dq.setZero();
    lv_dv.setZero();
    lv_dx.setZero();
    v_dv.setZero();
    a_dq.setZero();
    a_dv.setZero();
    a_da.setZero();
    da_dx.setZero();
    da_du.setZero();
    da_df.setZero();
    f_copy.setZero();
    fout.setZero();
    fout_copy.setZero();
    dfdt_dx.setZero();
    dfdt_du.setZero();
    dfdt_df.setZero();
    dfdt_dx_copy.setZero();
    dfdt_du_copy.setZero();
    dfdt_df_copy.setZero();
    Lf.setZero();
    Lff.setZero();
    f_residual.setZero();
    JtF.setZero();
  }

  using Base::costs;
  using Base::Minv;
  using Base::multibody;
  using Base::pinocchio;
  using Base::u_drift;
  // using Base::dtau_dx;
  using Base::tmp_xstatic;

  MatrixXs dtau_dx;
  // Contact frame rotation and Jacobians
  Matrix3s oRf;
  MatrixXs lJ;
  MatrixXs oJ;
  // Partials of ABA w.r.t. state and control
  MatrixXs aba_dq;
  MatrixXs aba_dv;
  MatrixXs aba_dx;
  MatrixXs aba_dtau;
  MatrixXs aba_df;
  // Frame linear velocity and acceleration in LOCAL and LOCAL_WORLD_ALIGNED
  // frames
  Vector3s lv;
  Vector3s la;
  Vector3s ov;
  Vector3s oa;
  // Partials of frame spatial velocity w.r.t. joint pos, vel, acc
  MatrixXs lv_dq;
  MatrixXs lv_dv;
  MatrixXs lv_dx;
  // Partials of frame spatial acceleration w.r.t. joint pos, vel, acc
  MatrixXs v_dv;
  MatrixXs a_dq;
  MatrixXs a_dv;
  MatrixXs a_da;
  // Partial of frame spatial acc w.r.t. state
  MatrixXs da_dx;
  MatrixXs da_du;
  MatrixXs da_df;
  // Current force and next force
  Vector3s f_copy;
  Vector3s fout;
  Vector3s fout_copy;
  // Spatial wrench due to contact force
  pinocchio::ForceTpl<Scalar> pinForce;
  pinocchio::container::aligned_vector<pinocchio::ForceTpl<Scalar> >
      fext;  //!< External spatial forces in body coordinates (joint level)
  pinocchio::container::aligned_vector<pinocchio::ForceTpl<Scalar> >
      fext_copy;  //!< External spatial forces in body coordinates (joint level)
  // Partial derivatives of next force w.r.t. augmented state
  MatrixXs dfdt_dx;
  MatrixXs dfdt_du;
  Matrix3s dfdt_df;
  MatrixXs dfdt_dx_copy;
  MatrixXs dfdt_du_copy;
  Matrix3s dfdt_df_copy;
  // Partials of cost w.r.t. force
  Vector3s Lf;
  Matrix3s Lff;
  // Force residual for hard coded tracking cost
  Vector3s f_residual;

  MatrixXs JtF;
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
#include <sobec/crocomplements/softcontact/dam3d-augmented.hxx>

#endif  // SOBEC_SOFTCONTACT3D_AUGMENTED_FWDDYN_HPP_

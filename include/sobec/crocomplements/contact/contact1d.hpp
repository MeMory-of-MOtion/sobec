///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_CONTACT_1D_HPP_
#define SOBEC_CONTACT_1D_HPP_

#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/contact-base.hpp>
#include <crocoddyl/multibody/contacts/contact-1d.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/motion.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
namespace newcontacts {

enum Vector3MaskType { x = 0, y = 1, z = 2 };

template <typename _Scalar>
class ContactModel1DTpl : public crocoddyl::ContactModel1DTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef crocoddyl::ContactModel1DTpl<Scalar> Base;
  typedef ContactData1DTpl<Scalar> Data;
  typedef crocoddyl::StateMultibodyTpl<Scalar> StateMultibody;
  typedef typename MathBase::Vector2s Vector2s;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Matrix3s Matrix3s;
  typedef typename MathBase::Matrix6s Matrix6s;
  /**
   * @brief Initialize the 1d contact model
   *
   * @param[in] state  State of the multibody system
   * @param[in] id     Reference frame id of the contact
   * @param[in] xref   Contact position used for the Baumgarte stabilization
   * @param[in] nu     Dimension of the control vector
   * @param[in] gains  Baumgarte stabilization gains
   * @param[in] mask   Constraint 1D axis (default z)
   * @param[in] ref    Reference type of contact
   */
  ContactModel1DTpl(boost::shared_ptr<StateMultibody> state,
                    const pinocchio::FrameIndex id, const Vector3s& xref,
                    const std::size_t nu,
                    const Vector2s& gains = Vector2s::Zero(),
                    const Vector3MaskType& mask = Vector3MaskType::z,
                    const pinocchio::ReferenceFrame ref = pinocchio::LOCAL);

  /**
   * @brief Initialize the 1d contact model
   *
   * The default `nu` is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state  State of the multibody system
   * @param[in] id     Reference frame id of the contact
   * @param[in] xref   Contact position used for the Baumgarte stabilization
   * @param[in] gains  Baumgarte stabilization gains
   * @param[in] ref    Reference type of contact
   */
  ContactModel1DTpl(boost::shared_ptr<StateMultibody> state,
                    const pinocchio::FrameIndex id, const Vector3s& xref,
                    const Vector2s& gains = Vector2s::Zero(),
                    const pinocchio::ReferenceFrame ref = pinocchio::LOCAL);

  virtual ~ContactModel1DTpl();

  /**
   * @brief Compute the 1d contact Jacobian and drift
   *
   * @param[in] data  1d contact data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(
      const boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>& data,
      const Eigen::Ref<const VectorXs>& x);

  /**
   * @brief Compute the derivatives of the 1d contact holonomic constraint
   *
   * @param[in] data  1d contact data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(
      const boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>& data,
      const Eigen::Ref<const VectorXs>& x);

  /**
   * @brief Convert the force into a stack of spatial forces
   *
   * @param[in] data   1d contact data
   * @param[in] force  1d force
   */
  virtual void updateForce(
      const boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>& data,
      const VectorXs& force);

  /**
   * @brief Create the 1d contact data
   */
  virtual boost::shared_ptr<crocoddyl::ContactDataAbstractTpl<Scalar>>
  createData(pinocchio::DataTpl<Scalar>* const data);

  /**
   * @brief Return the reference frame translation
   */
  const Vector3s& get_reference() const;

  /**
   * @brief Create the 1d contact data
   */
  const Vector2s& get_gains() const;

  /**
   * @brief Modify the reference frame translation
   */
  void set_reference(const Vector3s& reference);

  /**
   * @brief Modify pinocchio::ReferenceFrame
   */
  void set_type(const pinocchio::ReferenceFrame type);

  /**
   * @brief Get pinocchio::ReferenceFrame
   */
  const pinocchio::ReferenceFrame get_type() const;

  /**
   * @brief Modify contact 1D mask
   */
  void set_mask(const Vector3MaskType mask);

  /**
   * @brief Get contact 1D mask
   */
  const Vector3MaskType get_mask() const;

  /**
   * @brief Print relevant information of the 1d contact model
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::id_;
  using Base::nc_;
  using Base::nu_;
  using Base::state_;

 private:
  Vector3s xref_;   //!< Contact position used for the Baumgarte stabilization
  Vector2s gains_;  //!< Baumgarte stabilization gains
  Vector3MaskType mask_;            //!< Axis of the 1D contact in (x,y,z)
  pinocchio::ReferenceFrame type_;  //!< Reference type of contact
};

template <typename _Scalar>
struct ContactData1DTpl : public crocoddyl::ContactData1DTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef crocoddyl::ContactData1DTpl<Scalar> Base;
  typedef typename MathBase::Matrix2s Matrix2s;
  typedef typename MathBase::Matrix3s Matrix3s;
  typedef typename MathBase::Matrix6xs Matrix6xs;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::MatrixXs MatrixXs;

  template <template <typename Scalar> class Model>
  ContactData1DTpl(Model<Scalar>* const model,
                   pinocchio::DataTpl<Scalar>* const data)
      : Base(model, data),
        fJf(6, model->get_state()->get_nv()),
        v_partial_dq(6, model->get_state()->get_nv()),
        a_partial_dq(6, model->get_state()->get_nv()),
        a_partial_dv(6, model->get_state()->get_nv()),
        a_partial_da(6, model->get_state()->get_nv()),
        fXjdv_dq(6, model->get_state()->get_nv()),
        fXjda_dq(6, model->get_state()->get_nv()),
        fXjda_dv(6, model->get_state()->get_nv()),
        da0_dx_3d_(3, model->get_state()->get_ndx()),
        drnea_skew_term_(model->get_state()->get_nv(),
                         model->get_state()->get_nv()) {
    frame = model->get_id();
    jMf = model->get_state()->get_pinocchio()->frames[frame].placement;
    fXj = jMf.inverse().toActionMatrix();
    fJf.setZero();
    v_partial_dq.setZero();
    a_partial_dq.setZero();
    a_partial_dv.setZero();
    a_partial_da.setZero();
    fXjdv_dq.setZero();
    fXjda_dq.setZero();
    fXjda_dv.setZero();
    vv.setZero();
    vw.setZero();
    vv_skew.setZero();
    vw_skew.setZero();
    oRf.setZero();
    tmp_skew_.setZero();
    type = model->get_type();
    mask = model->get_mask();
    a0_3d_.setZero();
    da0_dx_3d_.setZero();
    drnea_skew_term_.setZero();
  }

  using Base::a0;
  using Base::da0_dx;
  using Base::df_du;
  using Base::df_dx;
  using Base::f;
  using Base::frame;
  using Base::fXj;
  using Base::Jc;
  using Base::jMf;
  using Base::pinocchio;

  pinocchio::MotionTpl<Scalar> v;
  pinocchio::MotionTpl<Scalar> a;
  Matrix6xs fJf;
  Matrix6xs v_partial_dq;
  Matrix6xs a_partial_dq;
  Matrix6xs a_partial_dv;
  Matrix6xs a_partial_da;
  Matrix6xs fXjdv_dq;
  Matrix6xs fXjda_dq;
  Matrix6xs fXjda_dv;
  Vector3s vv;
  Vector3s vw;
  Matrix3s vv_skew;
  Matrix3s vw_skew;
  Matrix3s oRf;
  Matrix3s tmp_skew_;
  pinocchio::ReferenceFrame type;
  Vector3MaskType mask;
  Vector3s a0_3d_;
  MatrixXs da0_dx_3d_;
  MatrixXs drnea_skew_term_;
};

}  // namespace newcontacts
}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/crocomplements/contact/contact1d.hxx"

#endif  // SOBEC_CONTACT_1D_HPP_

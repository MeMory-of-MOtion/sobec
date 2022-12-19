///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_RESIDUAL_COM_VELOCITY_HPP_
#define SOBEC_RESIDUAL_COM_VELOCITY_HPP_

#include <crocoddyl/core/residual-base.hpp>
#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;

template <typename _Scalar>
struct ResidualDataCoMVelocityTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Matrix3xs Matrix3xs;

  template <template <typename Scalar> class Model>
  ResidualDataCoMVelocityTpl(Model<Scalar>* const model, DataCollectorAbstract* const data)
      : Base(model, data), dvcom_dq(3, model->get_state()->get_nv()) {
    dvcom_dq.setZero();
    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d = dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty(
          "Invalid argument: the shared data should be derived from "
          "DataCollectorMultibody");
    }

    // Avoids data casting at runtime
    pinocchio = d->pinocchio;
  }
  Matrix3xs dvcom_dq;
  pinocchio::DataTpl<Scalar>* pinocchio;  //!< Pinocchio data
  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

/**
 * @brief CoM velocity residual
 *
 * This cost function defines a residual vector as
 * \f$\mathbf{r}=\mathbf{v_{c}}-\mathbf{v_{c}}^*\f$, where
 * \f$\mathbf{v_{c}},\mathbf{v_{c}}^*\in~\mathbb{R}^3\f$ are the current and
 * reference CoM velocity, respetively. Note that the dimension of the residual
 * vector is obtained from 3. Furthermore, the Jacobians of the residual
 * function are computed analytically.
 *
 * As described in `ResidualModelAbstractTpl()`, the residual value and its
 * Jacobians are calculated by `calc` and `calcDiff`, respectively.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModelCoMVelocityTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataCoMVelocityTpl<Scalar> Data;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  /**
   * @brief Initialize the CoM velocity residual model
   *
   * @param[in] state  State of the multibody system
   * @param[in] vref   Reference CoM velocity
   * @param[in] nu     Dimension of the control vector
   */
  ResidualModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& vref, const std::size_t nu);

  /**
   * @brief Initialize the CoM velocity residual model
   *
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state  State of the multibody system
   * @param[in] vref   Reference CoM velocity
   */
  ResidualModelCoMVelocityTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& vref);
  virtual ~ResidualModelCoMVelocityTpl();

  /**
   * @brief Compute the CoM velocity residual
   *
   * @param[in] data  CoM velocity residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the CoM velocity residual
   *
   * @param[in] data  CoM velocity residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);
  virtual boost::shared_ptr<ResidualDataAbstract> createData(DataCollectorAbstract* const data);

  /**
   * @brief Return the CoM velocity reference
   */
  const Vector3s& get_reference() const;

  /**
   * @brief Modify the CoM velocity reference
   */
  void set_reference(const Vector3s& vref);

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  Vector3s vref_;                                      //!< Reference CoM velocity
  typename StateMultibody::PinocchioModel pin_model_;  //!< Pinocchio model used for internal computations
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */

#include "sobec/crocomplements/residual-com-velocity.hxx"

#endif  // SOBEC_RESIDUAL_COM_VELOCITY_HPP_

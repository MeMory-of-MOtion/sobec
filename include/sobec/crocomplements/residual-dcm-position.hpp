///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_RESIDUAL_DCM_POSITION_HPP_
#define CROCODDYL_MULTIBODY_RESIDUAL_DCM_POSITION_HPP_

#include "crocoddyl/core/residual-base.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;

/**
 * @brief DCM position residual
 *
 * This residual function defines the CoM tracking as
 * \f$\mathbf{r}=\mathbf{c}-\mathbf{c}^*\f$, where
 * \f$\mathbf{c},\mathbf{c}^*\in~\mathbb{R}^3\f$ are the current and reference
 * DCM position, respectively. Note that the dimension of the residual vector is
 * obtained from 3. Furthermore, the Jacobians of the residual function are
 * computed analytically.
 *
 * As described in `ResidualModelAbstractTpl()`, the residual value and its
 * Jacobians are calculated by `calc` and `calcDiff`, respectively.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModelDCMPositionTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataDCMPositionTpl<Scalar> Data;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::VectorXs VectorXs;

  /**
   * @brief Initialize the DCM position residual model
   *
   * @param[in] state  State of the multibody system
   * @param[in] cref   Reference CoM position
   * @param[in] nu     Dimension of the control vector
   */
  ResidualModelDCMPositionTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& cref, const double alpha,
                              const std::size_t nu);

  /**
   * @brief Initialize the DCM position residual model
   *
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state  State of the multibody system
   * @param[in] cref   Reference CoM position
   */
  ResidualModelDCMPositionTpl(boost::shared_ptr<StateMultibody> state, const Vector3s& cref, const double alpha);
  virtual ~ResidualModelDCMPositionTpl();

  /**
   * @brief Compute the DCM position residual
   *
   * @param[in] data  DCM position residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the DCM position residual
   *
   * @param[in] data  DCM position residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);
  virtual boost::shared_ptr<ResidualDataAbstract> createData(DataCollectorAbstract* const data);

  /**
   * @brief Return the DCM position reference
   */
  const Vector3s& get_reference() const;

  /**
   * @brief Modify the DCM position reference
   */
  void set_reference(const Vector3s& cref);

  /**
   * @brief Print relevant information of the dcm-position residual
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  Vector3s cref_;                                      //!< Reference DCM position
  double alpha_;                                       //!< Inverse of the time constant for the DCM position
  typename StateMultibody::PinocchioModel pin_model_;  //!< Pinocchio model used for internal computations
};

template <typename _Scalar>
struct ResidualDataDCMPositionTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Matrix3xs Matrix3xs;

  template <template <typename Scalar> class Model>
  ResidualDataDCMPositionTpl(Model<Scalar>* const model, DataCollectorAbstract* const data)
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

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/crocomplements/residual-dcm-position.hxx"

#endif  // SOBEC_RESIDUAL_DCM_POSITION_HPP_

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_RESIDUAL_ANTICIPATED_STATE_HPP_
#define SOBEC_RESIDUAL_ANTICIPATED_STATE_HPP_

#include <crocoddyl/core/residual-base.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/motion.hpp>

#include "sobec/fwd.hpp"

namespace sobec {

using namespace crocoddyl;

/**
 * @brief Anticipated state residual
 *
 * residual = q + dt * v
 *
 * As described in `ResidualModelAbstractTpl()`, the residual value and its
 * Jacobians are calculated by `calc` and `calcDiff`, respectively.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModelAnticipatedStateTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataAnticipatedStateTpl<Scalar> Data;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;

  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  /**
   * @brief Initialize the residual model
   *
   * @param[in] state       State of the multibody system
   * @param[in] nu          Dimension of the control vector
   * link is attached
   */
  ResidualModelAnticipatedStateTpl(boost::shared_ptr<typename Base::StateAbstract> state, 
                                   const std::size_t nu, 
                                   const double& anticipated_time);
  
  ResidualModelAnticipatedStateTpl(boost::shared_ptr<typename Base::StateAbstract> state,
                                   const double& anticipated_time);

  virtual ~ResidualModelAnticipatedStateTpl();

  /**
   * @brief Compute the anticipated state.
   *
   * @param[in] data  residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
                    const Eigen::Ref<const VectorXs> &u);

  /**
   * @brief Compute the derivatives of residual
   *
   * @param[in] data  residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
                        const Eigen::Ref<const VectorXs> &u);

  virtual void print(std::ostream& os) const;

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::unone_;
  
 private:
  double anticipated_time_;  //!< Anticipated time

};

template <typename _Scalar>
struct ResidualDataAnticipatedStateTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;

  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::VectorXs VectorXs;

  template <template <typename Scalar> class Model>
  ResidualDataAnticipatedStateTpl(Model<Scalar> *const model, DataCollectorAbstract *const data) 
      : Base(model, data), 
        xout(model->get_state()->get_ndx()), 
        Jx(model->get_state()->get_ndx(),model->get_state()->get_ndx()) {
    Jx.setZero();
    xout.setZero();
    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d = dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty(
          "Invalid argument: the shared data should be derived from "
          "DataCollectorMultibody");
    }
  }
  
  MatrixXs Jx;
  VectorXs xout;
  
  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/crocomplements/residual-anticipated-state.hxx"

#endif  // SOBEC_RESIDUAL_ANTICIPATED_STATE_HPP_

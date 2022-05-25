///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_CONTACT_FORCE_HPP_
#define SOBEC_CONTACT_FORCE_HPP_

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/residuals/contact-force.hpp"
#include "crocoddyl/core/residual-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/contact-base.hpp"
// #include "crocoddyl/multibody/impulse-base.hpp"
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
// #include "crocoddyl/multibody/contacts/contact-1d.hpp"
// #include "crocoddyl/multibody/contacts/contact-3d.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"
// #include "crocoddyl/multibody/impulses/multiple-impulses.hpp"
// #include "crocoddyl/multibody/impulses/impulse-3d.hpp"
// #include "crocoddyl/multibody/impulses/impulse-6d.hpp"
#include "crocoddyl/multibody/data/contacts.hpp"
// #include "crocoddyl/multibody/data/impulses.hpp"
#include "crocoddyl/core/utils/exception.hpp"

#include "contact3d.hpp"
#include "contact1d.hpp"
#include "multiple-contacts.hpp"

#include "sobec/fwd.hpp"


namespace sobec {

/**
 * @brief Define a contact force residual function
 *
 * This residual function is defined as \f$\mathbf{r}=\boldsymbol{\lambda}-\boldsymbol{\lambda}^*\f$,
 * where \f$\boldsymbol{\lambda}, \boldsymbol{\lambda}^*\f$ are the current and reference spatial forces, respectively.
 * The current spatial forces \f$\boldsymbol{\lambda}\in\mathbb{R}^{nc}\f$ is computed by
 * `DifferentialActionModelContactFwdDynamicsTpl` or `ActionModelImpulseFwdDynamicTpl`, with `nc` as the dimension of
 * the contact.
 *
 * Both residual and residual Jacobians are computed analytically, where the force vector \f$\boldsymbol{\lambda}\f$
 * and its Jacobians \f$\left(\frac{\partial\boldsymbol{\lambda}}{\partial\mathbf{x}},
 * \frac{\partial\boldsymbol{\lambda}}{\partial\mathbf{u}}\right)\f$ are computed by
 * `DifferentialActionModelContactFwdDynamicsTpl` or `ActionModelImpulseFwdDynamicTpl`. These values are stored in a
 * shared data (i.e., `DataCollectorContactTpl` or `DataCollectorImpulseTpl`). Note that this residual function cannot
 * be used with other action models.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`,
 * `DifferentialActionModelContactFwdDynamicsTpl`, `ActionModelImpulseFwdDynamicTpl`, `DataCollectorContactTpl`,
 * `DataCollectorImpulseTpl`
 */
template <typename _Scalar>
class ResidualModelContactForceTpl : public crocoddyl::ResidualModelContactForceTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef crocoddyl::ResidualModelContactForceTpl<Scalar> Base;
  typedef crocoddyl::ResidualDataContactForceTpl<Scalar> Data;
  typedef crocoddyl::StateMultibodyTpl<Scalar> StateMultibody;
  typedef crocoddyl::ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
//   typedef crocoddyl::DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef pinocchio::ForceTpl<Scalar> Force;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  /**
   * @brief Initialize the contact force residual model
   *
   * @param[in] state  Multibody state
   * @param[in] id     Reference frame id
   * @param[in] fref   Reference spatial contact force in the contact coordinates
   * @param[in] nc     Dimension of the contact force (nc <= 6)
   * @param[in] nu     Dimension of control vector
   */
  ResidualModelContactForceTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id,
                               const Force& fref, const std::size_t nc, const std::size_t nu);
                            //    const std::size_t type = 2);

  /**
   * @brief Initialize the contact force residual model
   *
   * The default `nu` is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state  Multibody state
   * @param[in] id     Reference frame id
   * @param[in] fref   Reference spatial contact force in the contact coordinates
   * @param[in] nc     Dimension of the contact force (nc <= 6)
   */
  ResidualModelContactForceTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id,
                               const Force& fref, const std::size_t nc);
  virtual ~ResidualModelContactForceTpl();

  /**
   * @brief Compute the contact force residual
   *
   * The CoP residual is computed based on the \f$\mathbf{A}\f$ matrix, the force vector is computed by
   * `DifferentialActionModelContactFwdDynamicsTpl` or `ActionModelImpulseFwdDynamicTpl` which is stored in
   * `DataCollectorContactTpl` or `DataCollectorImpulseTpl`.
   *
   * @param[in] data  Contact force data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the contact force residual
   *
   * The CoP residual is computed based on the \f$\mathbf{A}\f$ matrix, the force vector is computed by
   * `DifferentialActionModelContactFwdDynamicsTpl` or `ActionModelImpulseFwdDynamicTpl` which is stored in
   * `DataCollectorContactTpl` or `DataCollectorImpulseTpl`.
   *
   * @param[in] data  Contact force data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);

};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/contact-force.hxx"

#endif  // SOBEC_CONTACT_FORCE_HPP_

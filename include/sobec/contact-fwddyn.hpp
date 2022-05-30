///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, CTU, INRIA, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_CONTACT_FWDDYN_HPP_
#define SOBEC_CONTACT_FWDDYN_HPP_

#include <stdexcept>

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/core/diff-action-base.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/core/actuation-base.hpp"
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
// #include "crocoddyl/multibody/data/contacts.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"

#include "sobec/fwd.hpp"

#include "sobec/multiple-contacts.hpp"

namespace sobec {

/**
 * @brief Differential action model for contact forward dynamics in multibody systems.
 *
 * This class implements contact forward dynamics given a stack of rigid-contacts described in
 * `ContactModelMultipleTpl`, i.e.,
 * \f[
 * \left[\begin{matrix}\dot{\mathbf{v}} \\ -\boldsymbol{\lambda}\end{matrix}\right] =
 * \left[\begin{matrix}\mathbf{M} & \mathbf{J}^{\top}_c \\ {\mathbf{J}_{c}} & \mathbf{0} \end{matrix}\right]^{-1}
 * \left[\begin{matrix}\boldsymbol{\tau}_b \\ -\mathbf{a}_0 \\\end{matrix}\right],
 * \f]
 * where \f$\mathbf{q}\in Q\f$, \f$\mathbf{v}\in\mathbb{R}^{nv}\f$ are the configuration point and generalized velocity
 * (its tangent vector), respectively; \f$\boldsymbol{\tau}_b=\boldsymbol{\tau} - \mathbf{h}(\mathbf{q},\mathbf{v})\f$
 * is the bias forces that depends on the torque inputs \f$\boldsymbol{\tau}\f$ and the Coriolis effect and gravity
 * field \f$\mathbf{h}(\mathbf{q},\mathbf{v})\f$; \f$\mathbf{J}_c\in\mathbb{R}^{nc\times nv}\f$ is the contact Jacobian
 * expressed in the local frame; and \f$\mathbf{a}_0\in\mathbb{R}^{nc}\f$ is the desired acceleration in the constraint
 * space. To improve stability in the numerical integration, we define PD gains that are similar in spirit to Baumgarte
 * stabilization: \f[ \mathbf{a}_0 = \mathbf{a}_{\lambda(c)} - \alpha \,^oM^{ref}_{\lambda(c)}\ominus\,^oM_{\lambda(c)}
 * - \beta\mathbf{v}_{\lambda(c)}, \f] where \f$\mathbf{v}_{\lambda(c)}\f$, \f$\mathbf{a}_{\lambda(c)}\f$ are the
 * spatial velocity and acceleration at the parent body of the contact \f$\lambda(c)\f$, respectively; \f$\alpha\f$ and
 * \f$\beta\f$ are the stabilization gains; \f$\,^oM^{ref}_{\lambda(c)}\ominus\,^oM_{\lambda(c)}\f$ is the
 * \f$\mathbb{SE}(3)\f$ inverse composition between the reference contact placement and the current one.
 *
 * The derivatives of the system acceleration and contact forces are computed efficiently
 * based on the analytical derivatives of Recursive Newton Euler Algorithm (RNEA) as described in
 * \cite mastalli-icra20. Note that the algorithm for computing the RNEA derivatives is described in
 * \cite carpentier-rss18.
 *
 * The stack of cost functions is implemented in `CostModelSumTpl`. The computation of the contact dynamics and its
 * derivatives are carrying out inside `calc()` and `calcDiff()` functions, respectively. It is also important to
 * remark that `calcDiff()` computes the derivatives using the latest stored values by `calc()`. Thus, we need to run
 * `calc()` first.
 *
 * \sa `DifferentialActionModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class DifferentialActionModelContactFwdDynamicsTpl
    : public crocoddyl::DifferentialActionModelContactFwdDynamicsTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef crocoddyl::DifferentialActionModelContactFwdDynamicsTpl<Scalar> Base;
  typedef crocoddyl::DifferentialActionDataContactFwdDynamicsTpl<Scalar> Data;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef crocoddyl::CostModelSumTpl<Scalar> CostModelSum;
  typedef crocoddyl::StateMultibodyTpl<Scalar> StateMultibody;
  typedef crocoddyl::ContactModelMultipleTpl<Scalar> crocoContactModelMultiple;
  typedef crocoddyl::ActuationModelAbstractTpl<Scalar> ActuationModelAbstract;
  typedef crocoddyl::DifferentialActionDataAbstractTpl<Scalar> DifferentialActionDataAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  /**
   * @brief Initialize the contact forward-dynamics action model
   *
   * It describes the dynamics evolution of a multibody system under rigid-contact constraints defined by
   * `ContactModelMultipleTpl`. It computes the cost described in `CostModelSumTpl`.
   *
   * @param[in] state            State of the multibody system
   * @param[in] actuation        Actuation model
   * @param[in] contacts         Stack of rigid contact
   * @param[in] costs            Stack of cost functions
   * @param[in] JMinvJt_damping  Damping term used in operational space inertia matrix (default 0.)
   * @param[in] enable_force     Enable the computation of the contact force derivatives (default false)
   */
  DifferentialActionModelContactFwdDynamicsTpl(boost::shared_ptr<StateMultibody> state,
                                               boost::shared_ptr<ActuationModelAbstract> actuation,
                                               boost::shared_ptr<crocoContactModelMultiple> contacts,
                                               boost::shared_ptr<CostModelSum> costs,
                                               const Scalar JMinvJt_damping = Scalar(0.),
                                               const bool enable_force = false);
  virtual ~DifferentialActionModelContactFwdDynamicsTpl();

  /**
   * @brief Compute the derivatives of the contact dynamics, and cost function
   *
   * @param[in] data  Contact forward-dynamics data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<DifferentialActionDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u);

  // /**
  //  * @brief @copydoc Base::quasiStatic()
  //  */
  // virtual void quasiStatic(const boost::shared_ptr<DifferentialActionDataAbstract>& data, Eigen::Ref<VectorXs> u,
  //                          const Eigen::Ref<const VectorXs>& x, const std::size_t maxiter = 100,
  //                          const Scalar tol = Scalar(1e-9));

 private:
  bool enable_force_;
  boost::shared_ptr<sobec::ContactModelMultipleTpl<Scalar>> sobec_contacts_;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include <sobec/contact-fwddyn.hxx>

#endif  // SOBEC_CONTACT_FWDDYN_HPP_

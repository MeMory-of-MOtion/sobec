///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, CTU, INRIA, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/core/utils/math.hpp"

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>

#include "sobec/contact-fwddyn.hpp"

namespace sobec {

template <typename Scalar>
DifferentialActionModelContactFwdDynamicsTpl<Scalar>::DifferentialActionModelContactFwdDynamicsTpl(
    boost::shared_ptr<StateMultibody> state, boost::shared_ptr<ActuationModelAbstract> actuation,
    boost::shared_ptr<crocoContactModelMultiple> contacts, boost::shared_ptr<CostModelSum> costs,
    const Scalar JMinvJt_damping, const bool enable_force)
    : Base(state, actuation, contacts, costs, JMinvJt_damping, false), enable_force_(enable_force) {}


template <typename Scalar>
DifferentialActionModelContactFwdDynamicsTpl<Scalar>::~DifferentialActionModelContactFwdDynamicsTpl() {}



template <typename Scalar>
void DifferentialActionModelContactFwdDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
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
  const std::size_t nc = this->get_contacts()->get_nc();
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(this->get_state()->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(nv);

  Data* d = static_cast<Data*>(data.get());

  // Computing the dynamics derivatives
  // We resize the Kinv matrix because Eigen cannot call block operations recursively:
  // https://eigen.tuxfamily.org/bz/show_bug.cgi?id=408.
  // Therefore, it is not possible to pass d->Kinv.topLeftCorner(nv + nc, nv + nc)
  d->Kinv.resize(nv + nc, nv + nc);
  pinocchio::computeRNEADerivatives(this->get_pinocchio(), d->pinocchio, q, v, d->xout, d->multibody.contacts->fext);
  pinocchio::getKKTContactDynamicMatrixInverse(this->get_pinocchio(), d->pinocchio, d->multibody.contacts->Jc.topRows(nc),
                                               d->Kinv);

  this->get_actuation()->calcDiff(d->multibody.actuation, x, u);
  this->get_contacts()->calcDiff(d->multibody.contacts, x);

  // Add skew term to rnea derivative for contacs expressed in LOCAL_WORLD_ALIGNED
  // see https://www.overleaf.com/read/tzvrrxxtntwk for detailed calculations
  boost::shared_ptr<sobec::ContactModelMultipleTpl<Scalar>> sobec_contacts = boost::static_pointer_cast<sobec::ContactModelMultipleTpl<Scalar>>(this->get_contacts());
  sobec_contacts->updateRneaDerivatives(d->multibody.contacts, d->pinocchio);

  const Eigen::Block<MatrixXs> a_partial_dtau = d->Kinv.topLeftCorner(nv, nv);
  const Eigen::Block<MatrixXs> a_partial_da = d->Kinv.topRightCorner(nv, nc);
  const Eigen::Block<MatrixXs> f_partial_dtau = d->Kinv.bottomLeftCorner(nc, nv);
  const Eigen::Block<MatrixXs> f_partial_da = d->Kinv.bottomRightCorner(nc, nc);

  d->Fx.leftCols(nv).noalias() = -a_partial_dtau * d->pinocchio.dtau_dq;
  d->Fx.rightCols(nv).noalias() = -a_partial_dtau * d->pinocchio.dtau_dv;
  d->Fx.noalias() -= a_partial_da * d->multibody.contacts->da0_dx.topRows(nc);
  d->Fx.noalias() += a_partial_dtau * d->multibody.actuation->dtau_dx;
  d->Fu.noalias() = a_partial_dtau * d->multibody.actuation->dtau_du;

  // Computing the cost derivatives
  if (enable_force_) {
    d->df_dx.topLeftCorner(nc, nv).noalias() = f_partial_dtau * d->pinocchio.dtau_dq;
    d->df_dx.topRightCorner(nc, nv).noalias() = f_partial_dtau * d->pinocchio.dtau_dv;
    d->df_dx.topRows(nc).noalias() += f_partial_da * d->multibody.contacts->da0_dx.topRows(nc);
    d->df_dx.topRows(nc).noalias() -= f_partial_dtau * d->multibody.actuation->dtau_dx;
    d->df_du.topRows(nc).noalias() = -f_partial_dtau * d->multibody.actuation->dtau_du;
    const boost::shared_ptr<MatrixXs> df_dx = boost::make_shared<MatrixXs>(d->df_dx.topRows(nc));
    const boost::shared_ptr<MatrixXs> df_du = boost::make_shared<MatrixXs>(d->df_du.topRows(nc));
    sobec_contacts->updateAccelerationDiff(d->multibody.contacts, d->Fx.bottomRows(nv));
    sobec_contacts->updateForceDiff(d->multibody.contacts, df_dx, df_du);
  }
  this->get_costs()->calcDiff(d->costs, x, u);
}

}  // namespace sobec

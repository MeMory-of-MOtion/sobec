///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/utils/exception.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>

#include "sobec/residual-vel-collision.hpp"

namespace sobec {

using namespace crocoddyl;

template <typename Scalar>
ResidualModelVelCollisionTpl<Scalar>::ResidualModelVelCollisionTpl(
    boost::shared_ptr<StateMultibody> state, const std::size_t nu, boost::shared_ptr<GeometryModel> geom_model,
    const pinocchio::PairIndex pair_id, const pinocchio::FrameIndex frame_id, const pinocchio::ReferenceFrame type,
    const double beta)
    : Base(state, 2, nu, true, false, false),
      pin_model_(state->get_pinocchio()),
      geom_model_(geom_model),
      pair_id_(pair_id),
      joint_id_(pin_model_->frames[frame_id].parent),
      frame_id_(frame_id),
      type_(type),
      beta_(beta) {}

template <typename Scalar>
ResidualModelVelCollisionTpl<Scalar>::~ResidualModelVelCollisionTpl() {}

template <typename Scalar>
void ResidualModelVelCollisionTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract> &data,
                                                const Eigen::Ref<const VectorXs> &x,
                                                const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());

  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  pinocchio::updateGeometryPlacements(*pin_model_.get(), *d->pinocchio, *geom_model_.get(), d->geometry, q);
  pinocchio::computeDistance(*geom_model_.get(), d->geometry, pair_id_);

  // calculate residual
  d->e = d->geometry.distanceResults[pair_id_].nearest_points[0] -
         d->geometry.distanceResults[pair_id_].nearest_points[1];
  d->V = (pinocchio::getFrameVelocity(*pin_model_.get(), *d->pinocchio, frame_id_, type_)).toVector().head(2);
  d->n = d->e.norm() * d->e.norm();
  data->r = d->V / (d->n + beta_);
}

template <typename Scalar>
void ResidualModelVelCollisionTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract> &data,
                                                    const Eigen::Ref<const VectorXs> &,
                                                    const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());

  const std::size_t nv = state_->get_nv();

  // Calculate the vector from the joint jointId to the collision p1, expressed
  // in world frame
  d->d = d->geometry.distanceResults[pair_id_].nearest_points[0] - d->pinocchio->oMi[joint_id_].translation();
  pinocchio::getJointJacobian(*pin_model_.get(), *d->pinocchio, joint_id_, pinocchio::LOCAL_WORLD_ALIGNED, d->J);

  // Get the partial derivatives of the local frame velocity
  pinocchio::getFrameVelocityDerivatives(*pin_model_.get(), *d->pinocchio, frame_id_, type_, d->Vx.leftCols(nv),
                                         d->Vx.rightCols(nv));

  // Calculate the Jacobian at p1
  d->J.template topRows<3>().noalias() += pinocchio::skew(d->d).transpose() * (d->J.template bottomRows<3>());

  // --- Compute the residual derivatives ---
  data->Rx.topLeftCorner(2, nv) =
      -2 / ((d->n + beta_) * (d->n + beta_)) * d->V * d->e.transpose() * d->J.template topRows<3>();
  data->Rx.topLeftCorner(2, nv) += 1 / (d->n + beta_) * d->Vx.leftCols(nv).topRows(2);
  data->Rx.topRightCorner(2, nv) = 1 / (d->n + beta_) * d->Vx.rightCols(nv).topRows(2);
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelVelCollisionTpl<Scalar>::createData(
    DataCollectorAbstract *const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
const pinocchio::GeometryModel &ResidualModelVelCollisionTpl<Scalar>::get_geometry() const {
  return *geom_model_.get();
}

}  // namespace sobec

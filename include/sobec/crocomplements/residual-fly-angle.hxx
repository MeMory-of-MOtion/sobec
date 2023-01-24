///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/utils/exception.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "sobec/crocomplements/residual-fly-angle.hpp"

namespace sobec {
using namespace crocoddyl;
template <typename Scalar>
ResidualModelFlyAngleTpl<Scalar>::ResidualModelFlyAngleTpl(boost::shared_ptr<StateMultibody> state,
                                                           const pinocchio::FrameIndex frame_id, const Scalar slope,
                                                           const Scalar height, const Scalar dist, const Scalar width,
                                                           const std::size_t nu)
    : Base(state, 2, nu, true, true, false),
      frame_id(frame_id),
      slope(slope),
      height(height),
      dist(dist),
      width(width),
      height_offset(0),
      pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelFlyAngleTpl<Scalar>::ResidualModelFlyAngleTpl(boost::shared_ptr<StateMultibody> state,
                                                           const pinocchio::FrameIndex frame_id, const Scalar slope,
                                                           const Scalar height, const Scalar dist, const Scalar width)
    : Base(state, 2, true, true, false),
      frame_id(frame_id),
      slope(slope),
      height(height),
      dist(dist),
      width(width),
      height_offset(0),
      pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelFlyAngleTpl<Scalar>::~ResidualModelFlyAngleTpl() {}

template <typename Scalar>
void ResidualModelFlyAngleTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                            const Eigen::Ref<const VectorXs>& /*x*/,
                                            const Eigen::Ref<const VectorXs>&) {
  // Compute the residual residual give the reference CoM velocity

  Data* d = static_cast<Data*>(data.get());

  pinocchio::updateFramePlacement(pin_model_, *d->pinocchio, frame_id);

  d->sig = 1 / (1 + exp(-width * (d->pinocchio->oMf[frame_id].translation()[0] - dist)));
  d->sig_dt = width * d->sig * (1 - d->sig);
  d->alpha = atan(height * d->sig_dt);

  //d->rotation_alpha.row(0) << cos(d->alpha), 0, sin(d->alpha);
  //d->rotation_alpha.row(1) << 0, 1, 0;
  //d->rotation_alpha.row(2) << -sin(d->alpha), 0, cos(d->alpha);
  d->ez = exp(-slope * (d->pinocchio->oMf[frame_id].translation()[2] - height * d->sig));

  //data->r = (d->rotation_alpha *
  //           pinocchio::getFrameVelocity(pin_model_, *d->pinocchio, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear())
  //              .head(2);
   data->r = pinocchio::getFrameVelocity(pin_model_, *d->pinocchio, frame_id,
                                         pinocchio::LOCAL_WORLD_ALIGNED).linear().head(2);
  data->r *= (d->ez + height_offset);
}

template <typename Scalar>
void ResidualModelFlyAngleTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                const Eigen::Ref<const VectorXs>& /*x*/,
                                                const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  const std::size_t nv = state_->get_nv();

  /* Let' s do a little bit of maths ...
   * r = v/e    with e=exp(z/2)
   * r' = v'/e -  v/e z'/2 = v'/e - r/2 z'
   *
   * Wrong, we should consider l_v the local velocity, with o_v = oRl l_v
   * Then v=R l_v
   *      v' = R l_v' + R' l_v = R l_v' - l_v x Jr
   * Then r' = v'/e - r/2 z' = R l_v'/e - l_v x Jr/e - r/2 z'
   */

  pinocchio::getFrameVelocityDerivatives(pin_model_, *d->pinocchio, frame_id, pinocchio::LOCAL, d->l_dnu_dq,
                                         d->l_dnu_dv);
  const Vector3s& v = pinocchio::getFrameVelocity(pin_model_, *d->pinocchio, frame_id, pinocchio::LOCAL).linear();
  const Matrix3s& R = d->pinocchio->oMf[frame_id].rotation();

  d->sig_ddt = width * d->sig_dt * (1 - 2 * d->sig);
  //d->alpha_dt = height * d->sig_ddt / (1 + (height * d->sig_dt) * (height * d->sig_dt));
  //d->rotation_alpha_dt.row(0) << -sin(d->alpha), 0, cos(d->alpha);
  //d->rotation_alpha_dt.row(1) << 0, 0, 0;
  //d->rotation_alpha_dt.row(2) << -cos(d->alpha), 0, -sin(d->alpha);
  
  // First compute LWA derivatives of the velocity
  d->vxJ.noalias() = pinocchio::skew(-v) * d->l_dnu_dv.template bottomRows<3>();
  d->vxJ += d->l_dnu_dq.template topRows<3>();
  d->o_dv_dq = R * d->vxJ;
  d->o_dv_dv = R * d->l_dnu_dv.template topRows<3>();

  // First term with derivative of v
  //data->Rx.leftCols(nv) = (d->rotation_alpha * d->o_dv_dq).template topRows<2>();
  //data->Rx.rightCols(nv) = (d->rotation_alpha * d->o_dv_dv).template topRows<2>();
  data->Rx.leftCols(nv) = d->o_dv_dq.template topRows<2>();
  data->Rx.rightCols(nv) = d->o_dv_dv.template topRows<2>();
  data->Rx *= d->ez;

  // Second term with derivative of z
  data->Rx.leftCols(nv).row(0) -= data->r[0] * slope * (d->o_dv_dv.row(2) - height * d->sig_dt * d->o_dv_dv.row(0));
  data->Rx.leftCols(nv).row(1) -= data->r[1] * slope * (d->o_dv_dv.row(2) - height * d->sig_dt * d->o_dv_dv.row(0));

  //data->Rx.leftCols(nv).row(0) +=
  //    (d->rotation_alpha_dt *
  //     pinocchio::getFrameVelocity(pin_model_, *d->pinocchio, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear())[0] *
  //    d->ez * d->alpha_dt * d->o_dv_dv.row(0);
  //data->Rx.leftCols(nv).row(1) +=
  //    (d->rotation_alpha_dt *
  //     pinocchio::getFrameVelocity(pin_model_, *d->pinocchio, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear())[1] *
  //    d->ez * d->alpha_dt * d->o_dv_dv.row(0);
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelFlyAngleTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
const typename pinocchio::FrameIndex& ResidualModelFlyAngleTpl<Scalar>::get_frame_id() const {
  return frame_id;
}

template <typename Scalar>
void ResidualModelFlyAngleTpl<Scalar>::set_frame_id(const pinocchio::FrameIndex& fid) {
  frame_id = fid;
}

template <typename Scalar>
void ResidualModelFlyAngleTpl<Scalar>::set_sigmoid(const Scalar& h, const Scalar& d, const Scalar& o) { 
  height = h;
  dist = d; 
  height_offset = o;
}

}  // namespace sobec

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

#include "sobec/crocomplements/residual-fly-high.hpp"

namespace sobec {
using namespace crocoddyl;
template <typename Scalar>
ResidualModelFlyHighTpl<Scalar>::ResidualModelFlyHighTpl(boost::shared_ptr<StateMultibody> state,
                                                         const pinocchio::FrameIndex frame_id, const Scalar slope,
                                                         const std::size_t nu)
    : Base(state, 2, nu, true, true, false), frame_id(frame_id), slope(slope), pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelFlyHighTpl<Scalar>::ResidualModelFlyHighTpl(boost::shared_ptr<StateMultibody> state,
                                                         const pinocchio::FrameIndex frame_id, const Scalar slope)
    : Base(state, 2, true, true, false), frame_id(frame_id), slope(slope), pin_model_(*state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelFlyHighTpl<Scalar>::~ResidualModelFlyHighTpl() {}

template <typename Scalar>
void ResidualModelFlyHighTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                           const Eigen::Ref<const VectorXs>& /*x*/,
                                           const Eigen::Ref<const VectorXs>&) {
  // Compute the residual residual give the reference CoM velocity

  Data* d = static_cast<Data*>(data.get());

  pinocchio::updateFramePlacement(pin_model_, *d->pinocchio, frame_id);
  data->r = pinocchio::getFrameVelocity(pin_model_, *d->pinocchio, frame_id, pinocchio::LOCAL_WORLD_ALIGNED)
                .linear()
                .head(2);
  d->ez = exp(-d->pinocchio->oMf[frame_id].translation()[2] * slope);
  data->r *= d->ez;
}

template <typename Scalar>
void ResidualModelFlyHighTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
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

  // First compute LWA derivatives of the velocity
  d->vxJ.noalias() = pinocchio::skew(-v) * d->l_dnu_dv.template bottomRows<3>();
  d->vxJ += d->l_dnu_dq.template topRows<3>();
  d->o_dv_dq = R * d->vxJ;
  d->o_dv_dv = R * d->l_dnu_dv.template topRows<3>();

  // First term with derivative of v
  data->Rx.leftCols(nv) = d->o_dv_dq.template topRows<2>();
  data->Rx.rightCols(nv) = d->o_dv_dv.template topRows<2>();
  data->Rx *= d->ez;

  // Second term with derivative of z
  data->Rx.leftCols(nv).row(0) -= data->r[0] * slope * d->o_dv_dv.row(2);
  data->Rx.leftCols(nv).row(1) -= data->r[1] * slope * d->o_dv_dv.row(2);
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelFlyHighTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
const typename pinocchio::FrameIndex& ResidualModelFlyHighTpl<Scalar>::get_frame_id() const {
  return frame_id;
}

template <typename Scalar>
void ResidualModelFlyHighTpl<Scalar>::set_frame_id(const pinocchio::FrameIndex& fid) {
  frame_id = fid;
}

}  // namespace sobec

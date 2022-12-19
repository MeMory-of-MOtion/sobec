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

#include "sobec/crocomplements/residual-cop.hpp"

namespace sobec {

using namespace crocoddyl;

template <typename Scalar>
ResidualModelCenterOfPressureTpl<Scalar>::ResidualModelCenterOfPressureTpl(boost::shared_ptr<StateMultibody> state,
                                                                           const pinocchio::FrameIndex contact_id,
                                                                           const std::size_t nu)
    : Base(state, 2, nu, true, true, true), contact_id_(contact_id) {}

template <typename Scalar>
ResidualModelCenterOfPressureTpl<Scalar>::~ResidualModelCenterOfPressureTpl() {}

template <typename Scalar>
void ResidualModelCenterOfPressureTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract> &data,
                                                    const Eigen::Ref<const VectorXs> & /*x*/,
                                                    const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());
  Force f = d->contact->jMf.actInv(d->contact->f);

  if (f.linear()[2] != 0.0) {
    data->r[0] = f.angular()[1] / f.linear()[2];
    data->r[1] = -f.angular()[0] / f.linear()[2];
  } else {
    data->r[0] = 0;
    data->r[1] = 0;
  }
}

template <typename Scalar>
void ResidualModelCenterOfPressureTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract> &data,
                                                        const Eigen::Ref<const VectorXs> &,
                                                        const Eigen::Ref<const VectorXs> &) {
  Data *d = static_cast<Data *>(data.get());
  Force f = d->contact->jMf.actInv(d->contact->f);
  const MatrixXs &df_dx = d->contact->df_dx;
  const MatrixXs &df_du = d->contact->df_du;

  // r = tau/f
  // r'= tau'/f - tau/f^2 f' = (tau'-cop.f')/f
  if (f.linear()[2] != 0.0) {
    data->Rx.row(0) = df_dx.row(4);
    data->Rx.row(1) = -df_dx.row(3);
    data->Rx.row(0) -= data->r[0] * df_dx.row(2);
    data->Rx.row(1) -= data->r[1] * df_dx.row(2);
    data->Rx /= f.linear()[2];

    data->Ru.row(0) = df_du.row(4);
    data->Ru.row(1) = -df_du.row(3);
    data->Ru.row(0) -= data->r[0] * df_du.row(2);
    data->Ru.row(1) -= data->r[1] * df_du.row(2);
    data->Ru /= f.linear()[2];
  } else {
    data->Ru.setZero();
    data->Rx.setZero();
  }
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelCenterOfPressureTpl<Scalar>::createData(
    DataCollectorAbstract *const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

}  // namespace sobec

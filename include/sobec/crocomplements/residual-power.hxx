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

#include "sobec/crocomplements/residual-power.hpp"

namespace sobec {

using namespace crocoddyl;

template <typename Scalar>
ResidualModelPowerTpl<Scalar>::ResidualModelPowerTpl(boost::shared_ptr<StateMultibody> state,
                                                     const std::size_t nu)
    : Base(state, nu, nu, true, true, true) {}

template <typename Scalar>
ResidualModelPowerTpl<Scalar>::~ResidualModelPowerTpl() {}

template <typename Scalar>
void ResidualModelPowerTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract> &data,
                                                    const Eigen::Ref<const VectorXs> &x,
                                                    const Eigen::Ref<const VectorXs> &u) {
  
  Data* d = static_cast<Data*>(data.get());
  
  d->u2 = u.array().cwiseAbs2();
  d->v2 = x.tail(nu_).array().cwiseAbs2();
  data->r = Scalar(0.5) * d->u2.cwiseProduct(d->v2);
}

template <typename Scalar>
void ResidualModelPowerTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract> &data,
                                                        const Eigen::Ref<const VectorXs> &x,
                                                        const Eigen::Ref<const VectorXs> &u) {
  Data* d = static_cast<Data*>(data.get());
  
  data->Rx.rightCols(nu_).diagonal() = d->u2.cwiseProduct(x.tail(nu_));
  data->Ru.diagonal() = d->v2.cwiseProduct(u);
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelPowerTpl<Scalar>::createData(
    DataCollectorAbstract *const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

}  // namespace sobec

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2022, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "contact-force.hpp"

namespace sobec {
namespace newcontacts {

template <typename Scalar>
ResidualModelContactForceTpl<Scalar>::ResidualModelContactForceTpl(boost::shared_ptr<StateMultibody> state,
                                                                   const pinocchio::FrameIndex id, const Force& fref,
                                                                   const std::size_t nc, const std::size_t nu)
    : Base(state, id, fref, nc, nu) {}

template <typename Scalar>
ResidualModelContactForceTpl<Scalar>::ResidualModelContactForceTpl(boost::shared_ptr<StateMultibody> state,
                                                                   const pinocchio::FrameIndex id, const Force& fref,
                                                                   const std::size_t nc)
    : Base(state, id, fref, nc) {}

template <typename Scalar>
ResidualModelContactForceTpl<Scalar>::~ResidualModelContactForceTpl() {}

template <typename Scalar>
void ResidualModelContactForceTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                const Eigen::Ref<const VectorXs>&, const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());

  // We transform the force to the contact frame
  switch (d->contact_type) {
    case Contact1D: {
      ContactData1DTpl<Scalar>* d1d = static_cast<ContactData1DTpl<Scalar>*>(d->contact.get());
      if (d1d->type == pinocchio::LOCAL) {
        data->r = d->contact->jMf.rotation().transpose().row(d1d->mask) * d->contact->f.linear() -
                  this->get_reference().linear().row(d1d->mask);
      } else if (d1d->type == pinocchio::WORLD || d1d->type == pinocchio::LOCAL_WORLD_ALIGNED) {
        data->r = (d1d->oRf * d->contact->jMf.rotation().transpose()).row(d1d->mask) * d->contact->f.linear() -
                  this->get_reference().linear().row(d1d->mask);
      }
      break;
    }
    case Contact3D: {
      ContactData3DTpl<Scalar>* d3d = static_cast<ContactData3DTpl<Scalar>*>(d->contact.get());
      if (d3d->type == pinocchio::LOCAL) {
        data->r = (d->contact->jMf.actInv(d->contact->f).linear() - this->get_reference().linear());
      } else if (d3d->type == pinocchio::WORLD || d3d->type == pinocchio::LOCAL_WORLD_ALIGNED) {
        data->r = (d3d->oRf * d->contact->jMf.actInv(d->contact->f).linear() - this->get_reference().linear());
      }
      break;
    }
    case Contact6D: {
      ContactData6DTpl<Scalar>* d6d = static_cast<ContactData6DTpl<Scalar>*>(d->contact.get());
      if (d6d->type == pinocchio::LOCAL) {
        data->r = (d->contact->jMf.actInv(d->contact->f) - this->get_reference()).toVector();
      } else if (d6d->type == pinocchio::WORLD || d6d->type == pinocchio::LOCAL_WORLD_ALIGNED) {
        data->r = (d6d->lwaMl.act(d->contact->jMf.actInv(d->contact->f)) - this->get_reference()).toVector();
      }
      break;
    }
    default:
      break;
  }
}

template <typename Scalar>
void ResidualModelContactForceTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                    const Eigen::Ref<const VectorXs>&,
                                                    const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());

  const MatrixXs& df_dx = d->contact->df_dx;
  const MatrixXs& df_du = d->contact->df_du;

  switch (d->contact_type) {
    case Contact1D: {
      data->Rx = df_dx.template topRows<1>();
      data->Ru = df_du.template topRows<1>();
      break;
    }
    case Contact3D: {
      data->Rx = df_dx.template topRows<3>();
      data->Ru = df_du.template topRows<3>();
      break;
    }
    case Contact6D:
      data->Rx = df_dx;
      data->Ru = df_du;
      break;
    default:
      break;
  }
}

}  // namespace newcontacts
}  // namespace sobec

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2022, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

namespace sobec {
namespace newcontacts {

template <typename Scalar>
ContactModelMultipleTpl<Scalar>::ContactModelMultipleTpl(
    boost::shared_ptr<StateMultibody> state, const std::size_t nu)
    : Base(state, nu) {}

template <typename Scalar>
ContactModelMultipleTpl<Scalar>::ContactModelMultipleTpl(
    boost::shared_ptr<StateMultibody> state)
    : Base(state) {}

template <typename Scalar>
ContactModelMultipleTpl<Scalar>::~ContactModelMultipleTpl() {}

template <typename Scalar>
void ContactModelMultipleTpl<Scalar>::updateForceDiff(
    const boost::shared_ptr<ContactDataMultiple>& data,
    const boost::shared_ptr<MatrixXs> df_dx,
    const boost::shared_ptr<MatrixXs> df_du) const {
  const std::size_t ndx = this->get_state()->get_ndx();
  if (static_cast<std::size_t>(df_dx->rows()) != this->get_nc() ||
      static_cast<std::size_t>(df_dx->cols()) != ndx) {
    throw_pretty(
        "Invalid argument: " << "df_dx has wrong dimension (it should be " +
                                    std::to_string(this->get_nc()) + "," +
                                    std::to_string(ndx) + ")");
  }
  if (static_cast<std::size_t>(df_du->rows()) != this->get_nc() ||
      static_cast<std::size_t>(df_du->cols()) != this->get_nu()) {
    throw_pretty(
        "Invalid argument: " << "df_du has wrong dimension (it should be " +
                                    std::to_string(this->get_nc()) + "," +
                                    std::to_string(this->get_nu()) + ")");
  }
  if (static_cast<std::size_t>(data->contacts.size()) !=
      this->get_contacts().size()) {
    throw_pretty("Invalid argument: "
                 << "it doesn't match the number of contact datas and models");
  }

  std::size_t nc = 0;
  typename ContactModelContainer::const_iterator it_m, end_m;
  typename ContactDataContainer::const_iterator it_d, end_d;
  for (it_m = this->get_contacts().begin(), end_m = this->get_contacts().end(),
      it_d = data->contacts.begin(), end_d = data->contacts.end();
       it_m != end_m || it_d != end_d; ++it_m, ++it_d) {
    const boost::shared_ptr<ContactItem>& m_i = it_m->second;
    const boost::shared_ptr<ContactDataAbstract>& d_i = it_d->second;
    assert_pretty(it_m->first == it_d->first,
                  "it doesn't match the contact name between data and model");
    if (m_i->active) {
      const std::size_t nc_i = m_i->contact->get_nc();
      m_i->contact->updateForceDiff(d_i, df_dx->block(nc, 0, nc_i, ndx),
                                    df_du->block(nc, 0, nc_i, this->get_nu()));
      nc += nc_i;
    } else {
      m_i->contact->setZeroForceDiff(d_i);
    }
  }
}

template <typename Scalar>
void ContactModelMultipleTpl<Scalar>::updateRneaDerivatives(
    const boost::shared_ptr<ContactDataMultiple>& data,
    pinocchio::DataTpl<Scalar>& pinocchio) const {
  const std::size_t nv = this->get_state()->get_nv();
  if (static_cast<std::size_t>(data->contacts.size()) !=
      this->get_contacts().size()) {
    throw_pretty("Invalid argument: "
                 << "it doesn't match the number of contact datas and models");
  }
  typename ContactModelContainer::const_iterator it_m, end_m;
  typename ContactDataContainer::const_iterator it_d, end_d;
  for (it_m = this->get_contacts().begin(), end_m = this->get_contacts().end(),
      it_d = data->contacts.begin(), end_d = data->contacts.end();
       it_m != end_m || it_d != end_d; ++it_m, ++it_d) {
    const boost::shared_ptr<ContactItem>& m_i = it_m->second;
    const boost::shared_ptr<ContactDataAbstract>& d_i = it_d->second;
    assert_pretty(it_m->first == it_d->first,
                  "it doesn't match the contact name between data and model");
    if (m_i->active) {
      const std::size_t nc_i = m_i->contact->get_nc();
      if (nc_i == 6) {
        ContactModel6DTpl<Scalar>* cm_i =
            static_cast<ContactModel6DTpl<Scalar>*>(m_i->contact.get());
        ContactData6DTpl<Scalar>* cd_i =
            static_cast<ContactData6DTpl<Scalar>*>(d_i.get());
        if (cm_i->get_type() == pinocchio::WORLD ||
            cm_i->get_type() == pinocchio::LOCAL_WORLD_ALIGNED) {
          pinocchio.dtau_dq.block(0, 0, nv, nv) += cd_i->drnea_skew_term_;
        }
      }
      if (nc_i == 3) {
        ContactModel3DTpl<Scalar>* cm_i =
            static_cast<ContactModel3DTpl<Scalar>*>(m_i->contact.get());
        ContactData3DTpl<Scalar>* cd_i =
            static_cast<ContactData3DTpl<Scalar>*>(d_i.get());
        if (cm_i->get_type() == pinocchio::WORLD ||
            cm_i->get_type() == pinocchio::LOCAL_WORLD_ALIGNED) {
          pinocchio.dtau_dq.block(0, 0, nv, nv) += cd_i->drnea_skew_term_;
        }
      }
      if (nc_i == 1) {
        ContactModel1DTpl<Scalar>* cm_i =
            static_cast<ContactModel1DTpl<Scalar>*>(m_i->contact.get());
        ContactData1DTpl<Scalar>* cd_i =
            static_cast<ContactData1DTpl<Scalar>*>(d_i.get());
        if (cm_i->get_type() == pinocchio::WORLD ||
            cm_i->get_type() == pinocchio::LOCAL_WORLD_ALIGNED) {
          pinocchio.dtau_dq.block(0, 0, nv, nv) += cd_i->drnea_skew_term_;
        }
      }
    }
  }
}

// template <typename Scalar>
// MathBase::MatrixXs ContactModelMultipleTpl<Scalar>::rotateJacobians(const
// boost::shared_ptr<MathBase::MatrixXs>& Jin) {
//   MathBase::MatrixXs Jout = MatrixXs(Jin);
//   std::size_t nc = 0;
//   const std::size_t nv = this->get_state()->get_nv();
//   typename ContactModelContainer::const_iterator it_m, end_m;
//   typename ContactDataContainer::const_iterator it_d, end_d;
//   for (it_m = this->get_contacts().begin(), end_m =
//   this->get_contacts().end(), it_d = data->contacts.begin(), end_d =
//   data->contacts.end();
//        it_m != end_m || it_d != end_d; ++it_m, ++it_d) {
//     const boost::shared_ptr<ContactItem>& m_i = it_m->second;
//     const boost::shared_ptr<ContactDataAbstract>& d_i = it_d->second;
//     assert_pretty(it_m->first == it_d->first, "it doesn't match the contact
//     name between data and model"); if (m_i->active) {
//       const std::size_t nc_i = m_i->contact->get_nc();
//       // if (nc_i == 3) {
//       //   ContactModel3DTpl<Scalar>* cm_i =
//       static_cast<ContactModel3DTpl<Scalar>*>(m_i->contact.get());
//       //   ContactData3DTpl<Scalar>* cd_i =
//       static_cast<ContactData3DTpl<Scalar>*>(d_i.get());
//       //   Jout.block(nc, 0, nc_i, nv) = m_i->contact->Jc;
//       // }
//       if (nc_i == 1) {
//         ContactModel1DTpl<Scalar>* cm_i =
//         static_cast<ContactModel1DTpl<Scalar>*>(m_i->contact.get());
//         ContactData1DTpl<Scalar>* cd_i =
//         static_cast<ContactData1DTpl<Scalar>*>(d_i.get()); Jout.block(nc, 0,
//         nc_i, nv) = (d_i->oRf * d_i->fJf.topRows(3)).row(cm_i->get_mask());
//         // d->oRf * d->fJf
//       }
//       nc += nc_i;
//     }
//   }
//   return Jout
// }

}  // namespace newcontacts
}  // namespace sobec

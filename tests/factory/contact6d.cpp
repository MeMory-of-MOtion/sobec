///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "contact6d.hpp"

#include "crocoddyl/core/utils/exception.hpp"
#include "sobec/crocomplements/contact/contact6d.hpp"

namespace sobec {
namespace unittest {

ContactModel6DFactory::ContactModel6DFactory() {}
ContactModel6DFactory::~ContactModel6DFactory() {}

boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModel6DFactory::create(
    PinocchioModelTypes::Type model_type, PinocchioReferenceTypes::Type reference_type, Eigen::Vector2d gains,
    const std::string frame_name, std::size_t nu) const {
  PinocchioModelFactory model_factory(model_type);
  boost::shared_ptr<crocoddyl::StateMultibody> state =
      boost::make_shared<crocoddyl::StateMultibody>(model_factory.create());
  boost::shared_ptr<crocoddyl::ContactModelAbstract> contact;
  std::size_t frame_id = 0;
  if (frame_name == "") {
    frame_id = model_factory.get_frame_id();
  } else {
    frame_id = state->get_pinocchio()->getFrameId(frame_name);
  }
  if (nu == std::numeric_limits<std::size_t>::max()) {
    nu = state->get_nv();
  }
  // std::cout << "created contact6D for frame id = " << frame_name <<
  // std::endl;

  pinocchio::SE3 xref = pinocchio::SE3::Identity();
  if (reference_type == PinocchioReferenceTypes::LOCAL) {
    contact =
        boost::make_shared<sobec::newcontacts::ContactModel6D>(state, frame_id, xref, nu, gains, pinocchio::LOCAL);
  } else if (reference_type == PinocchioReferenceTypes::WORLD) {
    contact =
        boost::make_shared<sobec::newcontacts::ContactModel6D>(state, frame_id, xref, nu, gains, pinocchio::WORLD);
  } else if (reference_type == PinocchioReferenceTypes::LOCAL_WORLD_ALIGNED) {
    contact = boost::make_shared<sobec::newcontacts::ContactModel6D>(state, frame_id, xref, nu, gains,
                                                                     pinocchio::LOCAL_WORLD_ALIGNED);
  }
  return contact;
}

boost::shared_ptr<crocoddyl::ContactModelAbstract> create_random_contact6d() {
  static bool once = true;
  if (once) {
    srand((unsigned)time(NULL));
    once = false;
  }
  boost::shared_ptr<crocoddyl::ContactModelAbstract> contact;
  ContactModel6DFactory factory;
  if (rand() % 3 == 0) {
    contact = factory.create(PinocchioModelTypes::RandomHumanoid, PinocchioReferenceTypes::LOCAL);
  } else if (rand() % 3 == 1) {
    contact = factory.create(PinocchioModelTypes::RandomHumanoid, PinocchioReferenceTypes::LOCAL);
  } else {
    contact = factory.create(PinocchioModelTypes::RandomHumanoid, PinocchioReferenceTypes::LOCAL);
  }
  return contact;
}

}  // namespace unittest
}  // namespace sobec

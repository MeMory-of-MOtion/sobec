///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_CONTACT_3D_FACTORY_HPP_
#define SOBEC_CONTACT_3D_FACTORY_HPP_

#include <iostream>
#include <limits>

#include "crocoddyl/multibody/contact-base.hpp"
#include "crocoddyl/multibody/numdiff/contact.hpp"
#include "state.hpp"

#include "contact1d.hpp"

namespace sobec {
namespace unittest {

class ContactModel3DFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ContactModel3DFactory();
  ~ContactModel3DFactory();

  boost::shared_ptr<crocoddyl::ContactModelAbstract> create(
      PinocchioModelTypes::Type model_type,
      PinocchioReferenceTypes::Type reference_type,
      Eigen::Vector2d gains = Eigen::Vector2d::Zero(),
      const std::string frame_name = std::string(""),
      const std::size_t nu = std::numeric_limits<std::size_t>::max()) const;
};

boost::shared_ptr<crocoddyl::ContactModelAbstract> create_random_contact3D();

}  // namespace unittest
}  // namespace sobec

#endif  // SOBEC_CONTACT_3D_FACTORY_HPP_

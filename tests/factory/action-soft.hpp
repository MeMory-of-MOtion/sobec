///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_ACTION_IAM3D_AUGMENTED_FACTORY_HPP_
#define SOBEC_ACTION_IAM3D_AUGMENTED_FACTORY_HPP_

#include <iterator>

#include "crocoddyl/core/action-base.hpp"
#include "crocoddyl/core/numdiff/action.hpp"

#include "sobec/crocomplements/softcontact/iam3d-augmented.hpp"
#include "statesoft.hpp"
#include "diff-action-soft.hpp"

namespace sobec {
namespace unittest {

struct IAMSoftContactTypes {
  enum Type {
    IAMSoftContact3DAugmented,
    // IAMSoftContact3DAugmented_terminal,
    NbIAMSoftContactTypes
  };
  static std::vector<Type> init_all() {
    std::vector<Type> v;
    v.clear();
    for (int i = 0; i < NbIAMSoftContactTypes; ++i) {
      v.push_back((Type)i);
    }
    return v;
  }
  static const std::vector<Type> all;
};

std::ostream& operator<<(std::ostream& os, IAMSoftContactTypes::Type type);

class IAMSoftContactFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit IAMSoftContactFactory();
  ~IAMSoftContactFactory();

  boost::shared_ptr<sobec::IAMSoftContact3DAugmented> create(
      IAMSoftContactTypes::Type iam_type,
      DAMSoftContactTypes::Type dam_type,
      PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) const;
};

}  // namespace unittest
}  // namespace sobec

#endif  // SOBEC_ACTION_IAM3D_AUGMENTED_FACTORY_HPP_

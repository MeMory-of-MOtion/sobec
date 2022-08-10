///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, University of Edinburgh, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "action-soft.hpp"

#include <crocoddyl/core/utils/exception.hpp>

namespace sobec {
namespace unittest {

const std::vector<IAMSoftContactTypes::Type> IAMSoftContactTypes::all(
    IAMSoftContactTypes::init_all());

std::ostream& operator<<(std::ostream& os, IAMSoftContactTypes::Type type) {
  switch (type) {
    case IAMSoftContactTypes::IAMSoftContact3DAugmented:
      os << "IAMSoftContact3DAugmented";
      break;
    default:
      break;
  }
  return os;
}

IAMSoftContactFactory::IAMSoftContactFactory() {}
IAMSoftContactFactory::~IAMSoftContactFactory() {}

boost::shared_ptr<sobec::IAMSoftContact3DAugmented>
IAMSoftContactFactory::create(IAMSoftContactTypes::Type iam_type,
                              DAMSoftContactTypes::Type dam_type,
                              PinocchioReferenceTypes::Type ref_type) const {
  boost::shared_ptr<sobec::IAMSoftContact3DAugmented> iam;
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> dam =
      DAMSoftContactFactory().create(dam_type, ref_type);
  switch (iam_type) {
    case IAMSoftContactTypes::IAMSoftContact3DAugmented: {
      double time_step = 1e-3;
      bool with_cost_residual = true;
      iam = boost::make_shared<sobec::IAMSoftContact3DAugmented>(
          dam, time_step, with_cost_residual);
      break;
    }

    default:
      throw_pretty(__FILE__ ": Wrong IAMSoftContactTypes::Type given");
      break;
  }
  return iam;
}

}  // namespace unittest
}  // namespace sobec

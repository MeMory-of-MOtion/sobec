///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, University of Edinburgh, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "lpf.hpp"

#include <crocoddyl/core/utils/exception.hpp>

namespace sobec {
namespace unittest {

const std::vector<ActionModelLPFTypes::Type> ActionModelLPFTypes::all(
    ActionModelLPFTypes::init_all());

std::ostream& operator<<(std::ostream& os, ActionModelLPFTypes::Type type) {
  switch (type) {
    case ActionModelLPFTypes::IntegratedActionModelLPF:
      os << "IntegratedActionModelLPF";
      break;
    case ActionModelLPFTypes::NbActionModelLPFTypes:
      os << "NbActionModelLPFTypes";
      break;
    default:
      break;
  }
  return os;
}

ActionModelLPFFactory::ActionModelLPFFactory() {}
ActionModelLPFFactory::~ActionModelLPFFactory() {}

boost::shared_ptr<sobec::IntegratedActionModelLPF>
ActionModelLPFFactory::create(ActionModelLPFTypes::Type iam_type,
                              DifferentialActionModelTypes::Type dam_type,
                              PinocchioReferenceTypes::Type ref_type,
                              ContactModelMaskTypes::Type mask_type) const {
  boost::shared_ptr<sobec::IntegratedActionModelLPF> iam;
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dam =
      DifferentialActionModelFactory().create(dam_type, ref_type, mask_type);
  switch (iam_type) {
    case ActionModelLPFTypes::IntegratedActionModelLPF: {
      double time_step = 1e-6;
      bool with_cost_residual = true;
      double fc = 5;
      bool tau_plus_integration = false;
      int filter = 1;
      bool is_terminal = false;
      iam = boost::make_shared<sobec::IntegratedActionModelLPF>(
          dam, time_step, with_cost_residual, fc, tau_plus_integration, filter,
          is_terminal);
      // set hard-coded costs on unfiltered torque
      double cost_weight_w_reg = 0.02;
      Eigen::VectorXd cost_ref_w_reg = Eigen::VectorXd::Zero(dam->get_nu());
      double cost_weight_w_lim = 1.;
      iam->set_control_reg_cost(cost_weight_w_reg, cost_ref_w_reg);
      iam->set_control_lim_cost(cost_weight_w_lim);
      break;
    }
    default:
      throw_pretty(__FILE__ ": Wrong ActionModelLPFTypes::Type given");
      break;
  }
  return iam;
}

}  // namespace unittest
}  // namespace sobec

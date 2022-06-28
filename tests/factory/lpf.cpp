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
    case ActionModelLPFTypes::IntegratedActionModelLPF_ALL:
      os << "IntegratedActionModelLPF_ALL";
      break;
    case ActionModelLPFTypes::IntegratedActionModelLPF_RAND:
      os << "IntegratedActionModelLPF_RAND";
      break;
    case ActionModelLPFTypes::IntegratedActionModelLPF_NONE:
      os << "IntegratedActionModelLPF_NONE";
      break;
    case ActionModelLPFTypes::IntegratedActionModelLPF_alpha0:
      os << "IntegratedActionModelLPF_alpha0";
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
  // LPFJointMaskType lpf_mask_type) const {
  boost::shared_ptr<sobec::IntegratedActionModelLPF> iam;
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dam =
      DifferentialActionModelFactory().create(dam_type, ref_type, mask_type);
  switch (iam_type) {
    case ActionModelLPFTypes::IntegratedActionModelLPF_ALL: {
      double time_step = 1e-6;
      bool with_cost_residual = true;
      double fc = 5;
      bool tau_plus_integration = false;
      int filter = 1;
      bool is_terminal = false;
      // Select LPF joints
      boost::shared_ptr<crocoddyl::StateMultibody> stateMultibody =
          boost::static_pointer_cast<crocoddyl::StateMultibody>(
              dam->get_state());
      boost::shared_ptr<pinocchio::Model> model =
          stateMultibody->get_pinocchio();
      std::vector<std::string> lpf_joint_names =
          LPFJointListFactory().create_names(model, LPFJointMaskType::ALL);
      iam = boost::make_shared<sobec::IntegratedActionModelLPF>(
          dam, lpf_joint_names, time_step, with_cost_residual, fc,
          tau_plus_integration, filter, is_terminal);
      // set hard-coded costs on unfiltered torque
      double tauReg_weight = 0.02;
      Eigen::VectorXd tauReg_ref = Eigen::VectorXd::Zero(iam->get_ntau());
      double tauLim_weight = 1.;
      iam->set_control_reg_cost(tauReg_weight, tauReg_ref);
      iam->set_control_lim_cost(tauLim_weight);
      break;
    }
    case ActionModelLPFTypes::IntegratedActionModelLPF_RAND: {
      double time_step = 1e-6;
      bool with_cost_residual = true;
      double fc = 5;
      bool tau_plus_integration = false;
      int filter = 1;
      bool is_terminal = false;
      // Select LPF joints
      boost::shared_ptr<crocoddyl::StateMultibody> stateMultibody =
          boost::static_pointer_cast<crocoddyl::StateMultibody>(
              dam->get_state());
      boost::shared_ptr<pinocchio::Model> model =
          stateMultibody->get_pinocchio();
      std::vector<std::string> lpf_joint_names =
          LPFJointListFactory().create_names(model, LPFJointMaskType::RAND);
      iam = boost::make_shared<sobec::IntegratedActionModelLPF>(
          dam, lpf_joint_names, time_step, with_cost_residual, fc,
          tau_plus_integration, filter, is_terminal);
      // set hard-coded costs on unfiltered torque
      double tauReg_weight = 0.02;
      Eigen::VectorXd tauReg_ref = Eigen::VectorXd::Zero(iam->get_ntau());
      double tauLim_weight = 1.;
      iam->set_control_reg_cost(tauReg_weight, tauReg_ref);
      iam->set_control_lim_cost(tauLim_weight);
      break;
    }
    case ActionModelLPFTypes::IntegratedActionModelLPF_NONE: {
      double time_step = 1e-6;
      bool with_cost_residual = true;
      double fc = 5;
      bool tau_plus_integration = false;
      int filter = 1;
      bool is_terminal = false;
      // Select LPF joints
      boost::shared_ptr<crocoddyl::StateMultibody> stateMultibody =
          boost::static_pointer_cast<crocoddyl::StateMultibody>(
              dam->get_state());
      boost::shared_ptr<pinocchio::Model> model =
          stateMultibody->get_pinocchio();
      std::vector<std::string> lpf_joint_names =
          LPFJointListFactory().create_names(model, LPFJointMaskType::NONE);
      iam = boost::make_shared<sobec::IntegratedActionModelLPF>(
          dam, lpf_joint_names, time_step, with_cost_residual, fc,
          tau_plus_integration, filter, is_terminal);
      // set hard-coded costs on unfiltered torque
      double tauReg_weight = 0.02;
      Eigen::VectorXd tauReg_ref = Eigen::VectorXd::Zero(iam->get_ntau());
      double tauLim_weight = 1.;
      iam->set_control_reg_cost(tauReg_weight, tauReg_ref);
      iam->set_control_lim_cost(tauLim_weight);
      break;
    }
    case ActionModelLPFTypes::IntegratedActionModelLPF_alpha0: {
      double time_step = 1e-6;
      bool with_cost_residual = true;
      double alpha = 0.;
      double fc = 50000;
      bool tau_plus_integration = false;
      int filter = 1;
      bool is_terminal = false;
      // Select LPF joints
      boost::shared_ptr<crocoddyl::StateMultibody> stateMultibody =
          boost::static_pointer_cast<crocoddyl::StateMultibody>(
              dam->get_state());
      boost::shared_ptr<pinocchio::Model> model =
          stateMultibody->get_pinocchio();
      std::vector<std::string> lpf_joint_names =
          LPFJointListFactory().create_names(model, LPFJointMaskType::ALL);
      iam = boost::make_shared<sobec::IntegratedActionModelLPF>(
          dam, lpf_joint_names, time_step, with_cost_residual, fc,
          tau_plus_integration, filter, is_terminal);
      // set hard-coded costs on unfiltered torque
      double tauReg_weight = 0.;
      Eigen::VectorXd tauReg_ref = Eigen::VectorXd::Zero(iam->get_ntau());
      double tauLim_weight = 0.;
      iam->set_control_reg_cost(tauReg_weight, tauReg_ref);
      iam->set_control_lim_cost(tauLim_weight);
      iam->set_alpha(alpha);
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

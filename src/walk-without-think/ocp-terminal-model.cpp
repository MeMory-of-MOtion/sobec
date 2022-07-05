///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "sobec/walk-without-think/ocp.hpp"

namespace sobec {

AMA OCPWalk::buildTerminalModel() {
  // Horizon length
  int T = contact_pattern.cols() - 1;

  // assume state and actuation have already been created
  // state = croc.StateMultibody(robot.model)
  // actuation = croc.ActuationModelFloatingBase(state)

  // Contacts
  auto contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state, actuation->get_nu());
  for (int k = 0; k < robot->contactIds.size();
       ++k) {  // k, cid in enumerate(robot.contactIds):
    if (contact_pattern(k, T) == 0.0) continue;
    int cid = robot->contactIds[k];
    auto contact = boost::make_shared<ContactModel6D>(
        state, cid, pinocchio::SE3::Identity(), actuation->get_nu(),
        params->baumgartGains);
    contacts->addContact(robot->model->frames[cid].name + "_contact", contact);
  }

  // Costs
  auto costs = boost::make_shared<CostModelSum>(state, actuation->get_nu());

  auto stateTerminalTarget = robot->x0;
  stateTerminalTarget.head<3>() += params->vcomRef * (T * params->DT);
  auto stateTerminalResidual = boost::make_shared<ResidualModelState>(
      state, stateTerminalTarget, actuation->get_nu());
  auto activation_state = boost::make_shared<ActivationModelWeightedQuad>(
      params->stateTerminalImportance.cwiseProduct(
          params->stateTerminalImportance));
  auto stateTerminalCost = boost::make_shared<CostModelResidual>(
      state, activation_state, stateTerminalResidual);
  costs->addCost("stateReg", stateTerminalCost, params->stateTerminalWeight);

  auto damodel =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state, actuation, contacts, costs, params->kktDamping, true);
  auto termmodel =
      boost::make_shared<IntegratedActionModelEuler>(damodel, params->DT);

  return termmodel;
}

}  // namespace sobec

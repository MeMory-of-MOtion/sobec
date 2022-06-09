///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, University of Edinburgh, CTU, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "diff-action.hpp"

#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "contact3d.hpp"
#include "cost.hpp"
#include "sobec/contact-force.hpp"

namespace sobec {
namespace unittest {

const std::vector<DifferentialActionModelTypes::Type> DifferentialActionModelTypes::all(
    DifferentialActionModelTypes::init_all());

std::ostream& operator<<(std::ostream& os, DifferentialActionModelTypes::Type dam_type) {
  switch (dam_type) {
    case DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm:
      os << "DifferentialActionModelFreeFwdDynamics_TalosArm";
      break;
    case DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm_Squashed:
      os << "DifferentialActionModelFreeFwdDynamics_TalosArm_Squashed";
      break;
    case DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_TalosArm:
      os << "DifferentialActionModelContact1DFwdDynamics_TalosArm";
      break;
    case DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_TalosArm:
      os << "DifferentialActionModelContact3DFwdDynamics_TalosArm";
      break;
    case DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_HyQ:
      os << "DifferentialActionModelContact1DFwdDynamics_HyQ";
      break;
    case DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_HyQ:
      os << "DifferentialActionModelContact3DFwdDynamics_HyQ";
      break;
    case DifferentialActionModelTypes::NbDifferentialActionModelTypes:
      os << "NbDifferentialActionModelTypes";
      break;
    default:
      break;
  }
  return os;
}

DifferentialActionModelFactory::DifferentialActionModelFactory() {}
DifferentialActionModelFactory::~DifferentialActionModelFactory() {}

boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> DifferentialActionModelFactory::create(
    DifferentialActionModelTypes::Type dam_type, PinocchioReferenceTypes::Type ref_type,
    ContactModelMaskTypes::Type mask_type) const {
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> action;
  switch (dam_type) {
    case DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm:
      action =
          create_freeFwdDynamics(StateModelTypes::StateMultibody_TalosArm, ActuationModelTypes::ActuationModelFull);
      break;
    case DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm_Squashed:
      action = create_freeFwdDynamics(StateModelTypes::StateMultibody_TalosArm,
                                      ActuationModelTypes::ActuationModelSquashingFull);
      break;

    // TalosArm state
    case DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_TalosArm:
      action = create_contact3DFwdDynamics(StateModelTypes::StateMultibody_TalosArm,
                                           ActuationModelTypes::ActuationModelFull, ref_type);
      break;
    case DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_TalosArm:
      action = create_contact1DFwdDynamics(StateModelTypes::StateMultibody_TalosArm,
                                           ActuationModelTypes::ActuationModelFull, ref_type, mask_type);
      break;

    case DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_HyQ:
      action = create_contact3DFwdDynamics(StateModelTypes::StateMultibody_HyQ,
                                           ActuationModelTypes::ActuationModelFloatingBase, ref_type);
      break;
    case DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_HyQ:
      action = create_contact1DFwdDynamics(StateModelTypes::StateMultibody_HyQ,
                                           ActuationModelTypes::ActuationModelFloatingBase, ref_type, mask_type);
      break;

    default:
      throw_pretty(__FILE__ ": Wrong DifferentialActionModelTypes::Type given");
      break;
  }
  return action;
}

boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>
DifferentialActionModelFactory::create_freeFwdDynamics(StateModelTypes::Type state_type,
                                                       ActuationModelTypes::Type actuation_type) const {
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> action;
  boost::shared_ptr<crocoddyl::StateMultibody> state;
  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  boost::shared_ptr<crocoddyl::CostModelSum> cost;
  state = boost::static_pointer_cast<crocoddyl::StateMultibody>(StateModelFactory().create(state_type));
  actuation = ActuationModelFactory().create(actuation_type, state_type);
  cost = boost::make_shared<crocoddyl::CostModelSum>(state, actuation->get_nu());
  //   cost->addCost("state",
  //                 CostModelFactory().create(
  //                     CostModelTypes::CostModelResidualState, state_type,
  //                     ActivationModelTypes::ActivationModelQuad),
  //                 1.);
  cost->addCost("control",
                CostModelFactory().create(CostModelTypes::CostModelResidualControl, state_type,
                                          ActivationModelTypes::ActivationModelQuad),
                1.);
  cost->addCost("frame",
                CostModelFactory().create(CostModelTypes::CostModelResidualFramePlacement, state_type,
                                          ActivationModelTypes::ActivationModelQuad),
                1.);
  action = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, cost);
  return action;
}

boost::shared_ptr<sobec::DifferentialActionModelContactFwdDynamics>
DifferentialActionModelFactory::create_contact3DFwdDynamics(StateModelTypes::Type state_type,
                                                            ActuationModelTypes::Type actuation_type,
                                                            PinocchioReferenceTypes::Type ref_type) const {
  boost::shared_ptr<sobec::DifferentialActionModelContactFwdDynamics> action;
  boost::shared_ptr<crocoddyl::StateMultibody> state;
  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  boost::shared_ptr<crocoddyl::ContactModelMultiple> contact;
  boost::shared_ptr<crocoddyl::CostModelSum> cost;
  state = boost::static_pointer_cast<crocoddyl::StateMultibody>(StateModelFactory().create(state_type));
  actuation = ActuationModelFactory().create(actuation_type, state_type);
  contact = boost::make_shared<crocoddyl::ContactModelMultiple>(state, actuation->get_nu());
  cost = boost::make_shared<crocoddyl::CostModelSum>(state, actuation->get_nu());
  pinocchio::Force force = pinocchio::Force::Zero();
  switch (state_type) {
    case StateModelTypes::StateMultibody_TalosArm: {
      contact->addContact(
          "lf",
          ContactModel3DFactory().create(PinocchioModelTypes::TalosArm, ref_type, Eigen::Vector2d::Zero(),
                                         "gripper_left_fingertip_1_link", actuation->get_nu()),
          true);
      // force regularization
      cost->addCost("lf",
                    boost::make_shared<crocoddyl::CostModelResidual>(
                        state, boost::make_shared<sobec::ResidualModelContactForce>(
                                   state, state->get_pinocchio()->getFrameId("gripper_left_fingertip_1_link"), force,
                                   3, actuation->get_nu())),
                    0.1);
      break;
    }
    case StateModelTypes::StateMultibody_HyQ: {
      contact->addContact("lf",
                          ContactModel3DFactory().create(PinocchioModelTypes::HyQ, ref_type, Eigen::Vector2d::Zero(),
                                                         "lf_foot", actuation->get_nu()),
                          true);
      contact->addContact("rf",
                          ContactModel3DFactory().create(PinocchioModelTypes::HyQ, ref_type, Eigen::Vector2d::Zero(),
                                                         "rf_foot", actuation->get_nu()),
                          true);
      contact->addContact("lh",
                          ContactModel3DFactory().create(PinocchioModelTypes::HyQ, ref_type, Eigen::Vector2d::Zero(),
                                                         "lh_foot", actuation->get_nu()),
                          true);
      contact->addContact("rh",
                          ContactModel3DFactory().create(PinocchioModelTypes::HyQ, ref_type, Eigen::Vector2d::Zero(),
                                                         "rh_foot", actuation->get_nu()),
                          true);
      break;
    }
    default:
      throw_pretty(__FILE__ ": Wrong StateModelTypes::Type given");
      break;
  }
  //   cost->addCost(
  //       "state",
  //       CostModelFactory().create(
  //           CostModelTypes::CostModelResidualState, state_type,
  //           ActivationModelTypes::ActivationModelQuad, actuation->get_nu()),
  //       0.1);
  cost->addCost("control",
                CostModelFactory().create(CostModelTypes::CostModelResidualControl, state_type,
                                          ActivationModelTypes::ActivationModelQuad, actuation->get_nu()),
                0.1);
  action =
      boost::make_shared<sobec::DifferentialActionModelContactFwdDynamics>(state, actuation, contact, cost, 0., true);
  return action;
}

boost::shared_ptr<sobec::DifferentialActionModelContactFwdDynamics>
DifferentialActionModelFactory::create_contact1DFwdDynamics(StateModelTypes::Type state_type,
                                                            ActuationModelTypes::Type actuation_type,
                                                            PinocchioReferenceTypes::Type ref_type,
                                                            ContactModelMaskTypes::Type mask_type) const {
  boost::shared_ptr<sobec::DifferentialActionModelContactFwdDynamics> action;
  boost::shared_ptr<crocoddyl::StateMultibody> state;
  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  boost::shared_ptr<crocoddyl::ContactModelMultiple> contact;
  boost::shared_ptr<crocoddyl::CostModelSum> cost;
  state = boost::static_pointer_cast<crocoddyl::StateMultibody>(StateModelFactory().create(state_type));
  actuation = ActuationModelFactory().create(actuation_type, state_type);
  contact = boost::make_shared<crocoddyl::ContactModelMultiple>(state, actuation->get_nu());
  cost = boost::make_shared<crocoddyl::CostModelSum>(state, actuation->get_nu());
  pinocchio::Force force = pinocchio::Force::Zero();
  switch (state_type) {
    case StateModelTypes::StateMultibody_TalosArm: {
      contact->addContact(
          "lf",
          ContactModel1DFactory().create(mask_type, PinocchioModelTypes::TalosArm, ref_type, Eigen::Vector2d::Zero(),
                                         "gripper_left_fingertip_1_link", actuation->get_nu()),
          true);
      // force regularization
      cost->addCost("lf",
                    boost::make_shared<crocoddyl::CostModelResidual>(
                        state, boost::make_shared<sobec::ResidualModelContactForce>(
                                   state, state->get_pinocchio()->getFrameId("gripper_left_fingertip_1_link"), force,
                                   1, actuation->get_nu())),
                    0.1);
      break;
    }
    case StateModelTypes::StateMultibody_HyQ: {
      contact->addContact("lf",
                          ContactModel1DFactory().create(mask_type, PinocchioModelTypes::HyQ, ref_type,
                                                         Eigen::Vector2d::Zero(), "lf_foot", actuation->get_nu()),
                          true);
      contact->addContact("rf",
                          ContactModel1DFactory().create(mask_type, PinocchioModelTypes::HyQ, ref_type,
                                                         Eigen::Vector2d::Zero(), "rf_foot", actuation->get_nu()),
                          true);
      contact->addContact("lh",
                          ContactModel1DFactory().create(mask_type, PinocchioModelTypes::HyQ, ref_type,
                                                         Eigen::Vector2d::Zero(), "lh_foot", actuation->get_nu()),
                          true);
      contact->addContact("rh",
                          ContactModel1DFactory().create(mask_type, PinocchioModelTypes::HyQ, ref_type,
                                                         Eigen::Vector2d::Zero(), "rh_foot", actuation->get_nu()),
                          true);
      break;
    }
    default:
      throw_pretty(__FILE__ ": Wrong StateModelTypes::Type given");
      break;
  }
  //   cost->addCost(
  //       "state",
  //       CostModelFactory().create(
  //           CostModelTypes::CostModelResidualState, state_type,
  //           ActivationModelTypes::ActivationModelQuad, actuation->get_nu()),
  //       0.1);
  cost->addCost("control",
                CostModelFactory().create(CostModelTypes::CostModelResidualControl, state_type,
                                          ActivationModelTypes::ActivationModelQuad, actuation->get_nu()),
                0.1);
  action =
      boost::make_shared<sobec::DifferentialActionModelContactFwdDynamics>(state, actuation, contact, cost, 0., true);
  return action;
}

}  // namespace unittest
}  // namespace sobec

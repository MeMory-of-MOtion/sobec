///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, University of Edinburgh, CTU, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "diff-action-soft.hpp"

#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "contact3d.hpp"
#include "cost.hpp"

namespace sobec {
namespace unittest {

const std::vector<DAMSoftContactTypes::Type>
    DAMSoftContactTypes::all(DAMSoftContactTypes::init_all());

std::ostream& operator<<(std::ostream& os,
                         DAMSoftContactTypes::Type dam_type) {
  switch (dam_type) {
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_TalosArm:
      os << "DAMSoftContact3DAugmentedFwdDynamics_TalosArm";
      break;
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_HyQ:
      os << "DAMSoftContact3DAugmentedFwdDynamics_HyQ";
      break;
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_RandomHumanoid:
      os << "DAMSoftContact3DAugmentedFwdDynamics_RandomHumanoid";
      break;
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_Talos:
      os << "DAMSoftContact3DAugmentedFwdDynamics_Talos";
      break;
    default:
      break;
  }
  return os;
}

DAMSoftContactFactory::DAMSoftContactFactory() {}
DAMSoftContactFactory::~DAMSoftContactFactory() {}

boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics>
DAMSoftContactFactory::create(DAMSoftContactTypes::Type dam_type,
                              PinocchioReferenceTypes::Type ref_type) const {
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> action;
  switch (dam_type) {
    // TalosArm 
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_TalosArm:
      action = create_augmentedDAMSoft3D(
          StateModelTypes::StateMultibody_TalosArm,
          ActuationModelTypes::ActuationModelFull, ref_type);
      break;
    // HyQ  
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_HyQ:
      action = create_augmentedDAMSoft3D(
          StateModelTypes::StateMultibody_HyQ,
          ActuationModelTypes::ActuationModelFloatingBase, ref_type);
      break;
    // RandmHumanoid  
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_RandomHumanoid:
      action = create_augmentedDAMSoft3D(
          StateModelTypes::StateMultibody_RandomHumanoid,
          ActuationModelTypes::ActuationModelFloatingBase, ref_type);
      break;
    // Talos  
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_Talos:
      action = create_augmentedDAMSoft3D(
          StateModelTypes::StateMultibody_Talos,
          ActuationModelTypes::ActuationModelFloatingBase, ref_type);
      break;
    default:
      throw_pretty(__FILE__ ": Wrong DAMSoftContactTypes::Type given");
      break;
  }
  return action;
}


boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics>
DAMSoftContactFactory::create_augmentedDAMSoft3D(StateModelTypes::Type state_type,
                                                 ActuationModelTypes::Type actuation_type,
                                                 PinocchioReferenceTypes::Type ref_type) const {
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> action;
  boost::shared_ptr<crocoddyl::StateMultibody> state;
  boost::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;
  boost::shared_ptr<crocoddyl::ContactModelMultiple> contact;
  boost::shared_ptr<crocoddyl::CostModelSum> cost;
  state = boost::static_pointer_cast<crocoddyl::StateMultibody>(StateModelFactory().create(state_type));
  actuation = ActuationModelFactory().create(actuation_type, state_type);
  cost = boost::make_shared<crocoddyl::CostModelSum>(state, actuation->get_nu());
  std::string frameName = "";

  pinocchio::ReferenceFrame pinRefFrame;
  switch(ref_type){
    case PinocchioReferenceTypes::Type::LOCAL:
      pinRefFrame = pinocchio::LOCAL;
      break;
    case PinocchioReferenceTypes::Type::LOCAL_WORLD_ALIGNED:
      pinRefFrame = pinocchio::LOCAL_WORLD_ALIGNED;
      break;
    case PinocchioReferenceTypes::Type::WORLD:
      pinRefFrame = pinocchio::LOCAL_WORLD_ALIGNED;
      break;
    default:
      throw_pretty(__FILE__ ": Wrong PinocchioReferenceTypes::Type given");
      break;
  }

  switch (state_type) {
    case StateModelTypes::StateMultibody_TalosArm: {
      frameName = "gripper_left_fingertip_1_link";
      break;
    }
    case StateModelTypes::StateMultibody_HyQ: {
      frameName = "rh_haa_joint";
      break;
    }
    case StateModelTypes::StateMultibody_RandomHumanoid: {
      frameName = "rleg6_body";
      break;
    }
    case StateModelTypes::StateMultibody_Talos: {
      frameName = "arm_right_7_link";
      break;
    }
    default:
      throw_pretty(__FILE__ ": Wrong soft contact frame name given");
      break;
  }

  cost->addCost(
      "control",
      CostModelFactory().create(
          CostModelTypes::CostModelResidualControl, state_type,
          ActivationModelTypes::ActivationModelQuad, actuation->get_nu()),
      0.1);
  double Kp = 100;
  double Kv = 10;
  Eigen::Vector3d oPc = Eigen::Vector3d::Zero();
  action = boost::make_shared<sobec::DAMSoftContact3DAugmentedFwdDynamics>(
      state, 
      actuation, 
      cost, 
      state->get_pinocchio()->getFrameId(frameName), 
      Kp, Kv, oPc, pinRefFrame);
  action->set_force_cost(Eigen::Vector3d::Zero(), 0.01);

  return action;
}


}  // namespace unittest
}  // namespace sobec

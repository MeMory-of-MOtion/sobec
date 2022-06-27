///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "statelpf.hpp"

#include <crocoddyl/core/states/euclidean.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <example-robot-data/path.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "state.hpp"

namespace sobec {
namespace unittest {
using namespace crocoddyl;

LPFJointListFactory::LPFJointListFactory() {}
LPFJointListFactory::~LPFJointListFactory() {}
std::vector<std::string> LPFJointListFactory::create_names(
    boost::shared_ptr<pinocchio::Model> model,
    LPFJointMaskType lpf_mask_type) const {
  std::vector<std::string> lpf_joint_names = {};
  switch (lpf_mask_type) {
    case LPFJointMaskType::ALL: {
      for (std::vector<std::string>::iterator iter = model->names.begin();
           iter != model->names.end(); ++iter) {
        if ((int)model->getJointId(*iter) < model->njoints &&
            model->nvs[model->getJointId(*iter)] == 1) {
          lpf_joint_names.push_back(*iter);
        }
      }
      break;
    }
    case LPFJointMaskType::NONE: {
      break;
    }
    case LPFJointMaskType::RAND: {
      int maxJointId(model->njoints - 1), minJointId(2);
      int randJointId = rand() % (maxJointId - minJointId + 1) + minJointId;
      lpf_joint_names.push_back(model->names[randJointId]);
      break;
    }
  }
  return lpf_joint_names;
}

std::vector<int> LPFJointListFactory::create_ids(
    boost::shared_ptr<pinocchio::Model> model,
    LPFJointMaskType lpf_mask_type) const {
  std::vector<int> lpf_joint_ids = {};
  switch (lpf_mask_type) {
    case LPFJointMaskType::ALL: {
      for (std::vector<std::string>::iterator iter = model->names.begin();
           iter != model->names.end(); ++iter) {
        if ((int)model->getJointId(*iter) < model->njoints &&
            model->nvs[model->getJointId(*iter)] == 1) {
          lpf_joint_ids.push_back((int)model->getJointId(*iter));
        }
      }
      break;
    }
    case LPFJointMaskType::NONE: {
      break;
    }
    case LPFJointMaskType::RAND: {
      int maxJointId(model->njoints - 1), minJointId(2);
      int randJointId = rand() % (maxJointId - minJointId + 1) + minJointId;
      lpf_joint_ids.push_back(randJointId);
      break;
    }
  }
  return lpf_joint_ids;
}

const std::vector<StateLPFModelTypes::Type> StateLPFModelTypes::all(
    StateLPFModelTypes::init_all());

std::ostream& operator<<(std::ostream& os, StateLPFModelTypes::Type type) {
  switch (type) {
    case StateLPFModelTypes::StateLPF_TalosArm:
      os << "StateLPF_TalosArm";
      break;
    case StateLPFModelTypes::StateLPF_HyQ:
      os << "StateLPF_HyQ";
      break;
    case StateLPFModelTypes::StateLPF_Talos:
      os << "StateLPF_Talos";
      break;
    case StateLPFModelTypes::StateLPF_RandomHumanoid:
      os << "StateLPF_RandomHumanoid";
      break;
    case StateLPFModelTypes::NbStateLPFModelTypes:
      os << "NbStateLPFModelTypes";
      break;
    default:
      break;
  }
  return os;
}

StateLPFModelFactory::StateLPFModelFactory() {}
StateLPFModelFactory::~StateLPFModelFactory() {}

boost::shared_ptr<sobec::StateLPF> StateLPFModelFactory::create(
    StateLPFModelTypes::Type state_type, LPFJointMaskType lpf_mask_type) const {
  boost::shared_ptr<pinocchio::Model> model;
  boost::shared_ptr<sobec::StateLPF> state;
  switch (state_type) {
    case StateLPFModelTypes::StateLPF_TalosArm: {
      model = PinocchioModelFactory(PinocchioModelTypes::TalosArm).create();
      boost::shared_ptr<crocoddyl::ActuationModelFull> actuation =
          boost::make_shared<crocoddyl::ActuationModelFull>(
              StateModelFactory().create(
                  StateModelTypes::StateMultibody_TalosArm));
      std::vector<int> lpf_joint_ids =
          LPFJointListFactory().create_ids(model, lpf_mask_type);
      state = boost::make_shared<sobec::StateLPF>(model, lpf_joint_ids);

      break;
    }
    case StateLPFModelTypes::StateLPF_HyQ: {
      model = PinocchioModelFactory(PinocchioModelTypes::HyQ).create();
      boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation =
          boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
              boost::static_pointer_cast<crocoddyl::StateMultibody>(
                  StateModelFactory().create(
                      StateModelTypes::StateMultibody_HyQ)));
      std::vector<int> lpf_joint_ids =
          LPFJointListFactory().create_ids(model, lpf_mask_type);
      state = boost::make_shared<sobec::StateLPF>(model, lpf_joint_ids);
      break;
    }
    case StateLPFModelTypes::StateLPF_Talos: {
      model = PinocchioModelFactory(PinocchioModelTypes::Talos).create();
      boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation =
          boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
              boost::static_pointer_cast<crocoddyl::StateMultibody>(
                  StateModelFactory().create(
                      StateModelTypes::StateMultibody_Talos)));
      std::vector<int> lpf_joint_ids =
          LPFJointListFactory().create_ids(model, lpf_mask_type);
      state = boost::make_shared<sobec::StateLPF>(model, lpf_joint_ids);
      break;
    }
    case StateLPFModelTypes::StateLPF_RandomHumanoid: {
      model =
          PinocchioModelFactory(PinocchioModelTypes::RandomHumanoid).create();
      boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation =
          boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
              boost::static_pointer_cast<crocoddyl::StateMultibody>(
                  StateModelFactory().create(
                      StateModelTypes::StateMultibody_RandomHumanoid)));
      std::vector<int> lpf_joint_ids =
          LPFJointListFactory().create_ids(model, lpf_mask_type);
      state = boost::make_shared<sobec::StateLPF>(model, lpf_joint_ids);
      break;
    }
    default:
      throw_pretty(__FILE__ ": Wrong StateLPFModelTypes::Type given");
      break;
  }
  return state;
}

}  // namespace unittest
}  // namespace sobec

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "statesoft.hpp"

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


const std::vector<StateSoftContactModelTypes::Type> StateSoftContactModelTypes::all(
    StateSoftContactModelTypes::init_all());

std::ostream& operator<<(std::ostream& os, StateSoftContactModelTypes::Type type) {
  switch (type) {
    case StateSoftContactModelTypes::StateSoftContact_TalosArm:
      os << "StateSoftContact_TalosArm";
      break;
    case StateSoftContactModelTypes::StateSoftContact_HyQ:
      os << "StateSoftContact_HyQ";
      break;
    case StateSoftContactModelTypes::StateSoftContact_Talos:
      os << "StateSoftContact_Talos";
      break;
    case StateSoftContactModelTypes::StateSoftContact_RandomHumanoid:
      os << "StateSoftContact_RandomHumanoid";
      break;
    case StateSoftContactModelTypes::NbStateSoftContactModelTypes:
      os << "NbStateSoftContactModelTypes";
      break;
    default:
      break;
  }
  return os;
}

StateSoftContactModelFactory::StateSoftContactModelFactory() {}
StateSoftContactModelFactory::~StateSoftContactModelFactory() {}

boost::shared_ptr<sobec::StateSoftContact> StateSoftContactModelFactory::create(
    StateSoftContactModelTypes::Type state_type, std::size_t nc) const {
  boost::shared_ptr<pinocchio::Model> model;
  boost::shared_ptr<sobec::StateSoftContact> state;
  switch (state_type) {
    case StateSoftContactModelTypes::StateSoftContact_TalosArm: {
      model = PinocchioModelFactory(PinocchioModelTypes::TalosArm).create();
      boost::shared_ptr<crocoddyl::ActuationModelFull> actuation = 
          boost::make_shared<crocoddyl::ActuationModelFull>(
              StateModelFactory().create(StateModelTypes::StateMultibody_TalosArm));
      state = boost::make_shared<sobec::StateSoftContact>(model, nc);

      break;
    }
    case StateSoftContactModelTypes::StateSoftContact_HyQ: {
      model = PinocchioModelFactory(PinocchioModelTypes::HyQ).create();
      boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation =
          boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
              boost::static_pointer_cast<crocoddyl::StateMultibody>(
                  StateModelFactory().create(StateModelTypes::StateMultibody_HyQ)));
      state = boost::make_shared<sobec::StateSoftContact>(model, nc);
      break;
    }
    case StateSoftContactModelTypes::StateSoftContact_Talos: {
      model = PinocchioModelFactory(PinocchioModelTypes::Talos).create();
      boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation =
          boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
              boost::static_pointer_cast<crocoddyl::StateMultibody>(
                  StateModelFactory().create(StateModelTypes::StateMultibody_Talos)));
      state = boost::make_shared<sobec::StateSoftContact>(model, nc);
      break;
    }
    case StateSoftContactModelTypes::StateSoftContact_RandomHumanoid: {
      model =
          PinocchioModelFactory(PinocchioModelTypes::RandomHumanoid).create();
      boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation =
          boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
              boost::static_pointer_cast<crocoddyl::StateMultibody>(
                  StateModelFactory().create(StateModelTypes::StateMultibody_RandomHumanoid)));
      state = boost::make_shared<sobec::StateSoftContact>(model, nc);
      break;
    }
    default:
      throw_pretty(__FILE__ ": Wrong StateSoftContactModelTypes::Type given");
      break;
  }
  return state;
}

}  // namespace unittest
}  // namespace sobec

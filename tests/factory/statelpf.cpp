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
    StateLPFModelTypes::Type state_type, bool nu0) const {
  boost::shared_ptr<pinocchio::Model> model;
  boost::shared_ptr<sobec::StateLPF> state;
  switch (state_type) {
    case StateLPFModelTypes::StateLPF_TalosArm: {
      model = PinocchioModelFactory(PinocchioModelTypes::TalosArm).create();
      if (!nu0) {
        boost::shared_ptr<crocoddyl::ActuationModelFull> actuation =
            boost::make_shared<crocoddyl::ActuationModelFull>(
                StateModelFactory().create(
                    StateModelTypes::StateMultibody_TalosArm));
        state = boost::make_shared<sobec::StateLPF>(model, actuation->get_nu());
      } else {
        state = boost::make_shared<sobec::StateLPF>(model, 0);
      }
      break;
    }
    case StateLPFModelTypes::StateLPF_HyQ: {
      model = PinocchioModelFactory(PinocchioModelTypes::HyQ).create();
      if (!nu0) {
        boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation =
            boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
                boost::static_pointer_cast<crocoddyl::StateMultibody>(
                    StateModelFactory().create(
                        StateModelTypes::StateMultibody_HyQ)));
        state = boost::make_shared<sobec::StateLPF>(model, actuation->get_nu());
      } else {
        state = boost::make_shared<sobec::StateLPF>(model, 0);
      }
      break;
    }
    case StateLPFModelTypes::StateLPF_Talos: {
      model = PinocchioModelFactory(PinocchioModelTypes::Talos).create();
      if (!nu0) {
        boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation =
            boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
                boost::static_pointer_cast<crocoddyl::StateMultibody>(
                    StateModelFactory().create(
                        StateModelTypes::StateMultibody_Talos)));
        state = boost::make_shared<sobec::StateLPF>(model, actuation->get_nu());
      } else {
        state = boost::make_shared<sobec::StateLPF>(model, 0);
      }
      break;
    }
    case StateLPFModelTypes::StateLPF_RandomHumanoid: {
      model =
          PinocchioModelFactory(PinocchioModelTypes::RandomHumanoid).create();
      if (!nu0) {
        boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation =
            boost::make_shared<crocoddyl::ActuationModelFloatingBase>(
                boost::static_pointer_cast<crocoddyl::StateMultibody>(
                    StateModelFactory().create(
                        StateModelTypes::StateMultibody_RandomHumanoid)));
        state = boost::make_shared<sobec::StateLPF>(model, actuation->get_nu());
      } else {
        state = boost::make_shared<sobec::StateLPF>(model, 0);
      }
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

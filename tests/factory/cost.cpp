///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "cost.hpp"

#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/control-gravity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>

#include "sobec/crocomplements/residual-2D-surface.hpp"
#include "sobec/crocomplements/residual-com-velocity.hpp"
#include "sobec/crocomplements/residual-dcm-position.hpp"
#include "sobec/crocomplements/residual-fly-angle.hpp"
#include "sobec/crocomplements/residual-fly-high.hpp"
#include "sobec/crocomplements/residual-anticipated-state.hpp"
#include "sobec/crocomplements/residual-power.hpp"
// #include "crocoddyl/multibody/residuals/centroidal-momentum.hpp"
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/residuals/contact-friction-cone.hpp>
#include <crocoddyl/multibody/residuals/contact-wrench-cone.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>

namespace sobec {
namespace unittest {
using namespace crocoddyl;

const std::vector<CostModelTypes::Type> CostModelTypes::all(CostModelTypes::init_all());
const std::vector<CostModelNoFFTypes::Type> CostModelNoFFTypes::all(CostModelNoFFTypes::init_all());

std::ostream& operator<<(std::ostream& os, CostModelTypes::Type type) {
  switch (type) {
      // case CostModelTypes::CostModelResidualState:
      //   os << "CostModelResidualState";
      //   break;
    case CostModelTypes::CostModelResidualControl:
      os << "CostModelResidualControl";
      break;
    case CostModelTypes::CostModelResidualCoMPosition:
      os << "CostModelResidualCoMPosition";
      break;
    case CostModelTypes::CostModelResidualCoMVelocity:
      os << "CostModelResidualCoMVelocity";
      break;
      // case CostModelTypes::CostModelResidualFlyHigh:
      //    os << "CostModelResidualFlyHigh";
      //    break;
      // case CostModelTypes::CostModelResidualFlyAngle:
      //    os << "CostModelResidualFlyAngle";
      //    break;
    case CostModelTypes::CostModelResidualPower:
      os << "CostModelResidualPower";
      break;
    case CostModelTypes::CostModelResidualAnticipatedState:
      os << "CostModelResidualAnticipatedState";
      break;
    case CostModelTypes::CostModelResidual2DSurface:
      os << "CostModelResidual2DSurface";
      break;
    case CostModelTypes::CostModelResidualFramePlacement:
      os << "CostModelResidualFramePlacement";
      break;
    case CostModelTypes::CostModelResidualFrameRotation:
      os << "CostModelResidualFrameRotation";
      break;
    case CostModelTypes::CostModelResidualFrameTranslation:
      os << "CostModelResidualFrameTranslation";
      break;
    // case CostModelTypes::CostModelResidualDCMPosition:
    //   os << "CostModelResidualDCMPosition";
    //   break;
    case CostModelTypes::CostModelResidualFrameVelocity:
      os << "CostModelResidualFrameVelocity";
      break;
    case CostModelTypes::NbCostModelTypes:
      os << "NbCostModelTypes";
      break;
    default:
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, CostModelNoFFTypes::Type type) {
  switch (type) {
    case CostModelNoFFTypes::CostModelResidualControlGrav:
      os << "CostModelResidualControlGrav";
      break;
    case CostModelNoFFTypes::NbCostModelNoFFTypes:
      os << "NbCostModelNoFFTypes";
      break;
    default:
      break;
  }
  return os;
}

CostModelFactory::CostModelFactory() {}
CostModelFactory::~CostModelFactory() {}

boost::shared_ptr<crocoddyl::CostModelAbstract> CostModelFactory::create(CostModelTypes::Type cost_type,
                                                                         StateModelTypes::Type state_type,
                                                                         ActivationModelTypes::Type activation_type,
                                                                         std::size_t nu) const {
  StateModelFactory state_factory;
  ActivationModelFactory activation_factory;
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost;
  boost::shared_ptr<crocoddyl::StateMultibody> state =
      boost::static_pointer_cast<crocoddyl::StateMultibody>(state_factory.create(state_type));
  double alpha = fabs(Eigen::VectorXd::Random(1)[0]);
  double beta = fabs(Eigen::VectorXd::Random(1)[0]);
  double gamma = fabs(Eigen::VectorXd::Random(1)[0]);
  crocoddyl::FrameIndex frame_index = state->get_pinocchio()->frames.size() - 1;
  pinocchio::SE3 frame_SE3 = pinocchio::SE3::Random();
  if (nu == std::numeric_limits<std::size_t>::max()) {
    nu = state->get_nv();
  }
  switch (cost_type) {
    // case CostModelTypes::CostModelResidualState:
    //   cost = boost::make_shared<crocoddyl::CostModelResidual>(
    //       state, activation_factory.create(activation_type,
    //       state->get_ndx()),
    //       boost::make_shared<crocoddyl::ResidualModelState>(state,
    //                                                         state->rand(),
    //                                                         nu));
    //   break;
    case CostModelTypes::CostModelResidualControl:
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, nu),
          boost::make_shared<crocoddyl::ResidualModelControl>(state, Eigen::VectorXd::Random(nu)));
      break;
    case CostModelTypes::CostModelResidualCoMPosition:
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, 3),
          boost::make_shared<crocoddyl::ResidualModelCoMPosition>(state, Eigen::Vector3d::Random(), nu));
      break;
    case CostModelTypes::CostModelResidualCoMVelocity:
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, 3),
          boost::make_shared<sobec::ResidualModelCoMVelocity>(state, Eigen::Vector3d::Random(), nu));
      break;
    // case CostModelTypes::CostModelResidualFlyHigh: {
    //   cost = boost::make_shared<crocoddyl::CostModelResidual>(
    //       state, activation_factory.create(activation_type, 2),
    //       boost::make_shared<sobec::ResidualModelFlyHigh>(state, frame_index,
    //       1, nu));
    //   break;
    // }
    // case CostModelTypes::CostModelResidualFlyAngle: {
    //   cost = boost::make_shared<crocoddyl::CostModelResidual>(
    //       state, activation_factory.create(activation_type, 2),
    //       boost::make_shared<sobec::ResidualModelFlyAngle>(state,
    //       frame_index, 1, alpha,
    //                                                       beta, gamma,nu));
    //   break;
    // }
    case CostModelTypes::CostModelResidualPower:
       cost = boost::make_shared<crocoddyl::CostModelResidual>(
           state, activation_factory.create(activation_type, nu),
           boost::make_shared<sobec::ResidualModelPower>(state,nu));
       break;
    case CostModelTypes::CostModelResidualAnticipatedState: 
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, state->get_nv()),
          boost::make_shared<sobec::ResidualModelAnticipatedState>(state, nu, alpha));
      break;
    case CostModelTypes::CostModelResidual2DSurface: 
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, 2),
          boost::make_shared<sobec::ResidualModel2DSurface>(state, frame_index, Eigen::Vector2d::Random(), 0.2, 0.2,
                                                            0.1, nu));
      break;
    case CostModelTypes::CostModelResidualFramePlacement:
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, 6),
          boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state, frame_index, frame_SE3, nu));
      break;
    case CostModelTypes::CostModelResidualFrameRotation:
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, 3),
          boost::make_shared<crocoddyl::ResidualModelFrameRotation>(state, frame_index, frame_SE3.rotation(), nu));
      break;
    case CostModelTypes::CostModelResidualFrameTranslation:
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, 3),
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state, frame_index, frame_SE3.translation(),
                                                                       nu));
      break;
    case CostModelTypes::CostModelResidualDCMPosition:
       cost = boost::make_shared<crocoddyl::CostModelResidual>(
           state, activation_factory.create(activation_type, 3),
           boost::make_shared<sobec::ResidualModelDCMPosition>(state,
           Eigen::Vector3d::Random(), alpha, nu));
       break;
    case CostModelTypes::CostModelResidualFrameVelocity:
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, 6),
          boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(state, frame_index, pinocchio::Motion::Random(),
                                                                    pinocchio::ReferenceFrame::LOCAL, nu));
      break;
    default:
      throw_pretty(__FILE__ ": Wrong CostModelTypes::Type given");
      break;
  }
  return cost;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> CostModelFactory::create(CostModelNoFFTypes::Type cost_type,
                                                                         ActivationModelTypes::Type activation_type,
                                                                         std::size_t nu) const {
  StateModelFactory state_factory;
  ActivationModelFactory activation_factory;
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost;
  boost::shared_ptr<crocoddyl::StateMultibody> state = boost::static_pointer_cast<crocoddyl::StateMultibody>(
      state_factory.create(StateModelTypes::StateMultibody_TalosArm));
  if (nu == std::numeric_limits<std::size_t>::max()) {
    nu = state->get_nv();
  }

  switch (cost_type) {
    case CostModelNoFFTypes::CostModelResidualControlGrav:
      cost = boost::make_shared<crocoddyl::CostModelResidual>(
          state, activation_factory.create(activation_type, static_cast<Eigen::Index>(state->get_nv())),
          boost::make_shared<ResidualModelControlGrav>(state, nu));
      break;
    default:
      throw_pretty(__FILE__ ": Wrong CostModelTypes::Type given");
      break;
  }
  return cost;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> create_random_cost(StateModelTypes::Type state_type, std::size_t nu) {
  static bool once = true;
  if (once) {
    srand((unsigned)time(NULL));
    once = false;
  }

  CostModelFactory factory;
  CostModelTypes::Type rand_type = static_cast<CostModelTypes::Type>(rand() % CostModelTypes::NbCostModelTypes);
  return factory.create(rand_type, state_type, ActivationModelTypes::ActivationModelQuad, nu);
}

}  // namespace unittest
}  // namespace sobec

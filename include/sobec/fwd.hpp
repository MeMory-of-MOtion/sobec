///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_FWD_HPP_
#define SOBEC_FWD_HPP_

#include <pinocchio/fwd.hpp>

#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/quadratic-flat-log.hpp>
#include <crocoddyl/core/activations/smooth-1norm.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solver-base.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/contacts/contact-6d.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/frames.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/contact-control-gravity.hpp>
#include <crocoddyl/multibody/residuals/contact-force.hpp>
#include <crocoddyl/multibody/residuals/contact-wrench-cone.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "sobec/crocomplements/activation-quad-ref.hpp"
#include "sobec/crocomplements/residual-com-velocity.hpp"

namespace sobec {

// Cost COM-vel
template <typename Scalar>
class ResidualModelCoMVelocityTpl;
template <typename Scalar>
struct ResidualDataCoMVelocityTpl;
typedef ResidualModelCoMVelocityTpl<double> ResidualModelCoMVelocity;
typedef ResidualDataCoMVelocityTpl<double> ResidualDataCoMVelocity;

// Cost COP
template <typename Scalar>
class ResidualModelCenterOfPressureTpl;
template <typename Scalar>
struct ResidualDataCenterOfPressureTpl;
typedef ResidualModelCenterOfPressureTpl<double> ResidualModelCenterOfPressure;
typedef ResidualDataCenterOfPressureTpl<double> ResidualDataCenterOfPressure;

// Cost CenterOfFriction
template <typename Scalar>
class ResidualModelCenterOfFrictionTpl;
template <typename Scalar>
struct ResidualDataCenterOfFrictionTpl;
typedef ResidualModelCenterOfFrictionTpl<double> ResidualModelCenterOfFriction;
typedef ResidualDataCenterOfFrictionTpl<double> ResidualDataCenterOfFriction;

// Cost velocity collision
template <typename Scalar>
class ResidualModelVelCollisionTpl;
template <typename Scalar>
struct ResidualDataVelCollisionTpl;
typedef ResidualModelVelCollisionTpl<double> ResidualModelVelCollision;
typedef ResidualDataVelCollisionTpl<double> ResidualDataVelCollision;

// Cost fly high
template <typename Scalar>
class ResidualModelFlyAngleTpl;
template <typename Scalar>
struct ResidualDataFlyAngleTpl;
typedef ResidualModelFlyAngleTpl<double> ResidualModelFlyAngle;
typedef ResidualDataFlyAngleTpl<double> ResidualDataFlyAngle;

// Cost fly angle
template <typename Scalar>
class ResidualModelFlyHighTpl;
template <typename Scalar>
struct ResidualDataFlyHighTpl;
typedef ResidualModelFlyHighTpl<double> ResidualModelFlyHigh;
typedef ResidualDataFlyHighTpl<double> ResidualDataFlyHigh;

// Cost DCM
template <typename Scalar>
class ResidualModelDCMPositionTpl;
template <typename Scalar>
struct ResidualDataDCMPositionTpl;
typedef ResidualModelDCMPositionTpl<double> ResidualModelDCMPosition;
typedef ResidualDataDCMPositionTpl<double> ResidualDataDCMPosition;

// Cost feet collision
template <typename Scalar>
class ResidualModelFeetCollisionTpl;
template <typename Scalar>
struct ResidualDataFeetCollisionTpl;
typedef ResidualModelFeetCollisionTpl<double> ResidualModelFeetCollision;
typedef ResidualDataFeetCollisionTpl<double> ResidualDataFeetCollision;

// Cost 2D surface
template <typename Scalar>
class ResidualModel2DSurfaceTpl;
template <typename Scalar>
struct ResidualData2DSurfaceTpl;
typedef ResidualModel2DSurfaceTpl<double> ResidualModel2DSurface;
typedef ResidualData2DSurfaceTpl<double> ResidualData2DSurface;

// Activation quad-ref
template <typename Scalar>
class ActivationModelQuadRefTpl;
typedef ActivationModelQuadRefTpl<double> ActivationModelQuadRef;
typedef boost::shared_ptr<ActivationModelQuadRef> ActivationModelQuadRefPtr;

// Activation weighted-log
template <typename Scalar>
class ActivationModelWeightedLogTpl;
template <typename Scalar>
struct ActivationDataWeightedLogTpl;
typedef ActivationModelWeightedLogTpl<double> ActivationModelWeightedLog;
typedef ActivationDataWeightedLogTpl<double> ActivationDataWeightedLog;
typedef boost::shared_ptr<ActivationModelWeightedLog> ActivationModelWeightedLogPtr;

typedef Eigen::Matrix<double, 6, 1> eVector6;
typedef Eigen::Matrix<double, 4, 1> eVector4;
typedef Eigen::Vector3d eVector3;
typedef Eigen::Vector2d eVector2;
typedef boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> IAM;
typedef boost::shared_ptr<crocoddyl::IntegratedActionDataEuler> IAD;
typedef boost::shared_ptr<crocoddyl::ActionModelAbstract> AMA;
typedef boost::shared_ptr<crocoddyl::ActionDataAbstract> ADA;
typedef boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics> DAM;
typedef boost::shared_ptr<crocoddyl::DifferentialActionDataContactFwdDynamics> DAD;
typedef boost::shared_ptr<crocoddyl::CostModelSum> Cost;
typedef boost::shared_ptr<crocoddyl::ContactModelMultiple> Contact;
typedef boost::shared_ptr<crocoddyl::SolverFDDP> DDP;

typedef boost::shared_ptr<crocoddyl::ResidualModelFramePlacement> ResidualModelFramePlacementPtr;
typedef boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone> ResidualModelContactWrenchConePtr;

// State LPF
template <typename Scalar>
class StateLPFTpl;
typedef StateLPFTpl<double> StateLPF;

// IAM LPF
template <typename Scalar>
class IntegratedActionModelLPFTpl;
typedef IntegratedActionModelLPFTpl<double> IntegratedActionModelLPF;
template <typename Scalar>
class IntegratedActionDataLPFTpl;
typedef IntegratedActionDataLPFTpl<double> IntegratedActionDataLPF;

// OCP
class OCPWalkParam;
class OCPWalk;
// MPC
class MPCWalk;

}  // namespace sobec

namespace sobec {
namespace newcontacts {

// contact 6D
template <typename Scalar>
class ContactModel6DTpl;
typedef ContactModel6DTpl<double> ContactModel6D;
template <typename Scalar>
class ContactData6DTpl;
typedef ContactData6DTpl<double> ContactData6D;

// contact 3D
template <typename Scalar>
class ContactModel3DTpl;
typedef ContactModel3DTpl<double> ContactModel3D;
template <typename Scalar>
class ContactData3DTpl;
typedef ContactData3DTpl<double> ContactData3D;

// contact 1D
template <typename Scalar>
class ContactModel1DTpl;
typedef ContactModel1DTpl<double> ContactModel1D;
template <typename Scalar>
class ContactData1DTpl;
typedef ContactData1DTpl<double> ContactData1D;

// multiple contacts
template <typename Scalar>
class ContactModelMultipleTpl;
typedef ContactModelMultipleTpl<double> ContactModelMultiple;

// DAM contact fwd dynamics
template <typename Scalar>
class DifferentialActionModelContactFwdDynamicsTpl;
typedef DifferentialActionModelContactFwdDynamicsTpl<double> DifferentialActionModelContactFwdDynamics;

// Residual contact force
template <typename Scalar>
class ResidualModelContactForceTpl;
typedef ResidualModelContactForceTpl<double> ResidualModelContactForce;

enum ContactType { ContactUndefined, Contact1D, Contact2D, Contact3D, Contact6D };

}  // namespace newcontacts
}  // namespace sobec

#endif  // SOBEC_FWD_HPP_

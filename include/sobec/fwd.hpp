///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_FWD_HPP_
#define SOBEC_FWD_HPP_

#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>

namespace sobec {

// Cost COM-vel
template <typename Scalar>
class ResidualModelCoMVelocityTpl;
template <typename Scalar>
struct ResidualDataCoMVelocityTpl;
typedef ResidualModelCoMVelocityTpl<double> ResidualModelCoMVelocity;
typedef ResidualDataCoMVelocityTpl<double> ResidualDataCoMVelocity;

// Activation quad-ref
template <typename Scalar>
class ActivationModelQuadRefTpl;
typedef ActivationModelQuadRefTpl<double> ActivationModelQuadRef;


typedef Eigen::Matrix<double, 6, 1> eVector6;
typedef Eigen::Matrix<double, 4, 1> eVector4;
typedef Eigen::Vector3d eVector3;
typedef Eigen::Vector2d eVector2;
typedef boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> IAM;
typedef boost::shared_ptr<crocoddyl::IntegratedActionDataEuler> IAD;
typedef boost::shared_ptr<crocoddyl::ActionModelAbstract> AMA;
typedef boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics> DAM;
typedef boost::shared_ptr<crocoddyl::CostModelSum> Cost;
typedef boost::shared_ptr<crocoddyl::ContactModelMultiple> Contact;
typedef boost::shared_ptr<crocoddyl::SolverFDDP> DDP;

}  // namespace sobec

#endif  // SOBEC_FWD_HPP_

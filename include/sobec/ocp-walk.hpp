///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_OCP_WALK_HPP_
#define SOBEC_OCP_WALK_HPP_

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;
/**
 * @brief OCP builder.
 */

struct OCPWalkParam
{
  double DT;
  std::vector<pinocchio::FrameIndex> contactIds;
  
  
};

struct OCPRobotWrapper
{
  boost::shared_ptr<pinocchio::Model> model;
  boost::shared_ptr<pinocchio::Data> data;
  std::vector<pinocchio::FrameIndex> contactIds;
  std::map<pinocchio::FrameIndex,pinocchio::FrameIndex> towIds,heelIds;
  Eigen::VectorXd x0;
  Eigen::Vector3d com0;
  double robotGravityForce;

  OCPRobotWrapper( boost::shared_ptr<pinocchio::Model> model,
                   const std::string & contactKey,
                   const std::string & referencePosture = "half_sitting");
   
  
};
  
class OCPWalk {
  typedef typename MathBaseTpl<double>::VectorXs VectorXd;
  typedef typename MathBaseTpl<double>::VectorXs Vector3d;
  typedef boost::shared_ptr<ActionModelAbstract> ActionPtr;
  typedef std::vector<ActionPtr> ActionList;
  typedef typename crocoddyl::DifferentialActionModelContactFwdDynamicsTpl<double> DAM;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OCPWalk(boost::shared_ptr<OCPRobotWrapper> robot,
                   boost::shared_ptr<OCPWalkParam> params)
    : params(params),robot(robot) {}

  virtual ~OCPWalk() {}

  boost::shared_ptr<IntegratedActionModelEuler> buildRunningModel();
  boost::shared_ptr<IntegratedActionModelEuler> buildTerminalModel();
  
 public:

  /// @brief the OCP problem used for solving.
  boost::shared_ptr<ShootingProblem> problem;

  /// @brief Keep a direct reference to the terminal state
  boost::shared_ptr<StateMultibody> state;

 protected:
  boost::shared_ptr<OCPWalkParam> params;
  boost::shared_ptr<OCPRobotWrapper> robot;
};

}  // namespace sobec

#endif  // SOBEC_OCP_WALK_HPP_

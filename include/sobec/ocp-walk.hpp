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
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/spatial/force.hpp>

#include "sobec/fwd.hpp"
#include "sobec/residual-cop.hpp"

namespace sobec {
using namespace crocoddyl;
/**
 * @brief OCP builder.
 */

struct OCPWalkParams {
  double DT;
  std::vector<std::string> mainJointIds;
  Eigen::Vector2d baumgartGains;
  Eigen::VectorXd stateImportance;
  Eigen::VectorXd stateTerminalImportance;
  Eigen::VectorXd controlImportance;
  Eigen::VectorXd vcomImportance;
  Eigen::VectorXd forceImportance;

  Eigen::Vector3d vcomRef;

  double footSize;

  double refStateWeight;
  double refTorqueWeight;
  double comWeight;
  double vcomWeight;
  double copWeight;
  double conePenaltyWeight;
  double coneAxisWeight;
  double refForceWeight;
  double impactAltitudeWeight;
  double impactVelocityWeight;
  double impactRotationWeight;
  double refMainJointsAtImpactWeight;
  double verticalFootVelWeight;
  double flyHighSlope;
  double flyHighWeight;
  double groundColWeight;
  double footMinimalDistance;
  double feetCollisionWeight;
  double kktDamping;
  double stateTerminalWeight;
  double solver_th_stop;
  int transitionDuration;

  void readParams(std::string& StringToParse);
  void readParamsFile(const std::string& Filename);
};

struct OCPRobotWrapper {
  boost::shared_ptr<pinocchio::Model> model;
  boost::shared_ptr<pinocchio::Data> data;
  std::vector<pinocchio::FrameIndex> contactIds;
  std::map<pinocchio::FrameIndex, pinocchio::FrameIndex> towIds, heelIds;
  Eigen::VectorXd x0;
  Eigen::Vector3d com0;
  double robotGravityForce;

  OCPRobotWrapper(boost::shared_ptr<pinocchio::Model> model,
                  const std::string& contactKey,
                  const std::string& referencePosture = "half_sitting");
};

Eigen::MatrixXd computeWeightShareSmoothProfile(
    const Eigen::Ref<const Eigen::MatrixXd> contact_pattern, int duration);

class OCPWalk {
  typedef typename MathBaseTpl<double>::VectorXs VectorXd;
  typedef typename MathBaseTpl<double>::VectorXs Vector3d;
  typedef typename Eigen::Matrix<double, Eigen::Dynamic, 2> MatrixX2d;
  typedef typename Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixX6d;
  typedef std::vector<AMA> ActionList;
  typedef
      typename crocoddyl::DifferentialActionModelContactFwdDynamicsTpl<double>
          DAM;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OCPWalk(boost::shared_ptr<OCPRobotWrapper> robot,
                   boost::shared_ptr<OCPWalkParams> params,
                   const Eigen::Ref<const Eigen::MatrixXd> contact_pattern);

  virtual ~OCPWalk() {}

  std::vector<AMA> buildRunningModels();
  AMA buildTerminalModel();
  void buildSolver();
  std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>
  buildInitialGuess();

  void computeReferenceForces();

 public:
  /// @brief the OCP problem used for solving.
  boost::shared_ptr<ShootingProblem> problem;

  /// @brief the OCP solver.
  boost::shared_ptr<SolverFDDP> solver;

  /// @brief Keep a direct reference to the terminal state
  boost::shared_ptr<StateMultibody> state;

  boost::shared_ptr<ActuationModelFloatingBase> actuation;

  std::vector<std::vector<pinocchio::Force>> referenceForces;
  Eigen::MatrixXd contact_pattern;

 protected:
  boost::shared_ptr<OCPWalkParams> params;
  boost::shared_ptr<OCPRobotWrapper> robot;
};

}  // namespace sobec

#endif  // SOBEC_OCP_WALK_HPP_

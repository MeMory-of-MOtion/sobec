///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_MPC_WALK_HPP_
#define SOBEC_MPC_WALK_HPP_

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/multibody/fwd.hpp>
//#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;
/**
 * @brief MPC manager, calling iterative subpart of a larger OCP.
 */

struct MPCWalkParams {
  /// @brief reference COM velocity
  Eigen::Vector3d vcomRef;
  /// @brief reference 0 state
  Eigen::VectorXd x0;
  /// @brief Duration of the MPC horizon.
  int Tmpc;
  /// @brief Duration of start phase of the OCP.
  int Tstart;
  /// @brief Duration of double-support phases of the OCP.
  int Tdouble;
  /// @brief Duration of single-support phases of the OCP.
  int Tsingle;
  /// @brief Duration of the end phase of the OCP.
  int Tend;
  /// @brief timestep in problem shooting nodes
  double DT;
  /// @brief stop threshold to configure the solver
  double solver_th_stop;
  /// @brief solver param reg_min
  double solver_reg_min;
  /// @brief Solver max number of iteration
  int solver_maxiter;

  /// @brief name of the regularization cost that is modified by mpc update.
  std::string stateRegCostName;

  MPCWalkParams();
  virtual ~MPCWalkParams() {}

  /// @brief Read a yaml parameter list, as a string.
  void readParamsFromYamlString(std::string& StringToParse);
  /// @brief Read a yaml parameter list from a file.
  void readParamsFromYamlFile(std::string& Filename);
};

class MPCWalk {
  typedef typename MathBaseTpl<double>::VectorXs VectorXd;
  typedef typename MathBaseTpl<double>::VectorXs Vector3d;
  typedef std::vector<AMA> ActionList;
  // typedef typename
  // crocoddyl::DifferentialActionModelContactFwdDynamicsTpl<double> DAM;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MPCWalk(boost::shared_ptr<MPCWalkParams> params,
                   boost::shared_ptr<ShootingProblem> problem);

  virtual ~MPCWalk() {}

  /// @brief once all fields are set, init the mpc manager with guess traj
  void initialize(const std::vector<Eigen::VectorXd>& xs,
                  const std::vector<Eigen::VectorXd>& us);

  /// @brief calc the OCP solution. Init must be called first.
  void calc(const Eigen::Ref<const VectorXd>& x, const int t);

  /////// INTERNALS
  void updateTerminalCost(const int t);
  void findTerminalStateResidualModel();
  void findStateModel();

 public:
  /// @brief Parameters to tune the algorithm, given at init.
  boost::shared_ptr<MPCWalkParams> params;

  /// @brief The reference shooting problem storing all shooting nodes
  boost::shared_ptr<ShootingProblem> storage;

  /// @brief the MPC problem used for solving.
  boost::shared_ptr<ShootingProblem> problem;

  /// @brief Solver for MPC
  boost::shared_ptr<SolverFDDP> solver;

  /// @brief Keep a direct reference to the terminal residual
  boost::shared_ptr<ResidualModelState> terminalStateResidual;

  /// @brief Keep a direct reference to the terminal state
  boost::shared_ptr<StateMultibody> state;

 protected:
  double reg;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */

#include "sobec/mpc-walk.hxx"

#endif  // SOBEC_MPC_WALK_HPP_

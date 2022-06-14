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
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
//#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;
/**
 * @brief MPC manager, calling iterative subpart of a larger OCP.
 */
  
class MPCWalk {
  typedef typename MathBaseTpl<double>::VectorXs VectorXd;
  typedef typename MathBaseTpl<double>::VectorXs Vector3d;
  typedef boost::shared_ptr<ActionModelAbstract> ActionPtr;
  typedef std::vector<ActionPtr> ActionList;
  //typedef typename crocoddyl::DifferentialActionModelContactFwdDynamicsTpl<double> DAM;
  
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  explicit MPCWalk(boost::shared_ptr<ShootingProblem> problem);
  
  virtual ~MPCWalk() {}

  /// @brief once all fields are set, init the mpc manager with guess traj
  void initialize(const std::vector<Eigen::VectorXd>& xs,
                  const std::vector<Eigen::VectorXd>& us);

  /// @brief calc the OCP solution. Init must be called first.
  void calc(const Eigen::Ref<const VectorXd>& x,
            const int t);

  /////// INTERNALS
  void updateTerminalCost(const int t);
  void findTerminalStateResidual();

  
  // Setters and getters

  void set_Tmpc(const int v) { Tmpc = v; }
  int get_Tmpc() { return Tmpc; }

  void set_Tstart(const int v) { Tstart = v; }
  int get_Tstart() { return Tstart; }

  void set_Tdouble(const int v) { Tdouble = v; }
  int get_Tdouble() { return Tdouble; }

  void set_Tsingle(const int v) { Tsingle = v; }
  int get_Tsingle() { return Tsingle; }

  void set_Tend(const int v) { Tend = v; }
  int get_Tend() { return Tend; }

  void set_vcomRef(const Eigen::Ref<const Vector3d> & v) { vcomRef = v; }
  const Vector3d& get_vcomRef() { return vcomRef; }
  

public:
  
  /// @brief reference COM velocity
  Vector3d vcomRef;
  /// @brief reference 0 state
  VectorXd x0;
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
  double solver_maxiter;
  
  /// @brief name of the regularization cost that is modified by mpc update.
  std::string stateRegCostName;
  
  /// @brief The reference shooting problem storing all shooting nodes
  boost::shared_ptr<ShootingProblem> storage;

  /// @brief the MPC problem used for solving.
  boost::shared_ptr<ShootingProblem> problem;
  
  /// @brief Solver for MPC
  boost::shared_ptr<SolverFDDP> solver;

  /// @brief Keep a direct reference to the terminal residual
  boost::shared_ptr<ResidualModelState> terminalStateResidual;

protected:
  double reg;
  
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */

#include "sobec/mpc-walk.hxx"

#endif  // SOBEC_MPC_WALK_HPP_

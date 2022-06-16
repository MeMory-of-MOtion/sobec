///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "sobec/mpc-walk.hpp"

namespace sobec {
using namespace crocoddyl;


  MPCWalkParams::
  MPCWalkParams()
    : vcomRef(3)
    , solver_th_stop(1e-9)
    , stateRegCostName("stateReg")
  {}

MPCWalk::MPCWalk(boost::shared_ptr<MPCWalkParams> params,
                 boost::shared_ptr<ShootingProblem> problem)
    : 
  params(params)
  ,storage(problem) {
  // std::cout << "Constructor" << std::endl;
}

void MPCWalk::initialize(const std::vector<Eigen::VectorXd>& xs,
                         const std::vector<Eigen::VectorXd>& us) {
  MPCWalkParams & p = *params;
  assert(p.Tmpc > 0);
  assert(p.Tstart + p.Tend + 2 * (p.Tdouble + p.Tsingle) <= storage->get_T());

  if (p.x0.size() == 0) p.x0 = storage->get_x0();

  // Init shooting problem for mpc solver
  ActionList runmodels;
  for (int t = 0; t < p.Tmpc; ++t) {
    // std::cout << storage->get_runningModels().size() << " " << t <<
    // std::endl;
    runmodels.push_back(storage->get_runningModels()[t]);
  }
  AMA termmodel = storage->get_terminalModel();
  problem = boost::make_shared<ShootingProblem>(p.x0, runmodels, termmodel);

  findTerminalStateResidualModel();
  findStateModel();
  updateTerminalCost(0);

  // Init solverc
  solver = boost::make_shared<SolverFDDP>(problem);
  solver->set_th_stop(p.solver_th_stop);
  solver->set_reg_min(p.solver_reg_min);

  reg = p.solver_reg_min;

  // Run first solve
  solver->solve(xs, us);
}

void MPCWalk::findStateModel() {
  state = boost::dynamic_pointer_cast<StateMultibody>(
      problem->get_terminalModel()->get_state());
}

void MPCWalk::findTerminalStateResidualModel() {
  boost::shared_ptr<IntegratedActionModelEuler> iam =
      boost::dynamic_pointer_cast<IntegratedActionModelEuler>(
          problem->get_terminalModel());
  assert(iam != 0);
  // std::cout << "IAM" << std::endl;

  boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics> dam =
      boost::dynamic_pointer_cast<
          crocoddyl::DifferentialActionModelContactFwdDynamics>(
          iam->get_differential());
  assert(dam != 0);
  // std::cout<<"DAM0" << *dam << std::endl;

  boost::shared_ptr<CostModelSum> costsum = dam->get_costs();
  // std::cout << "Cost sum" << std::endl;

  const CostModelSum::CostModelContainer& costs = costsum->get_costs();
  // std::cout << "Costs" << std::endl;
  assert(costs.find(params->stateRegCostName) != costs.end());

  boost::shared_ptr<CostModelAbstract> costa =
      boost::const_pointer_cast<CostModelAbstract>(
          costs.at(params->stateRegCostName)->cost);
  // std::cout << "IAM" << std::endl;

  boost::shared_ptr<CostModelResidual> cost =
      boost::dynamic_pointer_cast<CostModelResidual>(costa);
  assert(cost != 0);
  // std::cout << "Cost" << std::endl;

  cost->get_residual();
  boost::shared_ptr<ResidualModelState> residual =
      boost::dynamic_pointer_cast<ResidualModelState>(cost->get_residual());
  assert(residual != 0);
  residual->get_reference();
  // std::cout << "Res" << std::endl;

  this->terminalStateResidual = residual;
}

void MPCWalk::updateTerminalCost(const int t) {
  const MPCWalkParams & p = *params;
  VectorXd xref = p.x0;
  xref.head<3>() += p.vcomRef * (t + p.Tmpc) * p.DT;
  terminalStateResidual->set_reference(xref);
}

void MPCWalk::calc(const Eigen::Ref<const VectorXd>& x, const int t) {
  // std::cout << "calc Tmpc=" << Tmpc << std::endl;

  const MPCWalkParams & p = *params;
  
  /// Change the value of the reference cost
  updateTerminalCost(t);

  /// Recede the horizon
  int Tcycle = 2 * (p.Tsingle + p.Tdouble);
  int tlast = p.Tstart + 1 + ((t + p.Tmpc - p.Tstart - 1) % Tcycle);
  // std::cout << "tlast = " << tlast << std::endl;
  problem->circularAppend(storage->get_runningModels()[tlast],
                          storage->get_runningDatas()[tlast]);

  /// Change Warm start
  std::vector<Eigen::VectorXd>& xs_opt =
      const_cast<std::vector<Eigen::VectorXd>&>(solver->get_xs());
  std::vector<Eigen::VectorXd> xs_guess(xs_opt.begin() + 1, xs_opt.end());
  xs_guess.push_back(xs_guess.back());

  std::vector<Eigen::VectorXd>& us_opt =
      const_cast<std::vector<Eigen::VectorXd>&>(solver->get_us());
  std::vector<Eigen::VectorXd> us_guess(us_opt.begin() + 1, us_opt.end());
  us_guess.push_back(us_guess.back());

  solver->setCandidate(xs_guess, us_guess);

  /// Change init constraint
  problem->set_x0(x);

  /// Solve
  solver->solve(xs_guess, us_guess, p.solver_maxiter, false, reg);
  reg = solver->get_xreg();
}

}  // namespace sobec

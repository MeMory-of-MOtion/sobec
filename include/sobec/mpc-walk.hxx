///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include "sobec/mpc-walk.hpp"

namespace sobec {
using namespace crocoddyl;

  MPCWalk::MPCWalk(boost::shared_ptr<ShootingProblem> problem)
    : vcomRef(3)
    ,stateRegCostName("stateReg")

    ,storage(problem)
  {
    std::cout << "Constructor" << std::endl;
  }
  
  void MPCWalk::initialize(const std::vector<Eigen::VectorXd>& xs,
                           const std::vector<Eigen::VectorXd>& us)
  {
    if (x0.size()==0)
      x0 = storage->get_x0();
    
    // Init shooting problem for mpc solver
    ActionList runmodels;
    for(int t=0;t<Tmpc;++t)
      {
        std::cout << storage->get_runningModels().size() << " " << t << std::endl;
        runmodels.push_back(storage->get_runningModels()[t]);
      }
    ActionPtr termmodel = storage->get_terminalModel();
    problem = boost::make_shared<ShootingProblem>(x0,
                                                  runmodels,termmodel);

    findTerminalStateResidual();
    
    // Init solverc
    solver = boost::make_shared<SolverFDDP>(problem);

    // Run first solve
    //solver->solve(xs,us);
  }

  void MPCWalk::findTerminalStateResidual()
  {
    boost::shared_ptr<IntegratedActionModelEuler> iam =
      boost::dynamic_pointer_cast<IntegratedActionModelEuler>(problem->get_terminalModel());
    assert(iam!=0);
    std::cout << "IAM" << std::endl;

    boost::shared_ptr<DifferentialActionModelAbstract> dama = iam->get_differential();
    std::cout<<"DAMA"<<std::endl;

    //boost::shared_ptr<DifferentialActionModelContactFwdDynamics> dam0 =
      boost::dynamic_pointer_cast<DifferentialActionModelContactFwdDynamics>(dama);
    std::cout<<"DAM0" << std::endl;
    
    boost::shared_ptr<DAM> dam =
      boost::dynamic_pointer_cast<DAM>(iam->get_differential());
    assert(dam!=0);
    std::cout << "DAM" << std::endl;

    boost::shared_ptr<CostModelSum> costsum = dam->get_costs();
    std::cout << "Cost sum" << std::endl;
    
    const CostModelSum::CostModelContainer & costs = costsum->get_costs();
    std::cout << "Costs" << std::endl;
    assert(costs.find(stateRegCostName)!=costs.end());
           
    boost::shared_ptr<CostModelAbstract> costa =
      boost::const_pointer_cast<CostModelAbstract>(costs.at(stateRegCostName)->cost);
    std::cout << "IAM" << std::endl;
    
    boost::shared_ptr<CostModelResidual> cost =
      boost::dynamic_pointer_cast<CostModelResidual>(costa);
    assert(cost!=0);
    std::cout << "Cost" << std::endl;
    
    cost->get_residual();
    boost::shared_ptr<ResidualModelState> residual =
      boost::dynamic_pointer_cast<ResidualModelState>(cost->get_residual());
    assert(residual!=0);
    residual->get_reference();
    std::cout << "Res" << std::endl;
    
    this->terminalStateResidual = residual;
  }
  
  void MPCWalk::updateTerminalCost(const int t)
  {
    VectorXd xref = x0;
    xref.head<3>() += vcomRef*(t+Tmpc);
    terminalStateResidual->set_reference(xref);
  }
  
  void MPCWalk::calc(const Eigen::Ref<const VectorXd>& x,
                     const int t)
  {
    std::cout << "calc Tmpc=" << Tmpc << std::endl;

    /// Change the value of the reference cost
    updateTerminalCost(t);

    /// Change Warm start
    std::vector<Eigen::VectorXd>& xs_opt =
      const_cast< std::vector<Eigen::VectorXd>& > (solver->get_xs());
    std::vector<Eigen::VectorXd> xs_guess( xs_opt.begin()+1,xs_opt.end() );
    xs_guess.push_back(*xs_guess.end());
    
    std::vector<Eigen::VectorXd>& us_opt =
      const_cast< std::vector<Eigen::VectorXd>& > (solver->get_us());
    std::vector<Eigen::VectorXd> us_guess( us_opt.begin()+1,us_opt.end() );
    us_guess.push_back(*us_guess.end());

    /// Change init constraint
    problem->set_x0(x);
  }



  
}  // namespace sobec

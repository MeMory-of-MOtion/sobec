///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include "sobec/mpc-walk.hpp"

namespace sobec {
using namespace crocoddyl;

  MPCWalk::MPCWalk(boost::shared_ptr<ShootingProblem> problem)
    : vcomRef(3)

    ,storage(problem)
  {
    std::cout << "Constructor" << std::endl;
  }
  
  void MPCWalk::initialize(const std::vector<Eigen::VectorXd>& xs,
                           const std::vector<Eigen::VectorXd>& us)
  {
    // Init shooting problem for mpc solver
    ActionList runmodels;
    for(int t=0;t<Tmpc;++t)
      {
        std::cout << storage->get_runningModels().size() << " " << t << std::endl;
        runmodels.push_back(storage->get_runningModels()[t]);
      }
    ActionPtr termmodel = storage->get_terminalModel();
    problem = boost::make_shared<ShootingProblem>(storage->get_x0(),
                                                  runmodels,termmodel);

    // Init solverc
    solver = boost::make_shared<SolverFDDP>(problem);

    // Run first solve
    solver->solve(xs,us);
  }

  void MPCWalk::updateTerminalCost(const int t)
  {
    
  }
  
  void MPCWalk::calc(const Eigen::Ref<const VectorXd>& x,
                     const int t)
  {
    std::cout << "calc Tmpc=" << Tmpc << std::endl;
  }



  
}  // namespace sobec

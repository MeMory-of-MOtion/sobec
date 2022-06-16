///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/ocp-walk.hpp"

namespace sobec {

OCPWalk::OCPWalk(boost::shared_ptr<OCPRobotWrapper> robot,
                 boost::shared_ptr<OCPWalkParams> params,
                 const Eigen::Ref<const Eigen::MatrixXd> contact_pattern)
    : contact_pattern(contact_pattern), params(params), robot(robot) {
  computeReferenceForces();
}

void OCPWalk::buildSolver() {
  auto models = buildRunningModels();
  auto termmodel = buildTerminalModel();
  problem = boost::make_shared<ShootingProblem>(robot->x0, models, termmodel);
  solver = boost::make_shared<SolverFDDP>(problem);
  solver->set_th_stop(params->solver_th_stop);
}

std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>
OCPWalk::buildInitialGuess() {
  std::vector<Eigen::VectorXd> x0s, u0s;

  // TODO: load guessfile

  if (x0s.size() != problem->get_T() + 1 || u0s.size() != problem->get_T()) {
    x0s.clear();
    u0s.clear();
    const auto models = problem->get_runningModels();
    const auto datas = problem->get_runningDatas();
    for (int i = 0; i < problem->get_T(); i++) {
      x0s.push_back(problem->get_x0());
      u0s.push_back(models[i]->quasiStatic_x(datas[i], problem->get_x0()));
    }
    x0s.push_back(problem->get_x0()); // Final state for the terminal node (no U)
  }
  return std::make_pair(x0s, u0s);
}

}  // namespace sobec

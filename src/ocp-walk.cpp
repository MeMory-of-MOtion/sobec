///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/ocp-walk.hpp"

namespace sobec {

boost::shared_ptr<SolverFDDP> OCPWalk::buildSolver(
    const Eigen::Ref<const Eigen::MatrixX2d>& contact_pattern,
    const std::vector<std::vector<pinocchio::Force>>& reference_forces) {
  auto models = buildRunningModels(contact_pattern, reference_forces);
  auto termmodel = buildTerminalModel(contact_pattern);
  auto problem =
      boost::make_shared<ShootingProblem>(robot->x0, models, termmodel);
  auto ddp = boost::make_shared<SolverFDDP>(problem);
  ddp->set_th_stop(params->solver_th_stop);
  return ddp;
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
    for (int i = 0; i < problem->get_T() + 1; i++) {
      x0s.push_back(problem->get_x0());
      u0s.push_back(models[i]->quasiStatic_x(datas[i], problem->get_x0()));
    }
  }
  return std::make_pair(x0s, u0s);
}

}  // namespace sobec

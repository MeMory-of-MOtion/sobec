///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/ocp-walk.hpp"

namespace sobec {

OCPWalk::ActionPtr OCPWalk::buildTerminalModel(
    const Eigen::Ref<const MatrixX2d>& contact_pattern,
    const std::vector<std::vector<pinocchio::Force> >& reference_forces) {
  // TODO : get andrea's
  auto models = buildRunningModels(contact_pattern, reference_forces);
  return models[0];
}

boost::shared_ptr<SolverFDDP> OCPWalk::buildSolver(
    const Eigen::Ref<const Eigen::MatrixX2d>& contact_pattern,
    const std::vector<std::vector<pinocchio::Force> >& reference_forces) {
  auto models = buildRunningModels(contact_pattern, reference_forces);
  auto termmodel = buildTerminalModel(contact_pattern, reference_forces);
  auto problem =
      boost::make_shared<ShootingProblem>(robot->x0, models, termmodel);
  auto ddp = boost::make_shared<SolverFDDP>(problem);
  ddp->set_th_stop(params->solver_th_stop);
  return ddp;
}

}  // namespace sobec

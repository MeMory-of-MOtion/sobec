///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2022, LAAS-CNRS, New York University, Max Planck
// Gesellschaft
//                          University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#define BOOST_TEST_MODULE init shooting problem
#include <boost/test/included/unit_test.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <iostream>

#include "sobec/py2cpp.hpp"

BOOST_AUTO_TEST_CASE(test_init_shooting_problem) {
  auto problem = sobec::initShootingProblem(PROJECT_SOURCE_DIR "/mpc/walk.py");

  std::cout << "got problem: " << problem << std::endl;
  auto n_models = problem->get_runningModels().size();
  std::cout << "with " << n_models << " models, and x0:" << std::endl;
  std::cout << problem->get_x0() << std::endl;

  BOOST_CHECK(n_models == 324);
}

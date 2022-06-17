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
#include <iostream>

#include "sobec/ocp-walk.hpp"

BOOST_AUTO_TEST_CASE(test_walk_params) {
  sobec::OCPWalkParams anOCPWalkParams;
  std::string filename(PROJECT_SOURCE_DIR"/tests/config_walk.yaml");
  anOCPWalkParams.readParamsFile(filename);

  BOOST_CHECK(anOCPWalkParams.DT == 0.01);
  
}

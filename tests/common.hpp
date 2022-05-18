///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

// This file is a near direct copy of crocoddyl/unittest/unittest_common.hpp

/**
 * To be included last in the test_XXX.cpp,
 * otherwise it interferes with pinocchio boost::variant.
 */

#ifndef SOBEC_UNITTEST_COMMON_HPP_
#define SOBEC_UNITTEST_COMMON_HPP_

#define NUMDIFF_MODIFIER 10.

#include <iterator>
#include <boost/test/included/unit_test.hpp>
#include <boost/test/execution_monitor.hpp>  // for execution_exception
#include <boost/function.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string>
#include <crocoddyl/core/utils/exception.hpp>

#endif  // SOBEC_UNITTEST_COMMON_HPP_

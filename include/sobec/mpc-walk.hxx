///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/utils/exception.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "sobec/mpc-walk.hpp"

namespace sobec {
using namespace crocoddyl;

  MPCWalk::MPCWalk(boost::shared_ptr<ShootingProblem> problem)
  {
    std::cout << "Constructor" << std::endl;
  }
  
  MPCWalk::~MPCWalk()
  {
  }

  void MPCWalk::calc(const Eigen::Ref<const VectorXs>& x)
  {
    std::cout << "calc" << std::endl;
  }
  
}  // namespace sobec

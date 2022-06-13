///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/mpc-walk.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeMPCWalk() {
  bp::register_ptr_to_python<boost::shared_ptr<MPCWalk> >();

  bp::class_<MPCWalk> (
      "MPCWalk",
      bp::init<boost::shared_ptr<ShootingProblem > >(
               bp::args("self"),
          "Initialize the MPC (empty init)"))
    .def<void (MPCWalk::*)(
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &MPCWalk::calc, bp::args("self", "x"))
    ;

}

}  // namespace python
}  // namespace sobec

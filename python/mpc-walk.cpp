///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

// #include <pinocchio/bindings/python/utils/list.hpp>
// #include <pinocchio/multibody/fwd.hpp>

// // Must be included first!
// #include <boost/python.hpp>
// #include <boost/python/enum.hpp>
// #include <eigenpy/eigenpy.hpp>

// #include "sobec/fwd.hpp"
// #include "sobec/mpc-walk.hpp"

#include "sobec/mpc-walk.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeMPCWalk() {
  bp::register_ptr_to_python<boost::shared_ptr<MPCWalk> >();

  bp::class_<MPCWalk>(
      "MPCWalk", bp::init<boost::shared_ptr<ShootingProblem> >(bp::args("self"), "Initialize the MPC (empty init)"))
      .def<void (MPCWalk::*)(const Eigen::Ref<const Eigen::VectorXd>&, const int)>("calc", &MPCWalk::calc,
                                                                                   bp::args("self", "x", "t"))
      .def("initialize", &MPCWalk::initialize)
      .add_property("Tmpc", &MPCWalk::get_Tmpc, &MPCWalk::set_Tmpc, "duration of MPC horizon")
      .add_property("Tstart", &MPCWalk::get_Tstart, &MPCWalk::set_Tstart, "duration of the starting phase")
      .add_property("Tdouble", &MPCWalk::get_Tdouble, &MPCWalk::set_Tdouble, "duration of the double-support phase")
      .add_property("Tsingle", &MPCWalk::get_Tsingle, &MPCWalk::set_Tsingle, "duration of the single-support phase")
      .add_property("Tend", &MPCWalk::get_Tend, &MPCWalk::set_Tend, "duration of end the phase")
      .add_property("vcomRef", bp::make_getter(&MPCWalk::vcomRef, bp::return_internal_reference<>()),
                    bp::make_setter(&MPCWalk::vcomRef), "Reference of the com velocity, to tune the MPC at runtime.")
      .add_property("solver_th_stop", bp::make_getter(&MPCWalk::solver_th_stop),
                    bp::make_setter(&MPCWalk::solver_th_stop), "Stop threshold to configure the solver.")
      .add_property("solver_reg_min", bp::make_getter(&MPCWalk::solver_reg_min),
                    bp::make_setter(&MPCWalk::solver_reg_min),
                    "reg_min param (minimal regularization) to configure the solver.")
      .add_property("solver_maxiter", bp::make_getter(&MPCWalk::solver_maxiter),
                    bp::make_setter(&MPCWalk::solver_maxiter), "maxiter param to configure the solver.")
      .add_property("DT", bp::make_getter(&MPCWalk::DT), bp::make_setter(&MPCWalk::DT),
                    "time step duration of the shooting nodes.")
      .add_property("x0", bp::make_getter(&MPCWalk::x0, bp::return_internal_reference<>()),
                    bp::make_setter(&MPCWalk::x0), "Reference of the com velocity, to tune the MPC at runtime.")
      .add_property("storage", bp::make_getter(&MPCWalk::storage, bp::return_value_policy<bp::return_by_value>()),
                    "Shooting storage used for MPC solver")
      .add_property("problem", bp::make_getter(&MPCWalk::problem, bp::return_value_policy<bp::return_by_value>()),
                    "Shooting problem used for MPC solver")
      .add_property("state", bp::make_getter(&MPCWalk::state, bp::return_value_policy<bp::return_by_value>()),
                    "State model of the terminal node")
      .add_property("solver", bp::make_getter(&MPCWalk::solver, bp::return_value_policy<bp::return_by_value>()),
                    "OCP Solver inside the MPC.")

      ;
}

}  // namespace python
}  // namespace sobec

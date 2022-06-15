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
// #include "sobec/ocp-walk.hpp"

#include "sobec/ocp-walk.hpp"

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

void exposeOCPRobotWrapper() {
  bp::register_ptr_to_python<boost::shared_ptr<OCPRobotWrapper> >();

  bp::class_<OCPRobotWrapper>(
      "OCPRobotWrapper",
      bp::init<boost::shared_ptr<pinocchio::Model>, std::string, std::string>(
          bp::args("self", "model", "contactKey", "reference_posture"),
          "Initialize the OCP from robot and params"))
      .add_property("x0",
                    bp::make_getter(&OCPRobotWrapper::x0,
                                    bp::return_internal_reference<>()),
                    bp::make_setter(&OCPRobotWrapper::x0), "Reference state")
      .add_property("com0",
                    bp::make_getter(&OCPRobotWrapper::com0,
                                    bp::return_internal_reference<>()),
                    bp::make_setter(&OCPRobotWrapper::com0),
                    "Reference com value (computed at x0)")
      .add_property(
          "model",
          bp::make_getter(&OCPRobotWrapper::model,
                          // bp::return_internal_reference<>()),
                          bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&OCPRobotWrapper::model), "Pinocchio model")
      .add_property("data",
                    bp::make_getter(&OCPRobotWrapper::data,
                                    bp::return_internal_reference<>()),
                    "pinocchio data")
      // .add_property("data",
      //               bp::make_getter(&OCPRobotWrapper::data,
      //                               bp::return_value_policy<bp::return_by_value>()),
      //               //bp::return_internal_reference<>()),
      //               "Pinocchio data")
      ;
}
void exposeOCPParams() {
  bp::register_ptr_to_python<boost::shared_ptr<OCPWalkParams> >();

  bp::class_<OCPWalkParams>(
      "OCPWalkParams",
      bp::init<>(bp::args("self"), "Empty initialization of the OCP params"))
      // .add_property(
      //     "solver_reg_min", bp::make_getter(&OCPWalkParams::solver_reg_min),
      //     bp::make_setter(&OCPWalkParams::solver_reg_min),
      //     "reg_min param (minimal regularization) to configure the solver.")
      // .add_property("solver_maxiter",
      // bp::make_getter(&OCPWalkParams::solver_maxiter),
      //               bp::make_setter(&OCPWalkParams::solver_maxiter),
      //               "maxiter param to configure the solver.")
      .add_property("DT", bp::make_getter(&OCPWalkParams::DT),
                    bp::make_setter(&OCPWalkParams::DT),
                    "time step duration of the shooting nodes.");
}

void exposeOCPWalkclass() {
  bp::register_ptr_to_python<boost::shared_ptr<OCPWalk> >();

  bp::class_<OCPWalk>(
      "OCPWalk",
      bp::init<boost::shared_ptr<OCPRobotWrapper>,
               boost::shared_ptr<OCPWalkParams> >(
          bp::args("self"), "Initialize the OCP from robot and params"))
      // .add_property(
      //     "vcomRef",
      //     bp::make_getter(&OCPWalk::vcomRef,
      //     bp::return_internal_reference<>()),
      //     bp::make_setter(&OCPWalk::vcomRef),
      //     "Reference of the com velocity, to tune the OCP at runtime.")
      // .add_property("solver_th_stop",
      // bp::make_getter(&OCPWalk::solver_th_stop),
      //               bp::make_setter(&OCPWalk::solver_th_stop),
      //               "Stop threshold to configure the solver.")
      .def("buildRunningModel", &OCPWalk::buildRunningModel)
      .add_property(
          "problem",
          bp::make_getter(&OCPWalk::problem,
                          bp::return_value_policy<bp::return_by_value>()),
          "Shooting problem used for OCP solver")
      .add_property(
          "state",
          bp::make_getter(&OCPWalk::state,
                          bp::return_value_policy<bp::return_by_value>()),
          "State model of the terminal node")

      ;
}

void exposeOCPWalk() {
  exposeOCPRobotWrapper();
  exposeOCPParams();
  exposeOCPWalkclass();
}

}  // namespace python
}  // namespace sobec

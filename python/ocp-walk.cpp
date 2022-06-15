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
                    "time step duration of the shooting nodes.")


  // std::vector<pinocchio::FrameIndex> contactIds;
  // std::vector<std::string> mainJointIds;
   
    .add_property("baumgartGains",
                  bp::make_getter(&OCPWalkParams::baumgartGains,
                                  bp::return_internal_reference<>()),
                  bp::make_setter(&OCPWalkParams::baumgartGains),
                  ".")
  
    .add_property("stateImportance",
                  bp::make_getter(&OCPWalkParams::stateImportance,
                                  bp::return_internal_reference<>()),
                  bp::make_setter(&OCPWalkParams::stateImportance),
                  ".")
  
    .add_property("stateTerminalImportance",
                  bp::make_getter(&OCPWalkParams::stateTerminalImportance,
                                  bp::return_internal_reference<>()),
                  bp::make_setter(&OCPWalkParams::stateTerminalImportance),
                  ".")
  
    .add_property("controlImportance",
                  bp::make_getter(&OCPWalkParams::controlImportance,
                                  bp::return_internal_reference<>()),
                  bp::make_setter(&OCPWalkParams::controlImportance),
                  ".")
  
    .add_property("vcomImportance",
                  bp::make_getter(&OCPWalkParams::vcomImportance,
                                  bp::return_internal_reference<>()),
                  bp::make_setter(&OCPWalkParams::vcomImportance),
                  ".")
  
    .add_property("forceImportance",
                  bp::make_getter(&OCPWalkParams::forceImportance,
                                  bp::return_internal_reference<>()),
                  bp::make_setter(&OCPWalkParams::forceImportance),
                  ".")
  

    .add_property("vcomRef",
                  bp::make_getter(&OCPWalkParams::vcomRef,
                                  bp::return_internal_reference<>()),
                  bp::make_setter(&OCPWalkParams::vcomRef),
                  ".")
  
    .add_property("footSize", bp::make_getter(&OCPWalkParams::footSize),
                  bp::make_setter(&OCPWalkParams::footSize),
                  ".")
    .add_property("refStateWeight", bp::make_getter(&OCPWalkParams::refStateWeight),
                  bp::make_setter(&OCPWalkParams::refStateWeight),
                  ".")
    .add_property("refTorqueWeight", bp::make_getter(&OCPWalkParams::refTorqueWeight),
                  bp::make_setter(&OCPWalkParams::refTorqueWeight),
                  ".")
    .add_property("comWeight", bp::make_getter(&OCPWalkParams::comWeight),
                  bp::make_setter(&OCPWalkParams::comWeight),
                  ".")
    .add_property("vcomWeight", bp::make_getter(&OCPWalkParams::vcomWeight),
                  bp::make_setter(&OCPWalkParams::vcomWeight),
                  ".")
    .add_property("copWeight", bp::make_getter(&OCPWalkParams::copWeight),
                  bp::make_setter(&OCPWalkParams::copWeight),
                  ".")
    .add_property("conePenaltyWeight", bp::make_getter(&OCPWalkParams::conePenaltyWeight),
                  bp::make_setter(&OCPWalkParams::conePenaltyWeight),
                  ".")
    .add_property("coneAxisWeight", bp::make_getter(&OCPWalkParams::coneAxisWeight),
                  bp::make_setter(&OCPWalkParams::coneAxisWeight),
                  ".")
    .add_property("refForceWeight", bp::make_getter(&OCPWalkParams::refForceWeight),
                  bp::make_setter(&OCPWalkParams::refForceWeight),
                  ".")
    .add_property("impactAltitudeWeight", bp::make_getter(&OCPWalkParams::impactAltitudeWeight),
                  bp::make_setter(&OCPWalkParams::impactAltitudeWeight),
                  ".")
    .add_property("impactVelocityWeight", bp::make_getter(&OCPWalkParams::impactVelocityWeight),
                  bp::make_setter(&OCPWalkParams::impactVelocityWeight),
                  ".")
    .add_property("impactRotationWeight", bp::make_getter(&OCPWalkParams::impactRotationWeight),
                  bp::make_setter(&OCPWalkParams::impactRotationWeight),
                  ".")
    .add_property("refMainJointsAtImpactWeight", bp::make_getter(&OCPWalkParams::refMainJointsAtImpactWeight),
                  bp::make_setter(&OCPWalkParams::refMainJointsAtImpactWeight),
                  ".")
    .add_property("verticalFootVelWeight", bp::make_getter(&OCPWalkParams::verticalFootVelWeight),
                  bp::make_setter(&OCPWalkParams::verticalFootVelWeight),
                  ".")
    .add_property("flyHighSlope", bp::make_getter(&OCPWalkParams::flyHighSlope),
                  bp::make_setter(&OCPWalkParams::flyHighSlope),
                  ".")
    .add_property("flyHighWeight", bp::make_getter(&OCPWalkParams::flyHighWeight),
                  bp::make_setter(&OCPWalkParams::flyHighWeight),
                  ".")
    .add_property("groundColWeight", bp::make_getter(&OCPWalkParams::groundColWeight),
                  bp::make_setter(&OCPWalkParams::groundColWeight),
                  ".")
    .add_property("footMinimalDistance", bp::make_getter(&OCPWalkParams::footMinimalDistance),
                  bp::make_setter(&OCPWalkParams::footMinimalDistance),
                  ".")
    .add_property("feetCollisionWeight", bp::make_getter(&OCPWalkParams::feetCollisionWeight),
                  bp::make_setter(&OCPWalkParams::feetCollisionWeight),
                  ".")
    .add_property("kktDamping", bp::make_getter(&OCPWalkParams::kktDamping),
                  bp::make_setter(&OCPWalkParams::kktDamping),
                  ".")
    .add_property("stateTerminalWeight", bp::make_getter(&OCPWalkParams::stateTerminalWeight),
                  bp::make_setter(&OCPWalkParams::stateTerminalWeight),
                  ".")
    .add_property("solver_th_stop", bp::make_getter(&OCPWalkParams::solver_th_stop),
                  bp::make_setter(&OCPWalkParams::solver_th_stop),
                  ".")
    .add_property("transitionDuration", bp::make_getter(&OCPWalkParams::transitionDuration),
                  bp::make_setter(&OCPWalkParams::transitionDuration),
                  ".")
    ;
}

  void exposeRefForce() {
        bp::def("computeWeightShareSmoothProfile",
                &computeWeightShareSmoothProfile,
                bp::args("contact_pattern","duration"), //,"robotGravityForce"),
                "Compute the smooth weight transfer between contact points");
                //bp::return_value_policy<bp::return_by_value>());
  }

void exposeOCPWalkclass() {
  bp::register_ptr_to_python<boost::shared_ptr<OCPWalk> >();

  bp::class_<OCPWalk>(
      "OCPWalk",
      bp::init<boost::shared_ptr<OCPRobotWrapper>,
      boost::shared_ptr<OCPWalkParams>,Eigen::MatrixXd >
      (bp::args("self","robot","params","contact_patter"),
       "Initialize the OCP from robot, params and contact pattern"))
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
      .def("buildRunningModels", &OCPWalk::buildRunningModels)
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
  exposeRefForce();
}

}  // namespace python
}  // namespace sobec

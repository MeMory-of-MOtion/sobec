///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/lowpassfilter/statelpf.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>

namespace sobec {
namespace python {
namespace bp = boost::python;

void exposeStateLPF() {
  bp::register_ptr_to_python<
      boost::shared_ptr<sobec::StateLPF> >();

  bp::class_<sobec::StateLPF, bp::bases<crocoddyl::StateAbstract> >(
      "StateLPF",
      "State model for low-pass filtered joints",
      bp::init<boost::shared_ptr<pinocchio::Model>, std::vector<int>>(
          bp::args("self", "model", "lpf_joint_ids"),
          "Initialize the state lpf model.\n\n"
          ":param model: pinocchio model\n"
          ":param lpf_joint_names: list of joints names that are low-pass filtered\n")[bp::with_custodian_and_ward<1, 2>()])
      .def("zero", &sobec::StateLPF::zero, bp::args("self"),
           "Return the neutral robot configuration with zero velocity.\n\n"
           ":return neutral robot configuration with zero velocity")
      .def("rand", &sobec::StateLPF::rand, bp::args("self"),
           "Return a random reference state.\n\n"
           ":return random reference state")
      .def("diff", &sobec::StateLPF::diff_dx, bp::args("self", "x0", "x1"),
           "Operator that differentiates the two robot states.\n\n"
           "It returns the value of x1 [-] x0 operation. This operator uses the Lie\n"
           "algebra since the robot's root could lie in the SE(3) manifold.\n"
           ":param x0: current state (dim state.nx()).\n"
           ":param x1: next state (dim state.nx()).\n"
           ":return x1 - x0 value (dim state.nx()).")
      .def("integrate", &sobec::StateLPF::integrate_x, bp::args("self", "x", "dx"),
           "Operator that integrates the current robot state.\n\n"
           "It returns the value of x [+] dx operation. This operator uses the Lie\n"
           "algebra since the robot's root could lie in the SE(3) manifold.\n"
           "Futhermore there is no timestep here (i.e. dx = v*dt), note this if you're\n"
           "integrating a velocity v during an interval dt.\n"
           ":param x: current state (dim state.nx()).\n"
           ":param dx: displacement of the state (dim state.ndx()).\n"
           ":return x + dx value (dim state.nx()).")
      .def("Jdiff", &sobec::StateLPF::Jdiff_Js, bp::args("self", "x0", "x1", "firstsecond"),
                  "Compute the partial derivatives of the diff operator.\n\n"
                  "Both Jacobian matrices are represented throught an identity matrix, with the exception\n"
                  "that the robot's root is defined as free-flying joint (SE(3)). By default, this\n"
                  "function returns the derivatives of the first and second argument (i.e.\n"
                  "firstsecond='both'). However we ask for a specific partial derivative by setting\n"
                  "firstsecond='first' or firstsecond='second'.\n"
                  ":param x0: current state (dim state.nx()).\n"
                  ":param x1: next state (dim state.nx()).\n"
                  ":param firstsecond: derivative w.r.t x0 or x1 or both\n"
                  ":return the partial derivative(s) of the diff(x0, x1) function")
      .def("Jintegrate", &sobec::StateLPF::Jintegrate_Js, bp::args("self", "x", "dx", "firstsecond"),
                       "Compute the partial derivatives of arithmetic addition.\n\n"
                       "Both Jacobian matrices are represented throught an identity matrix. with the exception\n"
                       "that the robot's root is defined as free-flying joint (SE(3)). By default, this\n"
                       "function returns the derivatives of the first and second argument (i.e.\n"
                       "firstsecond='both'). However we ask for a specific partial derivative by setting\n"
                       "firstsecond='first' or firstsecond='second'.\n"
                       ":param x: current state (dim state.nx()).\n"
                       ":param dx: displacement of the state (dim state.ndx()).\n"
                       ":param firstsecond: derivative w.r.t x or dx or both\n"
                       ":return the partial derivative(s) of the integrate(x, dx) function")
      .def("JintegrateTransport", &sobec::StateLPF::JintegrateTransport,
           bp::args("self", "x", "dx", "Jin", "firstsecond"),
           "Parallel transport from integrate(x, dx) to x.\n\n"
           "This function performs the parallel transportation of an input matrix whose columns\n"
           "are expressed in the tangent space at integrate(x, dx) to the tangent space at x point\n"
           ":param x: state point (dim. state.nx).\n"
           ":param dx: velocity vector (dim state.ndx).\n"
           ":param Jin: input matrix (number of rows = state.nv).\n"
           ":param firstsecond: derivative w.r.t x or dx")
      .add_property("pinocchio",
                    bp::make_function(&StateMultibody::get_pinocchio, bp::return_value_policy<bp::return_by_value>()),
                    "pinocchio model");


}

}  // namespace python
}  // namespace sobec

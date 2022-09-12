///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/crocomplements/softcontact/iam3d-augmented.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/fwd.hpp>  // to avoid compilation error (https://github.com/loco-3d/crocoddyl/issues/205)

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {
using namespace crocoddyl;
namespace bp = boost::python;

void exposeIAMSoftContact3DAugmented() {
  bp::class_<IAMSoftContact3DAugmented, bp::bases<ActionModelAbstract> >(
      "IAMSoftContact3DAugmented",
      "Sympletic Euler integrator for differential action models.\n\n"
      "This class implements a sympletic Euler integrator (a.k.a "
      "semi-implicit\n"
      "integrator) give a differential action model, i.e.:\n"
      "  [q+, v+, tau+] = StateLPF.integrate([q, v], [v + a * dt, a * dt] * "
      "dt, [alpha*tau + (1-alpha)*w]).",
      bp::init<boost::shared_ptr<DAMSoftContact3DAugmentedFwdDynamics>,
               bp::optional<double, bool> >(
          bp::args("self", "diffModel", "stepTime", "withCostResidual"),
          "Initialize the sympletic Euler integrator.\n\n"
          ":param diffModel: differential action model\n"
          ":param stepTime: step time\n"
          ":param withCostResidual: includes the cost residuals and "
          "derivatives\n"
          "computation, or tau"))
      .def<void (IAMSoftContact3DAugmented::*)(
          const boost::shared_ptr<ActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &IAMSoftContact3DAugmented::calc,
          bp::args("self", "data", "x", "u"),
          "Compute the time-discrete evolution of a differential action "
          "model.\n\n"
          "It describes the time-discrete evolution of action model.\n"
          ":param data: action data\n"
          ":param x: state vector\n"
          ":param u: control input")
      .def<void (IAMSoftContact3DAugmented::*)(
          const boost::shared_ptr<ActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ActionModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (IAMSoftContact3DAugmented::*)(
          const boost::shared_ptr<ActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &IAMSoftContact3DAugmented::calcDiff,
          bp::args("self", "data", "x", "u"),
          "Computes the derivatives of the integrated action model wrt state "
          "and control. \n\n"
          "This function builds a quadratic approximation of the\n"
          "action model (i.e. dynamical system and cost function).\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: state vector\n"
          ":param u: control input\n")
      .def<void (IAMSoftContact3DAugmented::*)(
          const boost::shared_ptr<ActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ActionModelAbstract::calcDiff,
          bp::args("self", "data", "x"))
      .def("createData", &IAMSoftContact3DAugmented::createData,
           bp::args("self"), "Create the Euler integrator data.")
      .add_property(
          "differential",
          bp::make_function(&IAMSoftContact3DAugmented::get_differential,
                            bp::return_value_policy<bp::return_by_value>()),
          &IAMSoftContact3DAugmented::set_differential,
          "differential action model")
      .add_property(
          "dt",
          bp::make_function(&IAMSoftContact3DAugmented::get_dt,
                            bp::return_value_policy<bp::return_by_value>()),
          &IAMSoftContact3DAugmented::set_dt, "step time")

      .add_property(
          "nc",
          bp::make_function(&IAMSoftContact3DAugmented::get_nc,
                            bp::return_value_policy<bp::return_by_value>()),
          "Contact model dimension")
      .add_property(
          "ny",
          bp::make_function(&IAMSoftContact3DAugmented::get_ny,
                            bp::return_value_policy<bp::return_by_value>()),
          "augmented state dimension (nx+ntau)");

  bp::register_ptr_to_python<boost::shared_ptr<IADSoftContact3DAugmented> >();

  bp::class_<IADSoftContact3DAugmented, bp::bases<ActionDataAbstract> >(
      "IADSoftContact3DAugmented", "Sympletic Euler integrator data.",
      bp::init<IAMSoftContact3DAugmented*>(
          bp::args("self", "model"),
          "Create sympletic Euler integrator data.\n\n"
          ":param model: sympletic Euler integrator model"))
      .add_property(
          "differential",
          bp::make_getter(&IADSoftContact3DAugmented::differential,
                          bp::return_value_policy<bp::return_by_value>()),
          "differential action data")
      .add_property("dy",
                    bp::make_getter(&IADSoftContact3DAugmented::dy,
                                    bp::return_internal_reference<>()),
                    "state rate.");
}

}  // namespace python
}  // namespace sobec

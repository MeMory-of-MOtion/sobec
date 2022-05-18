///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "python/crocoddyl/core/core.hpp"
#include "python/crocoddyl/core/activation-base.hpp"
#include "crocoddyl/core/activations/quadratic-ref.hpp"

namespace crocoddyl {
namespace python {

void exposeActivationQuadRef() {
  bp::class_<ActivationModelQuadRef, bp::bases<ActivationModelAbstract> >(
      "ActivationModelQuadRef",
      "Quadratic activation model.\n\n"
      "A quadratic action describes a quadratic function that depends on the residual, i.e.\n"
      "0.5 *||r - reference||^2.",
      bp::init<Eigen::VectorXd>(bp::args("self", "reference"),
                    "Initialize the activation model.\n\n"
                    ":param reference: dimension of the reference vector"))
      .def("calc", &ActivationModelQuadRef::calc, bp::args("self", "data", "r"),
           "Compute the 0.5 * ||r - reference||^2.\n\n"
           ":param data: activation data\n"
           ":param r: residual vector")
      .def("calcDiff", &ActivationModelQuadRef::calcDiff, bp::args("self", "data", "r"),
           "Compute the derivatives of a quadratic function.\n\n"
           "Note that the Hessian is constant, so we don't write again this value.\n"
           "It assumes that calc has been run first.\n"
           ":param data: activation data\n"
           ":param r: residual vector \n")
      .def("createData", &ActivationModelQuadRef::createData, bp::args("self"),
           "Create the quadratic activation data.\n\n")
      .add_property("reference",
                    bp::make_function(&ActivationModelQuadRef::get_reference, bp::return_internal_reference<>()),
                    &ActivationModelQuadRef::set_reference, "reference of the quadratic term");
}

}  // namespace python
}  // namespace crocoddyl

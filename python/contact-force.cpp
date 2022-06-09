///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/contact-force.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>

namespace sobec {
namespace python {
using namespace crocoddyl;
namespace bp = boost::python;

void exposeResidualContactForce() {
  bp::register_ptr_to_python<boost::shared_ptr<sobec::ResidualModelContactForce> >();

  bp::class_<sobec::ResidualModelContactForce, bp::bases<crocoddyl::ResidualModelContactForce> >(
      "ResidualModelContactForce",
      "This residual function is defined as r = f-fref, where f,fref describe "
      "the current and reference\n"
      "the spatial forces, respectively.",
      bp::init<boost::shared_ptr<crocoddyl::StateMultibody>, pinocchio::FrameIndex, pinocchio::Force, std::size_t,
               std::size_t>(bp::args("self", "state", "id", "fref", "nc", "nu"),
                            "Initialize the contact force residual model.\n\n"
                            ":param state: state of the multibody system\n"
                            ":param id: reference frame id\n"
                            ":param fref: reference spatial contact force in the contact "
                            "coordinates\n"
                            ":param nc: dimension of the contact force (nc <= 6)\n"
                            ":param nu: dimension of control vector"));
}

}  // namespace python
}  // namespace sobec

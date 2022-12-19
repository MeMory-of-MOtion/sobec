///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/crocomplements/residual-dcm-position.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
// using namespace crocoddyl::python;
namespace bp = boost::python;

void exposeResidualDCMPosition() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelDCMPosition> >();

  bp::class_<ResidualModelDCMPosition, bp::bases<ResidualModelAbstract> >(
      "ResidualModelDCMPosition",
      "This residual function defines the DCM tracking as r = c - cref, with c "
      "and cref as the current and reference "
      "DCM position, respectively.",
      bp::init<boost::shared_ptr<StateMultibody>, Eigen::Vector3d, std::size_t,
               double>(bp::args("self", "state", "cref", "alpha", "nu"),
                       "Initialize the DCM position residual model.\n\n"
                       ":param state: state of the multibody system\n"
                       ":param cref: reference CoM position\n"
                       ":param alpha: inverse of the time constant\n"
                       ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, Eigen::Vector3d, double>(
          bp::args("self", "state", "cref", "alpha"),
          "Initialize the DCM position residual model.\n\n"
          "The default nu is obtained from state.nv.\n"
          ":param state: state of the multibody system\n"
          ":param cref: reference DCM position\n"
          ":param alpha: inverse of the time constant"))
      .def<void (ResidualModelDCMPosition::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelDCMPosition::calc,
          bp::args("self", "data", "x", "u"),
          "Compute the DCM position residual.\n\n"
          ":param data: residual data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelDCMPosition::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelDCMPosition::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelDCMPosition::calcDiff,
          bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the DCM position residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelDCMPosition::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff,
          bp::args("self", "data", "x"))
      .def("createData", &ResidualModelDCMPosition::createData,
           bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the DCM position residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for a predefined residual.\n"
           ":param data: shared data\n"
           ":return residual data.")
      .add_property("reference",
                    bp::make_function(&ResidualModelDCMPosition::get_reference,
                                      bp::return_internal_reference<>()),
                    &ResidualModelDCMPosition::set_reference,
                    "reference DCM position");

  bp::register_ptr_to_python<boost::shared_ptr<ResidualDataDCMPosition> >();

  bp::class_<ResidualDataDCMPosition, bp::bases<ResidualDataAbstract> >(
      "ResidualDataDCMPosition", "Data for DCM position residual.\n\n",
      bp::init<ResidualModelDCMPosition*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create DCM position residual data.\n\n"
          ":param model: DCM position residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3> >()])
      .add_property("pinocchio",
                    bp::make_getter(&ResidualDataDCMPosition::pinocchio,
                                    bp::return_internal_reference<>()),
                    "pinocchio data");
}

}  // namespace python
}  // namespace sobec

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "python/crocoddyl/multibody/multibody.hpp"
#include "crocoddyl/multibody/residuals/com-velocity.hpp"

namespace crocoddyl {
namespace python {

void exposeResidualCoMVelocity() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelCoMVelocity> >();

  bp::class_<ResidualModelCoMVelocity, bp::bases<ResidualModelAbstract> >(
      "ResidualModelCoMVelocity",
      "This residual function defines the CoM tracking as r = v - vref, with v and vref as the current and reference "
      "CoM velocity, respectively.",
      bp::init<boost::shared_ptr<StateMultibody>, Eigen::Vector3d, std::size_t>(
          bp::args("self", "state", "vref", "nu"),
          "Initialize the CoM velocity residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param vref: reference CoM velocity\n"
          ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, Eigen::Vector3d>(
          bp::args("self", "state", "vref"),
          "Initialize the CoM velocity residual model.\n\n"
          "The default nu is obtained from state.nv.\n"
          ":param state: state of the multibody system\n"
          ":param vref: reference CoM velocity"))
      .def<void (ResidualModelCoMVelocity::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                              const Eigen::Ref<const Eigen::VectorXd>&,
                                              const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelCoMVelocity::calc, bp::args("self", "data", "x", "u"),
          "Compute the CoM velocity residual.\n\n"
          ":param data: residual data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelCoMVelocity::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                              const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelCoMVelocity::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                              const Eigen::Ref<const Eigen::VectorXd>&,
                                              const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelCoMVelocity::calcDiff, bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the CoM velocity residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelCoMVelocity::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                              const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff, bp::args("self", "data", "x"))
      .def("createData", &ResidualModelCoMVelocity::createData, bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the CoM velocity residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. This function\n"
           "returns the allocated data for a predefined residual.\n"
           ":param data: shared data\n"
           ":return residual data.")
      .add_property("reference",
                    bp::make_function(&ResidualModelCoMVelocity::get_reference, bp::return_internal_reference<>()),
                    &ResidualModelCoMVelocity::set_reference, "reference CoM velocity");

  bp::register_ptr_to_python<boost::shared_ptr<ResidualDataCoMVelocity> >();

  bp::class_<ResidualDataCoMVelocity, bp::bases<ResidualDataAbstract> >(
      "ResidualDataCoMVelocity", "Data for CoM velocity residual.\n\n",
      bp::init<ResidualModelCoMVelocity*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create CoM velocity residual data.\n\n"
          ":param model: CoM velocity residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<1, 2, bp::with_custodian_and_ward<1, 3> >()])
      .add_property("pinocchio",
                    bp::make_getter(&ResidualDataCoMVelocity::pinocchio, bp::return_internal_reference<>()),
                    "pinocchio data");
}

}  // namespace python
}  // namespace crocoddyl

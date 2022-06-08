///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/contact3d.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>

namespace sobec {
namespace python {
using namespace crocoddyl;
namespace bp = boost::python;

void exposeContact3D() {
  bp::register_ptr_to_python<boost::shared_ptr<sobec::ContactModel3D> >();

  bp::class_<sobec::ContactModel3D, bp::bases<crocoddyl::ContactModel3D> >(
      "ContactModel3D",
      "Rigid 3D contact model.\n\n"
      "It defines a rigid 3D contact models (point contact) based on "
      "acceleration-based holonomic constraints.\n"
      "The calc and calcDiff functions compute the contact Jacobian and drift "
      "(holonomic constraint) or\n"
      "the derivatives of the holonomic constraint, respectively.",
      bp::init<boost::shared_ptr<crocoddyl::StateMultibody>,
               pinocchio::FrameIndex, Eigen::Vector3d, std::size_t,
               bp::optional<Eigen::Vector2d, pinocchio::ReferenceFrame> >(
          bp::args("self", "state", "id", "xref", "nu", "gains", "type"),
          "Initialize the contact model.\n\n"
          ":param state: state of the multibody system\n"
          ":param id: reference frame id of the contact\n"
          ":param xref: contact position used for the Baumgarte stabilization\n"
          ":param nu: dimension of control vector\n"
          ":param gains: gains of the contact model (default "
          "np.matrix([0.,0.]))\n"
          ":param type: reference type of contact"))
      .def(bp::init<boost::shared_ptr<crocoddyl::StateMultibody>,
                    pinocchio::FrameIndex, Eigen::Vector3d,
                    bp::optional<Eigen::Vector2d, pinocchio::ReferenceFrame> >(
          bp::args("self", "state", "id", "xref", "gains", "type"),
          "Initialize the contact model.\n\n"
          ":param state: state of the multibody system\n"
          ":param id: reference frame id of the contact\n"
          ":param xref: contact position used for the Baumgarte stabilization\n"
          ":param gains: gains of the contact model (default "
          "np.matrix([0.,0.]))\n"
          ":param type: reference type of contact"))
      .def("calc", &sobec::ContactModel3D::calc, bp::args("self", "data", "x"),
           "Compute the 3d contact Jacobian and drift.\n\n"
           "The rigid contact model throught acceleration-base holonomic "
           "constraint\n"
           "of the contact frame placement.\n"
           ":param data: contact data\n"
           ":param x: state point (dim. state.nx)")
      .def("calcDiff", &sobec::ContactModel3D::calcDiff,
           bp::args("self", "data", "x"),
           "Compute the derivatives of the 3d contact holonomic constraint.\n\n"
           "The rigid contact model throught acceleration-base holonomic "
           "constraint\n"
           "of the contact frame placement.\n"
           "It assumes that calc has been run first.\n"
           ":param data: cost data\n"
           ":param x: state point (dim. state.nx)")
      .def("updateForce", &sobec::ContactModel3D::updateForce,
           bp::args("self", "data", "force"),
           "Convert the force into a stack of spatial forces.\n\n"
           ":param data: cost data\n"
           ":param force: force vector (dimension 3)")
      .def("createData", &sobec::ContactModel3D::createData,
           bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the 3D contact data.\n\n"
           "Each contact model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for a predefined cost.\n"
           ":param data: Pinocchio data\n"
           ":return contact data.")
      .add_property("reference",
                    bp::make_function(&sobec::ContactModel3D::get_reference,
                                      bp::return_internal_reference<>()),
                    &sobec::ContactModel3D::set_reference,
                    "reference contact translation")
      .add_property(
          "gains",
          bp::make_function(&sobec::ContactModel3D::get_gains,
                            bp::return_value_policy<bp::return_by_value>()),
          "contact gains")
      .add_property(
          "type",
          bp::make_function(&sobec::ContactModel3D::get_type,
                            bp::return_value_policy<bp::return_by_value>()),
          &sobec::ContactModel3D::set_type, "type");

  bp::register_ptr_to_python<boost::shared_ptr<sobec::ContactData3D> >();

  bp::class_<sobec::ContactData3D, bp::bases<crocoddyl::ContactData3D> >(
      "ContactData3D", "Data for 3D contact.\n\n",
      bp::init<sobec::ContactModel3D*, pinocchio::Data*>(
          bp::args("self", "model", "data"),
          "Create 3D contact data.\n\n"
          ":param model: 3D contact model\n"
          ":param data: Pinocchio data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3> >()])
      .add_property(
          "v",
          bp::make_getter(&sobec::ContactData3D::v,
                          bp::return_value_policy<bp::return_by_value>()),
          "spatial velocity of the contact body")
      .add_property(
          "a",
          bp::make_getter(&sobec::ContactData3D::a,
                          bp::return_value_policy<bp::return_by_value>()),
          "spatial acceleration of the contact body")
      .add_property("fJf",
                    bp::make_getter(&sobec::ContactData3D::fJf,
                                    bp::return_internal_reference<>()),
                    "local Jacobian of the contact frame")
      .add_property("v_partial_dq",
                    bp::make_getter(&sobec::ContactData3D::v_partial_dq,
                                    bp::return_internal_reference<>()),
                    "Jacobian of the spatial body velocity")
      .add_property("a_partial_dq",
                    bp::make_getter(&sobec::ContactData3D::a_partial_dq,
                                    bp::return_internal_reference<>()),
                    "Jacobian of the spatial body acceleration")
      .add_property("a_partial_dv",
                    bp::make_getter(&sobec::ContactData3D::a_partial_dv,
                                    bp::return_internal_reference<>()),
                    "Jacobian of the spatial body acceleration")
      .add_property("a_partial_da",
                    bp::make_getter(&sobec::ContactData3D::a_partial_da,
                                    bp::return_internal_reference<>()),
                    "Jacobian of the spatial body acceleration")
      .add_property(
          "oRf",
          bp::make_getter(&sobec::ContactData3D::oRf,
                          bp::return_internal_reference<>()),
          "Rotation matrix of the contact body expressed in the world frame");
}

}  // namespace python
}  // namespace sobec

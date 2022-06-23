///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh, LAAS-CNRS, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/residual-center-of-friction.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeResidualCenterOfFriction() {
  bp::register_ptr_to_python<
      boost::shared_ptr<ResidualModelCenterOfFriction> >();

  bp::class_<ResidualModelCenterOfFriction, bp::bases<ResidualModelAbstract> >(
      "ResidualModelCenterOfFriction",
      bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex,
               std::size_t>(
          bp::args("self", "state", "contact_id", "nu"),
          "Initialize the residual model r(x,u)=center of friction.\n\n"
          ":param state: state of the multibody system\n"
          ":param contact_id: reference contact frame\n"
          ":param nu: dimension of control vector"))
      .def<void (ResidualModelCenterOfFriction::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelCenterOfFriction::calc,
          bp::args("self", "data", "x", "u"),
          "Compute the vel collision residual.\n\n"
          ":param data: residual data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelCenterOfFriction::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelCenterOfFriction::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelCenterOfFriction::calcDiff,
          bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the vel collision residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelCenterOfFriction::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff,
          bp::args("self", "data", "x"))
      .def("createData", &ResidualModelCenterOfFriction::createData,
           bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the residual data.\n\n"
           ":param data: shared data\n"
           ":return residual data.")
      .add_property(
          "contact_id", &ResidualModelCenterOfFriction::get_contact_id,
          &ResidualModelCenterOfFriction::set_contact_id, "Contact frame ID");

  bp::register_ptr_to_python<
      boost::shared_ptr<ResidualDataCenterOfFriction> >();

  bp::class_<ResidualDataCenterOfFriction, bp::bases<ResidualDataAbstract> >(
      "ResidualDataCenterOfFriction", "Data for vel collision residual.\n\n",
      bp::init<ResidualModelCenterOfFriction*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create Center of friction residual data.\n\n"
          ":param model: residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3> >()]);
}

}  // namespace python
}  // namespace sobec

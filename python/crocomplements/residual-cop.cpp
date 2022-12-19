///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh, LAAS-CNRS, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/crocomplements/residual-cop.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeResidualCenterOfPressure() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelCenterOfPressure> >();

  bp::class_<ResidualModelCenterOfPressure, bp::bases<ResidualModelAbstract> >(
      "ResidualModelCenterOfPressure", bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex, std::size_t>(
                                           bp::args("self", "state", "contact_id", "nu"),
                                           "Initialize the residual model r(x,u)=cop.\n\n"
                                           ":param state: state of the multibody system\n"
                                           ":param contact_id: reference contact frame\n"
                                           ":param nu: dimension of control vector"))
      .def<void (ResidualModelCenterOfPressure::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                                   const Eigen::Ref<const Eigen::VectorXd>&,
                                                   const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelCenterOfPressure::calc, bp::args("self", "data", "x", "u"),
          "Compute the vel collision residual.\n\n"
          ":param data: residual data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelCenterOfPressure::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                                   const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelCenterOfPressure::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                                   const Eigen::Ref<const Eigen::VectorXd>&,
                                                   const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelCenterOfPressure::calcDiff, bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the vel collision residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelCenterOfPressure::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                                   const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff, bp::args("self", "data", "x"))
      .def("createData", &ResidualModelCenterOfPressure::createData, bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the residual data.\n\n"
           ":param data: shared data\n"
           ":return residual data.")
      .add_property("contact_id", &ResidualModelCenterOfPressure::get_contact_id,
                    &ResidualModelCenterOfPressure::set_contact_id, "Contact frame ID");

  bp::register_ptr_to_python<boost::shared_ptr<ResidualDataCenterOfPressure> >();

  bp::class_<ResidualDataCenterOfPressure, bp::bases<ResidualDataAbstract> >(
      "ResidualDataCenterOfPressure", "Data for vel collision residual.\n\n",
      bp::init<ResidualModelCenterOfPressure*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create COP residual data.\n\n"
          ":param model: cop residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<1, 2, bp::with_custodian_and_ward<1, 3> >()]);
}

}  // namespace python
}  // namespace sobec

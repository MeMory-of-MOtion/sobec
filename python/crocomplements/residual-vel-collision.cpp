///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh, LAAS-CNRS, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifdef PINOCCHIO_WITH_HPP_FCL

#include "sobec/crocomplements/residual-vel-collision.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeResidualVelCollision() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelVelCollision> >();

  bp::class_<ResidualModelVelCollision, bp::bases<ResidualModelAbstract> >(
      "ResidualModelVelCollision",
      bp::init<boost::shared_ptr<StateMultibody>, std::size_t, boost::shared_ptr<pinocchio::GeometryModel>,
               pinocchio::PairIndex, pinocchio::FrameIndex, pinocchio::ReferenceFrame, double>(
          bp::args("self", "state", "nu", "geom_model", "pair_id", "frame_id", "type", "beta"),
          "Initialize the pair collision residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param nu: dimension of control vector\n"
          ":param geom_model: geometric model of the multibody system\n"
          ":param pair_id: id of the pair of colliding objects\n"
          ":param frame_id: reference colliding frame\n"
          ":param type: reference type of velocity\n"
          ":param beta: small parameter to avoid division by zero"))
      .def<void (ResidualModelVelCollision::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                               const Eigen::Ref<const Eigen::VectorXd>&,
                                               const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelVelCollision::calc, bp::args("self", "data", "x", "u"),
          "Compute the vel collision residual.\n\n"
          ":param data: residual data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelVelCollision::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                               const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelVelCollision::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                               const Eigen::Ref<const Eigen::VectorXd>&,
                                               const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelVelCollision::calcDiff, bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the vel collision residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelVelCollision::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                               const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff, bp::args("self", "data", "x"))
      .def("createData", &ResidualModelVelCollision::createData, bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the pair collision residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for a predefined residual.\n"
           ":param data: shared data\n"
           ":return residual data.");

  bp::register_ptr_to_python<boost::shared_ptr<ResidualDataVelCollision> >();

  bp::class_<ResidualDataVelCollision, bp::bases<ResidualDataAbstract> >(
      "ResidualDataVelCollision", "Data for vel collision residual.\n\n",
      bp::init<ResidualModelVelCollision*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create vel collision residual data.\n\n"
          ":param model: pair collision residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<1, 2, bp::with_custodian_and_ward<1, 3> >()])
      .add_property("pinocchio",
                    bp::make_getter(&ResidualDataVelCollision::pinocchio, bp::return_internal_reference<>()),
                    "pinocchio data")
      .add_property("V", bp::make_getter(&ResidualDataVelCollision::V, bp::return_internal_reference<>()),
                    "Planar velocity of reference frame")
      .add_property("e", bp::make_getter(&ResidualDataVelCollision::e, bp::return_internal_reference<>()),
                    "Distance between collision pair")
      .add_property("Vx", bp::make_getter(&ResidualDataVelCollision::Vx, bp::return_internal_reference<>()),
                    "Planar velocity derivative of reference frame")
      .add_property("geometry",
                    bp::make_getter(&ResidualDataVelCollision::geometry, bp::return_internal_reference<>()),
                    "pinocchio geometry data");
}

}  // namespace python
}  // namespace sobec

#endif  // #ifdef PINOCCHIO_WITH_HPP_FCL

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/crocomplements/residual-2D-surface.hpp"

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

void exposeResidual2DSurface() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModel2DSurface> >();

  bp::class_<ResidualModel2DSurface, bp::bases<ResidualModelAbstract> >(
      "ResidualModel2DSurface",
      "This residual function defines the inequalities to keep the effector "
      "into a given 2D surface placed with respect to the opposite effector"
      "r = A * translation.head(2) ",
      bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex,
               Eigen::Vector2d, double, double, double, std::size_t>(
          bp::args("self", "state", "frame_id", "support_translation",
                   "separation", "orientation", "nu"),
          "Initialize the 2D surface residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param frame_id: reference frame\n"
          ":param support_translation: position of the opposite foot\n"
          ":param separation: distance separating the surface and the opposite "
          "foot\n"
          ":param orientation: yaw of the opposite foot\n"
          ":param alpha: orientation of cone limits\n"
          ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex,
                    Eigen::Vector2d, double, double, double>(
          bp::args("self", "state", "frame_id", "support_translation",
                   "separation", "orientation"),
          "Initialize the 2D surface residual model.\n\n"
          "The default nu is obtained from state.nv.\n"
          ":param state: state of the multibody system\n"
          ":param frame_id: reference frame\n"
          ":param support_translation: position of the opposite foot\n"
          ":param separation: distance separating the surface and the opposite "
          "foot\n"
          ":param orientation: yaw of the opposite foot\n"
          ":param alpha: orientation of cone limits"))
      .def<void (ResidualModel2DSurface::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModel2DSurface::calc,
          bp::args("self", "data", "x", "u"),
          "Compute the 2D surface residual.\n\n"
          ":param data: residual data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModel2DSurface::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModel2DSurface::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModel2DSurface::calcDiff,
          bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the 2D surface residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModel2DSurface::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff,
          bp::args("self", "data", "x"))
      .def("createData", &ResidualModel2DSurface::createData,
           bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the 2D surface residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for a predefined residual.\n"
           ":param data: shared data\n"
           ":return residual data.")
      .def("set_Ab", &ResidualModel2DSurface::set_Ab,
           bp::args("self", "support_translation", "orientation"))
      .add_property("A",
                    bp::make_function(&ResidualModel2DSurface::get_A,
                                      bp::return_internal_reference<>()),
                    &ResidualModel2DSurface::set_A, "Inequality matrix")
      .add_property("b",
                    bp::make_function(&ResidualModel2DSurface::get_b,
                                      bp::return_internal_reference<>()),
                    &ResidualModel2DSurface::set_b, "Inequality vector");

  bp::register_ptr_to_python<boost::shared_ptr<ResidualData2DSurface> >();

  bp::class_<ResidualData2DSurface, bp::bases<ResidualDataAbstract> >(
      "ResidualData2DSurface", "Data for 2D surface residual.\n\n",
      bp::init<ResidualModel2DSurface*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create 2D surface residual data.\n\n"
          ":param model: 2D surface residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3> >()])
      .add_property("pinocchio",
                    bp::make_getter(&ResidualData2DSurface::pinocchio,
                                    bp::return_internal_reference<>()),
                    "pinocchio data");
}

}  // namespace python
}  // namespace sobec

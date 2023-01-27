#include <pinocchio/multibody/fwd.hpp>  // Must be included first!
#include <sstream>

// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/walk-with-traj/wbc_hand.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;

void initialize(WBCHand &self, const bp::dict &settings,
                const RobotDesigner &designer, const HorizonManager &horizon,
                const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
                const std::string &actuationCostName) {
  WBCHandSettings conf;

  conf.T = bp::extract<int>(settings["T"]);
  conf.Tduration = bp::extract<int>(settings["Tduration"]);
  conf.ddpIteration = bp::extract<int>(settings["ddpIteration"]);
  conf.Dt = bp::extract<double>(settings["Dt"]);
  conf.simu_step = bp::extract<double>(settings["simu_step"]);
  conf.Nc = bp::extract<int>(settings["Nc"]);

  self.initialize(conf, designer, horizon, q0, v0, actuationCostName);
}

template <typename T>
boost::shared_ptr<std::vector<T>> constructVectorFromList(const bp::list &in) {
  boost::shared_ptr<std::vector<T>> ptr = boost::make_shared<std::vector<T>>();
  ptr->resize(bp::len(in));
  for (int i = 0; i < bp::len(in); ++i) {
    (*ptr)[i] = boost::python::extract<T>(in[i]);
  }
  return ptr;
}

template <typename T>
std::string displayVector(std::vector<T> &self) {
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < self.size(); ++i) {
    oss << self[i] << ", ";
  }
  oss << std::endl;
  return oss.str();
}

bool timeToSolveDDP(WBCHand &self, const int iteration) {
  return self.timeToSolveDDP(iteration);
}

void exposeWBCHand() {
  bp::class_<std::vector<pinocchio::SE3>>("vector_pinocchio_se3_")
      .def(bp::vector_indexing_suite<std::vector<pinocchio::SE3>>())
      .def("__init__",
           make_constructor(constructVectorFromList<pinocchio::SE3>))
      .def("__repr__", &displayVector<pinocchio::SE3>);

  bp::class_<std::vector<eVector3>>("vector_eigen_vector3_")
      .def(bp::vector_indexing_suite<std::vector<eVector3>>())
      .def("__init__", make_constructor(constructVectorFromList<eVector3>))
      .def("__repr__", &displayVector<eVector3>);

  bp::class_<WBCHand>("WBCHand", bp::init<>())
      .def("initialize", &initialize,
           bp::args("self", "settings", "design", "horizon", "q0", "v0",
                    "actuationCostName"),
           "The posture required here is the full robot posture in the order "
           "of pinocchio")
      .def("shapeState",
           bp::make_function(
               &WBCHand::shapeState,
               bp::return_value_policy<
                   bp::reference_existing_object>()))  //, bp::args("self", "q",
                                                       //"v")
      .def("generateFullHorizon", &WBCHand::generateFullHorizon,
           bp::args("self", "modelMaker"))
      .def("timeToSolveDDP", &timeToSolveDDP, bp::args("self", "iteration"))
      .def("iterate",
           static_cast<void (WBCHand::*)(const int, const Eigen::VectorXd &,
                                     const Eigen::VectorXd &, const bool)>(
               &WBCHand::iterate),
           (bp::arg("self"), bp::arg("iteration"), bp::arg("q_current"),
            bp::arg("v_current"), bp::arg("is_feasible") = false))
      .def("iterate",
           static_cast<void (WBCHand::*)(const Eigen::VectorXd &,
                                     const Eigen::VectorXd &, const bool)>(
               &WBCHand::iterate),
           (bp::arg("self"), bp::arg("q_current"), bp::arg("v_current"),
            bp::arg("is_feasible") = false))
      .def<void (WBCHand::*)()>("recedeWithCycle", &WBCHand::recedeWithCycle,
                            bp::args("self"))
      .def("iteration",
           make_function(
               &WBCHand::get_iteration,
               bp::return_value_policy<bp::copy_const_reference>()))
      .add_property(
          "x0",
          bp::make_function(
              &WBCHand::get_x0,
              bp::return_value_policy<bp::reference_existing_object>()),
          &WBCHand::set_x0)
      .add_property(
          "fullHorizon",
          bp::make_function(
              &WBCHand::get_fullHorizon,
              bp::return_value_policy<bp::reference_existing_object>()),
          &WBCHand::set_fullHorizon)
      .add_property(
          "horizon",
          bp::make_function(
              &WBCHand::get_horizon,
              bp::return_value_policy<bp::reference_existing_object>()),
          &WBCHand::set_horizon)
      .add_property(
          "designer",
          bp::make_function(
              &WBCHand::get_designer,
              bp::return_value_policy<bp::reference_existing_object>()),
          &WBCHand::set_designer)
      .add_property(
          "ref_hand_pose",
          bp::make_function(
              &WBCHand::ref_hand_pose,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHand::*)(const eVector3 &)>(
              &WBCHand::setPoseRefHand))
      .add_property(
          "ref_com",
          bp::make_function(
              &WBCHand::ref_com,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHand::*)(eVector3)>(
              &WBCHand::setCoMRef));
}
}  // namespace python
}  // namespace sobec

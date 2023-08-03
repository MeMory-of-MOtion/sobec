#include <pinocchio/multibody/fwd.hpp>  // Must be included first!
#include <sstream>

// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/hand_table/wbc_hand.hpp>

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
  conf.TtrackingToContact = bp::extract<int>(settings["TtrackingToContact"]);
  conf.Tcontact = bp::extract<int>(settings["Tcontact"]);
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
      .def("generateFullHorizon", &WBCHand::generateFullHorizon,
           bp::args("self", "modelMakerHand"))
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
      .def("land_hand",
           make_function(
               &WBCHand::get_land_hand,
               bp::return_value_policy<bp::copy_const_reference>()))
      .def("takeoff_hand",
           make_function(
               &WBCHand::get_takeoff_hand,
               bp::return_value_policy<bp::copy_const_reference>()))
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
          "ref_LH_pose",
          bp::make_function(
              &WBCHand::ref_LH_pose,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHand::*)(const eVector3 &)>(
              &WBCHand::setPoseRef_LH))
      .add_property(
          "ref_RH_pose",
          bp::make_function(
              &WBCHand::ref_RH_pose,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHand::*)(const eVector3 &)>(
              &WBCHand::setPoseRef_RH))
      .add_property(
          "ref_com",
          bp::make_function(
              &WBCHand::ref_com,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHand::*)(eVector3)>(
              &WBCHand::setCoMRef))
      .add_property(
          "ref_com_vel",
          bp::make_function(
              &WBCHand::ref_com_vel,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHand::*)(eVector3)>(
              &WBCHand::setVelRef_COM))
      .add_property(
          "ref_force",
          bp::make_function(
              &WBCHand::ref_force,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHand::*)(const pinocchio::Force &)>(
              &WBCHand::setForceRef));
}
}  // namespace python
}  // namespace sobec

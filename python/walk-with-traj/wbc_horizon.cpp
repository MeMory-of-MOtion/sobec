#include <pinocchio/multibody/fwd.hpp>  // Must be included first!
#include <sstream>

// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/walk-with-traj/wbc_horizon.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;

void initialize(WBCHorizon &self, const bp::dict &settings,
                const RobotDesigner &designer, const HorizonManager &horizon,
                const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
                const std::string &actuationCostName) {
  WBCHorizonSettings conf;

  conf.totalSteps = bp::extract<int>(settings["totalSteps"]);
  conf.T = bp::extract<int>(settings["T"]);
  conf.TdoubleSupport = bp::extract<int>(settings["TdoubleSupport"]);
  conf.TsingleSupport = bp::extract<int>(settings["TsingleSupport"]);
  conf.Tstep = bp::extract<int>(settings["Tstep"]);
  conf.ddpIteration = bp::extract<int>(settings["ddpIteration"]);
  conf.Dt = bp::extract<double>(settings["Dt"]);
  conf.simu_step = bp::extract<double>(settings["simu_step"]);
  conf.min_force = bp::extract<double>(settings["min_force"]);
  conf.support_force = bp::extract<double>(settings["support_force"]);
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

bool timeToSolveDDP(WBCHorizon &self, const int iteration) {
  return self.timeToSolveDDP(iteration);
}

void exposeWBCHorizon() {
  bp::class_<std::vector<pinocchio::SE3>>("vector_pinocchio_se3_")
      .def(bp::vector_indexing_suite<std::vector<pinocchio::SE3>>())
      .def("__init__",
           make_constructor(constructVectorFromList<pinocchio::SE3>))
      .def("__repr__", &displayVector<pinocchio::SE3>);

  bp::class_<std::vector<eVector3>>("vector_eigen_vector3_")
      .def(bp::vector_indexing_suite<std::vector<eVector3>>())
      .def("__init__", make_constructor(constructVectorFromList<eVector3>))
      .def("__repr__", &displayVector<eVector3>);

  bp::class_<WBCHorizon>("WBCHorizon", bp::init<>())
      .def("initialize", &initialize,
           bp::args("self", "settings", "design", "horizon", "q0", "v0",
                    "actuationCostName"),
           "The posture required here is the full robot posture in the order "
           "of pinocchio")
      .def("shapeState",
           bp::make_function(
               &WBCHorizon::shapeState,
               bp::return_value_policy<
                   bp::reference_existing_object>()))  //, bp::args("self", "q",
                                                       //"v")
      .def("generateFullHorizon", &WBCHorizon::generateFullHorizon,
           (bp::args("self"), bp::arg("modelMaker"), bp::arg("experiment")))
      .def("timeToSolveDDP", &timeToSolveDDP, bp::args("self", "iteration"))
      .def("iterate",
           static_cast<void (WBCHorizon::*)(const int, const Eigen::VectorXd &,
                                            const Eigen::VectorXd &,
                                            const bool)>(&WBCHorizon::iterate),
           (bp::arg("self"), bp::arg("iteration"), bp::arg("q_current"),
            bp::arg("v_current"), bp::arg("is_feasible") = false))
      .def("iterate",
           static_cast<void (WBCHorizon::*)(
               const Eigen::VectorXd &, const Eigen::VectorXd &, const bool)>(
               &WBCHorizon::iterate),
           (bp::arg("self"), bp::arg("q_current"), bp::arg("v_current"),
            bp::arg("is_feasible") = false))
      .def("iterateNoThinking",
           static_cast<void (WBCHorizon::*)(
               const int, const Eigen::VectorXd &, const Eigen::VectorXd &,
               const bool)>(&WBCHorizon::iterateNoThinking),
           (bp::arg("self"), bp::arg("iteration"), bp::arg("q_current"),
            bp::arg("v_current"), bp::arg("is_feasible") = false))
      .def("iterateNoThinking",
           static_cast<void (WBCHorizon::*)(
               const Eigen::VectorXd &, const Eigen::VectorXd &, const bool)>(
               &WBCHorizon::iterateNoThinking),
           (bp::arg("self"), bp::arg("q_current"), bp::arg("v_current"),
            bp::arg("is_feasible") = false))
      .def(
          "iterateNoThinkingWithDelay",
          static_cast<void (WBCHorizon::*)(
              const Eigen::VectorXd &, const Eigen::VectorXd &, const bool,
              const bool, const bool)>(&WBCHorizon::iterateNoThinkingWithDelay),
          (bp::arg("self"), bp::arg("q_current"), bp::arg("v_current"),
           bp::arg("contact_left"), bp::arg("contact_right"),
           bp::arg("is_feasible") = false))
      .def<void (WBCHorizon::*)()>(
          "recedeWithCycle", &WBCHorizon::recedeWithCycle, bp::args("self"))
      .def<void (WBCHorizon::*)()>("goToNextDoubleSupport",
                                   &WBCHorizon::goToNextDoubleSupport,
                                   bp::args("self"))
      .add_property(
          "x0",
          bp::make_function(
              &WBCHorizon::get_x0,
              bp::return_value_policy<bp::reference_existing_object>()),
          &WBCHorizon::set_x0)
      .add_property(
          "fullHorizon",
          bp::make_function(
              &WBCHorizon::get_fullHorizon,
              bp::return_value_policy<bp::reference_existing_object>()),
          &WBCHorizon::set_fullHorizon)
      .add_property(
          "horizon",
          bp::make_function(
              &WBCHorizon::get_horizon,
              bp::return_value_policy<bp::reference_existing_object>()),
          &WBCHorizon::set_horizon)
      .add_property(
          "designer",
          bp::make_function(
              &WBCHorizon::get_designer,
              bp::return_value_policy<bp::reference_existing_object>()),
          &WBCHorizon::set_designer)
      .add_property(
          "ref_LF_poses",
          bp::make_function(
              &WBCHorizon::ref_LF_poses,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHorizon::*)(
              const std::vector<pinocchio::SE3> &)>(&WBCHorizon::setPoseRef_LF))
      .add_property(
          "ref_RF_poses",
          bp::make_function(
              &WBCHorizon::ref_RF_poses,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHorizon::*)(
              const std::vector<pinocchio::SE3> &)>(&WBCHorizon::setPoseRef_RF))
      .add_property(
          "ref_com",
          bp::make_function(
              &WBCHorizon::ref_com,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHorizon::*)(eVector3)>(&WBCHorizon::setCoMRef))
      .add_property(
          "ref_com_vel",
          bp::make_function(
              &WBCHorizon::ref_com_vel,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHorizon::*)(eVector3)>(
              &WBCHorizon::setVelRef_COM))
      .add_property(
          "ref_base_rot",
          bp::make_function(
              &WBCHorizon::ref_base_rot,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCHorizon::*)(Eigen::Matrix3d)>(
              &WBCHorizon::setBaseRotRef))
      .def("land_LF",
           make_function(
               &WBCHorizon::get_land_LF,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("land_RF",
           make_function(
               &WBCHorizon::get_land_RF,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("takeoff_LF",
           make_function(
               &WBCHorizon::get_takeoff_LF,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("takeoff_RF",
           make_function(
               &WBCHorizon::get_takeoff_RF,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("horizon_iteration",
           make_function(&WBCHorizon::get_horizon_iteration,
                         bp::return_value_policy<bp::copy_const_reference>()));
}
}  // namespace python
}  // namespace sobec

#include <pinocchio/multibody/fwd.hpp>  // Must be included first!
#include <sstream>

// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/walk-with-traj/wbc_nothinking.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;

void initialize(WBCNoThinking &self, const bp::dict &settings,
                const RobotDesigner &designer, const HorizonManager &horizon,
                const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
                const std::string &actuationCostName) {
  WBCSettings conf;

  conf.horizonSteps = bp::extract<int>(settings["horizonSteps"]);
  conf.totalSteps = bp::extract<int>(settings["totalSteps"]);
  conf.T = bp::extract<int>(settings["T"]);
  conf.TdoubleSupport = bp::extract<int>(settings["TdoubleSupport"]);
  conf.TsingleSupport = bp::extract<int>(settings["TsingleSupport"]);
  conf.Tstep = bp::extract<int>(settings["Tstep"]);
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

void exposeWBCNoThinking() {
  bp::enum_<LocomotionType>("LocomotionType")
      .value("WALKING", LocomotionType::WALKING)
      .value("STANDING", LocomotionType::STANDING);
  bp::class_<std::vector<pinocchio::SE3>>("vector_pinocchio_se3_")
      .def(bp::vector_indexing_suite<std::vector<pinocchio::SE3>>())
      .def("__init__",
           make_constructor(constructVectorFromList<pinocchio::SE3>))
      .def("__repr__", &displayVector<pinocchio::SE3>);

  bp::class_<std::vector<eVector3>>("vector_eigen_vector3_")
      .def(bp::vector_indexing_suite<std::vector<eVector3>>())
      .def("__init__", make_constructor(constructVectorFromList<eVector3>))
      .def("__repr__", &displayVector<eVector3>);

  bp::class_<WBCNoThinking>("WBCNoThinking", bp::init<>())
      .def("initialize", &initialize,
           bp::args("self", "settings", "design", "horizon", "q0", "v0",
                    "actuationCostName"),
           "The posture required here is the full robot posture in the order "
           "of pinocchio")
      .def("iterate",
           static_cast<void (WBCNoThinking::*)(const Eigen::VectorXd &,
                                     const Eigen::VectorXd &, const bool)>(
               &WBCNoThinking::iterate),
           (bp::arg("self"), bp::arg("q_current"), bp::arg("v_current"),
            bp::arg("is_feasible") = false))
      .add_property(
          "ref_com",
          bp::make_function(
              &WBCNoThinking::ref_com,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCNoThinking::*)(eVector3)>(
              &WBCNoThinking::setCoMRef))
      .add_property(
          "ref_com_vel",
          bp::make_function(
              &WBCNoThinking::ref_com_vel,
              bp::return_value_policy<bp::reference_existing_object>()),
          static_cast<void (WBCNoThinking::*)(eVector3)>(
              &WBCNoThinking::setVelRef_COM));
}
}  // namespace python
}  // namespace sobec

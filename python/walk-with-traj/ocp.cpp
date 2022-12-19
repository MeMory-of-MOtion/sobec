#include "sobec/ocp.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;
template <typename T>
inline void py_list_to_std_vector(const bp::object &iterable, std::vector<T> &out) {
  out = std::vector<T>(boost::python::stl_input_iterator<T>(iterable), boost::python::stl_input_iterator<T>());
}

void initialize(OCP &self, const bp::dict &settings, const bp::dict &model_settings, const bp::dict &design,
                const Eigen::VectorXd &q0, const Eigen::VectorXd &v0) {
  OCPSettings conf;

  conf.totalSteps = bp::extract<int>(settings["totalSteps"]);
  conf.T = bp::extract<int>(settings["T"]);
  conf.TdoubleSupport = bp::extract<int>(settings["TdoubleSupport"]);
  conf.TsimpleSupport = bp::extract<int>(settings["TsimpleSupport"]);
  conf.Tstep = bp::extract<int>(settings["Tstep"]);
  conf.ddpIteration = bp::extract<int>(settings["ddpIteration"]);
  conf.Dt = bp::extract<double>(settings["Dt"]);
  conf.simu_step = bp::extract<double>(settings["simu_step"]);
  conf.Nc = bp::extract<int>(settings["Nc"]);
  conf.stepSize = bp::extract<double>(settings["stepSize"]);
  conf.stepHeight = bp::extract<double>(settings["stepHeight"]);
  conf.stepDepth = bp::extract<double>(settings["stepDepth"]);
  conf.stepYCorrection = bp::extract<double>(settings["stepYCorrection"]);

  ModelMakerSettings model_conf;

  // timing
  model_conf.timeStep = bp::extract<double>(model_settings["timeStep"]);

  // physics
  model_conf.gravity = bp::extract<eVector3>(model_settings["gravity"]);
  model_conf.mu = bp::extract<double>(model_settings["mu"]);
  model_conf.coneBox = bp::extract<eVector2>(model_settings["coneBox"]);
  model_conf.minNforce = bp::extract<double>(model_settings["minNforce"]);
  model_conf.maxNforce = bp::extract<double>(model_settings["maxNforce"]);
  model_conf.comHeight = bp::extract<double>(model_settings["comHeight"]);
  model_conf.omega = bp::extract<double>(model_settings["omega"]);

  // gains
  model_conf.wFootPlacement = bp::extract<double>(model_settings["wFootPlacement"]);
  model_conf.wStateReg = bp::extract<double>(model_settings["wStateReg"]);
  model_conf.wControlReg = bp::extract<double>(model_settings["wControlReg"]);
  model_conf.wLimit = bp::extract<double>(model_settings["wLimit"]);
  model_conf.wVCoM = bp::extract<double>(model_settings["wVCoM"]);
  model_conf.wWrenchCone = bp::extract<double>(model_settings["wWrenchCone"]);
  model_conf.wFootTrans = bp::extract<double>(model_settings["wFootTrans"]);
  model_conf.wFootXYTrans = bp::extract<double>(model_settings["wFootXYTrans"]);
  model_conf.wFootRot = bp::extract<double>(model_settings["wFootRot"]);
  model_conf.wGroundCol = bp::extract<double>(model_settings["wGroundCol"]);
  model_conf.stateWeights = bp::extract<Eigen::VectorXd>(model_settings["stateWeights"]);
  model_conf.controlWeights = bp::extract<Eigen::VectorXd>(model_settings["controlWeights"]);
  model_conf.th_grad = bp::extract<double>(model_settings["th_grad"]);
  model_conf.th_stop = bp::extract<double>(model_settings["th_stop"]);

  RobotDesignerSettings robot_conf;
  robot_conf.urdfPath = bp::extract<std::string>(design["urdfPath"]);
  robot_conf.srdfPath = bp::extract<std::string>(design["srdfPath"]);
  robot_conf.leftFootName = bp::extract<std::string>(design["leftFootName"]);
  robot_conf.rightFootName = bp::extract<std::string>(design["rightFootName"]);
  robot_conf.robotDescription = bp::extract<std::string>(design["robotDescription"]);
  py_list_to_std_vector(design["controlledJointsNames"], robot_conf.controlledJointsNames);

  self.initialize(conf, model_conf, robot_conf, q0, v0);
}
HorizonManager get_horizon(OCP &self) { return self.get_horizon(); }

void exposeOCP() {
  bp::class_<OCP>("OCP", bp::init<>())
      .def("initialize", &initialize, bp::args("self", "settings", "model_settings", "design", "q0", "v0"),
           "The posture required here is the full robot posture in the order "
           "of pin0cchio")
      .def("updateEndPhase", &OCP::updateEndPhase, bp::args("self"))
      .def("updateOCP", &OCP::updateOCP, (bp::arg("self"), bp::arg("q_current"), bp::arg("v_current")))
      .def("get_horizon", &OCP::get_horizon, bp::args("self"));
}

}  // namespace python
}  // namespace sobec

#include <pinocchio/fwd.hpp>  // Must be included first!
// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/hand_table/model_factory_hand.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;

template <typename T>
inline void py_list_to_std_vector(const bp::object &iterable,
                                  std::vector<T> &out) {
  out = std::vector<T>(boost::python::stl_input_iterator<T>(iterable),
                       boost::python::stl_input_iterator<T>());
}

template <class T>
bp::list std_vector_to_py_list(const std::vector<T> &v) {
  bp::object get_iter = bp::iterator<std::vector<T> >();
  bp::object iter = get_iter(v);
  bp::list l(iter);
  return l;
}

void initialize(ModelMakerHand &self, const bp::dict &settings,
                const RobotDesigner &designer) {
  ModelMakerHandSettings conf;

  // timing
  conf.timeStep = bp::extract<double>(settings["timeStep"]);

  // physics
  conf.gravity = bp::extract<eVector3>(settings["gravity"]);
  conf.comHeight = bp::extract<double>(settings["comHeight"]);
  conf.omega = bp::extract<double>(settings["omega"]);
  conf.obstacleRadius = bp::extract<double>(settings["obstacleRadius"]);
  conf.obstaclePosition = bp::extract<eVector3>(settings["obstaclePosition"]);
  conf.obstacleHeight = bp::extract<double>(settings["obstacleHeight"]);
  
  // gains
  conf.wHandTranslation = bp::extract<double>(settings["wHandTranslation"]);
  conf.wHandRotation = bp::extract<double>(settings["wHandRotation"]);
  conf.wHandVelocity = bp::extract<double>(settings["wHandVelocity"]);
  conf.wHandCollision = bp::extract<double>(settings["wHandCollision"]);
  conf.wStateReg = bp::extract<double>(settings["wStateReg"]);
  conf.wControlReg = bp::extract<double>(settings["wControlReg"]);
  conf.wLimit = bp::extract<double>(settings["wLimit"]);
  conf.wForceHand = bp::extract<double>(settings["wForceHand"]);
  conf.wFrictionHand = bp::extract<double>(settings["wFrictionHand"]);
  conf.wDCM = bp::extract<double>(settings["wDCM"]);
  conf.wCoM = bp::extract<double>(settings["wCoM"]);
  conf.stateWeights = bp::extract<Eigen::VectorXd>(settings["stateWeights"]);
  conf.controlWeights = bp::extract<Eigen::VectorXd>(settings["controlWeights"]);
  conf.lowKinematicLimits = bp::extract<Eigen::VectorXd>(settings["lowKinematicLimits"]);
  conf.highKinematicLimits = bp::extract<Eigen::VectorXd>(settings["highKinematicLimits"]);
  conf.th_grad = bp::extract<double>(settings["th_grad"]);
  conf.th_stop = bp::extract<double>(settings["th_stop"]);

  self.initialize(conf, designer);
}

bp::dict get_settings(ModelMakerHand &self) {
  ModelMakerHandSettings conf = self.get_settings();
  bp::dict settings;
  settings["timeStep"] = conf.timeStep;
  settings["gravity"] = conf.gravity;
  settings["comHeight"] = conf.comHeight;
  settings["omega"] = conf.omega;
  settings["obstacleRadius"] = conf.obstacleRadius;
  settings["obstaclePosition"] = conf.obstaclePosition;
  settings["obstacleHeight"] = conf.obstacleHeight;
  settings["wHandTranslation"] = conf.wHandTranslation;
  settings["wHandRotation"] = conf.wHandRotation;
  settings["wHandVelocity"] = conf.wHandVelocity;
  settings["wHandCollision"] = conf.wHandCollision;
  settings["wStateReg"] = conf.wStateReg;
  settings["wControlReg"] = conf.wControlReg;
  settings["wLimit"] = conf.wLimit;
  settings["wForceHand"] = conf.wForceHand;
  settings["wFrictionHand"] = conf.wFrictionHand;
  settings["wDCM"] = conf.wDCM;
  settings["wCoM"] = conf.wCoM;
  settings["stateWeights"] = conf.stateWeights;
  settings["controlWeights"] = conf.controlWeights;
  settings["lowKinematicLimits"] = conf.lowKinematicLimits;
  settings["highKinematicLimits"] = conf.highKinematicLimits;
  settings["th_grad"] = conf.th_grad;
  settings["th_stop"] = conf.th_stop;
  return settings;
}

bp::list formulateHorizon(ModelMakerHand &self,
                          const bp::list &phases = bp::list(),
                          const int &length = 0) {
  if (bp::len(phases) > 0) {
    std::vector<Phase> contacts;
    py_list_to_std_vector(phases, contacts);
    std::vector<AMA> models = self.formulateHorizon(contacts);

    return std_vector_to_py_list(models);

  } else if (length > 0) {
    std::vector<AMA> models = self.formulateHorizon(length);
    return std_vector_to_py_list(models);
  } else {
    throw std::runtime_error(
        "Either a list of phases or an horizon length must be provided.");
  }
}

void defineFeetContact(ModelMakerHand &self,
                       crocoddyl::ContactModelMultiple &contactCollector) {
  Contact contacts =
      boost::make_shared<crocoddyl::ContactModelMultiple>(contactCollector);
  self.defineFeetContact(contacts);
  contactCollector = *contacts;
}

void defineHandContact(ModelMakerHand &self,
                       crocoddyl::ContactModelMultiple &contactCollector,
                       const Phase &phases = Phase::CONTACT_RIGHT) {
  Contact contacts =
      boost::make_shared<crocoddyl::ContactModelMultiple>(contactCollector);
  self.defineHandContact(contacts, phases);
  contactCollector = *contacts;
}

void defineHandForceTask(ModelMakerHand &self,
                          crocoddyl::CostModelSum &costCollector,
                          const Phase &phases = Phase::NO_HAND) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineHandForceTask(costs, phases);
  costCollector = *costs;
}

void defineHandFrictionTask(ModelMakerHand &self,
                          crocoddyl::CostModelSum &costCollector,
                          const Phase &phases = Phase::NO_HAND) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineHandFrictionTask(costs, phases);
  costCollector = *costs;
}

void defineHandTranslation(ModelMakerHand &self,
                        crocoddyl::CostModelSum &costCollector,
                        const Phase &phases = Phase::CONTACT_RIGHT) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineHandTranslation(costs, phases);
  costCollector = *costs;
}

void defineHandCollisionTask(ModelMakerHand &self,
                        crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineHandCollisionTask(costs);
  costCollector = *costs;
}

void defineHandRotation(ModelMakerHand &self,
                        crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineHandRotation(costs);
  costCollector = *costs;
}

void defineHandVelocity(ModelMakerHand &self,
                        crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineHandVelocity(costs);
  costCollector = *costs;
}

void defineDCMTask(ModelMakerHand &self,
                        crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineDCMTask(costs);
  costCollector = *costs;
}

void definePostureTask(ModelMakerHand &self,
                       crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.definePostureTask(costs);
  costCollector = *costs;
}

void defineActuationTask(ModelMakerHand &self,
                         crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineActuationTask(costs);
  costCollector = *costs;
}

void defineJointLimits(ModelMakerHand &self,
                       crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineJointLimits(costs);
  costCollector = *costs;
}

void exposeModelFactoryHand() {
  bp::enum_<Phase>("Phase")
      .value("TRACKING_LEFT", Phase::TRACKING_LEFT)
      .value("TRACKING_RIGHT", Phase::TRACKING_RIGHT)
      .value("NO_HAND", Phase::NO_HAND)
      .value("CONTACT_LEFT", Phase::CONTACT_LEFT)
      .value("CONTACT_RIGHT", Phase::CONTACT_RIGHT);

  bp::class_<ModelMakerHand>("ModelMakerHand", bp::init<>())
      .def("initialize", &initialize, bp::args("self", "settings", "design"))
      .def("get_settings", &get_settings, bp::args("self"))
      .def("defineFeetContact", &defineFeetContact,
           bp::arg("self"), bp::arg("costCollector"))
      .def("defineHandContact", &defineHandContact,
           (bp::arg("self"), bp::arg("contactCollector"),
            bp::arg("phases") = Phase::CONTACT_RIGHT))
      .def("defineHandForceTask", &defineHandForceTask,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("phases") = Phase::NO_HAND))
      .def("defineHandFrictionTask", &defineHandFrictionTask,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("phases") = Phase::NO_HAND))
      .def("defineHandTranslation", &defineHandTranslation,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("phases") = Phase::CONTACT_RIGHT))
      .def("defineHandCollisionTask", &defineHandCollisionTask,
           bp::arg("self"), bp::arg("costCollector"))
      .def("defineHandRotation", &defineHandRotation,
           bp::arg("self"), bp::arg("costCollector"))
      .def("defineHandVelocity", &defineHandVelocity,
           bp::arg("self"), bp::arg("costCollector"))
      .def("defineDCMTask", &defineDCMTask,
           bp::arg("self"), bp::arg("costCollector"))
      .def("definePostureTask", &definePostureTask,
           bp::args("self", "costCollector"))
      .def("defineActuationTask", &defineActuationTask,
           bp::args("self", "costCollector"))
      .def("defineJointLimits", &defineJointLimits,
           bp::args("self", "costCollector"))
      .def("formulateColFullTask", &ModelMakerHand::formulateColFullTask, bp::arg("self"))
      .def("formulatePointingTask", &ModelMakerHand::formulatePointingTask, bp::arg("self"))
      .def("formulateTerminalColFullTask", &ModelMakerHand::formulateTerminalColFullTask, bp::arg("self"))
      .def("formulateHandTracker", &ModelMakerHand::formulateHandTracker,
           (bp::arg("self"), bp::arg("phases") = Phase::NO_HAND))
      .def("formulateTerminalHandTracker", &ModelMakerHand::formulateTerminalHandTracker,
           (bp::arg("self"), bp::arg("phases") = Phase::NO_HAND))
      .def("getState", &ModelMakerHand::getState, bp::args("self"))
      .def("setState", &ModelMakerHand::setState, bp::args("self"))
      .def("getActuation", &ModelMakerHand::getActuation, bp::args("self"))
      .def("setActuation", &ModelMakerHand::setActuation, bp::args("self"))
      .def("formulateHorizon", &formulateHorizon,
           (bp::arg("self"), bp::arg("phases") = bp::list(),
            bp::arg("length") = 0));
}
}  // namespace python
}  // namespace sobec

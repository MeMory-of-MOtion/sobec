#include <pinocchio/fwd.hpp>  // Must be included first!
// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/walk-with-traj/model_factory_nothinking.hpp>

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

void initialize(ModelMakerNoThinkingSettings &self, const bp::dict &settings,
                const RobotDesigner &designer) {
  ModelMakerNoThinkingSettings conf;

  // timing
  conf.timeStep = bp::extract<double>(settings["timeStep"]);

  // physics
  conf.gravity = bp::extract<eVector3>(settings["gravity"]);
  conf.mu = bp::extract<double>(settings["mu"]);
  conf.coneBox = bp::extract<eVector2>(settings["coneBox"]);
  conf.minNforce = bp::extract<double>(settings["minNforce"]);
  conf.maxNforce = bp::extract<double>(settings["maxNforce"]);
  conf.comHeight = bp::extract<double>(settings["comHeight"]);
  conf.omega = bp::extract<double>(settings["omega"]);
  conf.footSize = bp::extract<double>(settings["footSize"]);
  conf.footMinimalDistance = bp::extract<double>(settings["footMinimalDistance"]);
  conf.flyHighSlope = bp::extract<double>(settings["flyHighSlope"]);

  // gains
  conf.wFootPlacement = bp::extract<double>(settings["wFootPlacement"]);
  conf.wStateReg = bp::extract<double>(settings["wStateReg"]);
  conf.wControlReg = bp::extract<double>(settings["wControlReg"]);
  conf.wLimit = bp::extract<double>(settings["wLimit"]);
  conf.wVCoM = bp::extract<double>(settings["wVCoM"]);
  conf.wCoM = bp::extract<double>(settings["wCoM"]);
  conf.wWrenchCone = bp::extract<double>(settings["wWrenchCone"]);
  conf.wFootRot = bp::extract<double>(settings["wFootRot"]);
  conf.wGroundCol = bp::extract<double>(settings["wGroundCol"]);
  conf.wCoP = bp::extract<double>(settings["wCoP"]);
  conf.wVelFoot = bp::extract<double>(settings["wVelFoot"]);
  conf.wColFeet = bp::extract<double>(settings["wColFeet"]);
  conf.wFlyHigh = bp::extract<double>(settings["wFlyHigh"]);
  conf.stateWeights = bp::extract<Eigen::VectorXd>(settings["stateWeights"]);
  conf.controlWeights =
      bp::extract<Eigen::VectorXd>(settings["controlWeights"]);
  conf.th_grad = bp::extract<double>(settings["th_grad"]);
  conf.th_stop = bp::extract<double>(settings["th_stop"]);

  self.initialize(conf, designer);
}

bp::dict get_settings(ModelMakerNoThinking &self) {
  ModelMakerNoThinkingSettings conf = self.get_settings();
  bp::dict settings;
  settings["timeStep"] = conf.timeStep;
  settings["gravity"] = conf.gravity;
  settings["mu"] = conf.mu;
  settings["coneBox"] = conf.coneBox;
  settings["minNforce"] = conf.minNforce;
  settings["maxNforce"] = conf.maxNforce;
  settings["comHeight"] = conf.comHeight;
  settings["omega"] = conf.omega;
  settings["footSize"] = conf.footSize;
  settings["footMinimalDistance"] = conf.footMinimalDistance;
  settings["flyHighSlope"] = conf.flyHighSlope;
  settings["wFootPlacement"] = conf.wFootPlacement;
  settings["wStateReg"] = conf.wStateReg;
  settings["wControlReg"] = conf.wControlReg;
  settings["wLimit"] = conf.wLimit;
  settings["wVCoM"] = conf.wVCoM;
  settings["wCoM"] = conf.wCoM;
  settings["wWrenchCone"] = conf.wWrenchCone;
  settings["wFootRot"] = conf.wFootRot;
  settings["wGroundCol"] = conf.wGroundCol;
  settings["wCoP"] = conf.wCoP;
  settings["wVelFoot"] = conf.wVelFoot;
  settings["wColFeet"] = conf.wColFeet;
  settings["wFlyHigh"] = conf.wFlyHigh;
  settings["stateWeights"] = conf.stateWeights;
  settings["controlWeights"] = conf.controlWeights;
  settings["th_grad"] = conf.th_grad;
  settings["th_stop"] = conf.th_stop;
  return settings;
}

bp::list formulateHorizon(ModelMakerNoThinking &self,
                          const bp::list &supports = bp::list(),
                          const int &length = 0) {
  if (bp::len(supports) > 0) {
    std::vector<Support> contacts;
    py_list_to_std_vector(supports, contacts);
    std::vector<AMA> models = self.formulateHorizon(contacts);

    return std_vector_to_py_list(models);

  } else if (length > 0) {
    std::vector<AMA> models = self.formulateHorizon(length);
    return std_vector_to_py_list(models);
  } else {
    throw std::runtime_error(
        "Either a list of supports or an horizon length must be provided.");
  }
}

void defineZFeetTracking(ModelMakerNoThinking &self,
                        crocoddyl::CostModelSum &costCollector,
                        const Support &supports = Support::DOUBLE) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineZFeetTracking(costs, supports);
  costCollector = *costs;
}

void defineFeetRotation(ModelMakerNoThinking &self,
                       crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineFeetRotation(costs);
  costCollector = *costs;
}

void definePostureTask(ModelMakerNoThinking &self,
                       crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.definePostureTask(costs);
  costCollector = *costs;
}

void defineCoMTask(ModelMakerNoThinking &self,
                   crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineCoMTask(costs);
  costCollector = *costs;
}

void defineCoMVelocity(ModelMakerNoThinking &self,
                       crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineCoMVelocity(costs);
  costCollector = *costs;
}

void defineVelFootTask(ModelMakerNoThinking &self,
                   crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineVelFootTask(costs);
  costCollector = *costs;
}

void defineGroundCollisionTask(ModelMakerNoThinking &self,
                   crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineGroundCollisionTask(costs);
  costCollector = *costs;
}

void defineFootCollisionTask(ModelMakerNoThinking &self,
                   crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineFootCollisionTask(costs);
  costCollector = *costs;
}

void defineFlyHighTask(ModelMakerNoThinking &self,
                   crocoddyl::CostModelSum &costCollector,
                   const Support &supports = Support::DOUBLE) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineFlyHighTask(costs, supports);
  costCollector = *costs;
}

void exposeModelFactory() {
  bp::enum_<Support>("Support")
      .value("LEFT", Support::LEFT)
      .value("RIGHT", Support::RIGHT)
      .value("DOUBLE", Support::DOUBLE);

  bp::class_<ModelMakerNoThinking>("ModelMakerNoThinking", bp::init<>())
      .def("initialize", &initialize, bp::args("self", "settings", "design"))
      // .def("formulateHorizon", &formulateHorizon, bp::args("self",
      // "supports"))
      .def("get_settings", &get_settings, bp::args("self"))
      .def("defineZFeetTracking", &defineZFeetTracking,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("supports") = Support::DOUBLE))
      .def("defineFeetRotation", &defineFeetRotation,
           bp::args("self", "costCollector"))
      .def("defineCoMTask", &defineCoMTask,
           bp::args("self", "costCollector"))
      .def("defineCoMVelocity", &defineCoMVelocity,
           bp::args("self", "costCollector"))
      .def("defineCoPTask", &defineCoPTask,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("supports") = Support::DOUBLE))
      .def("defineVelFootTask", &defineVelFootTask,
           bp::args("self", "costCollector"))
      .def("defineGroundCollisionTask", &defineGroundCollisionTask,
           bp::args("self", "costCollector"))
      .def("defineFlyHighTask", &defineFlyHighTask,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("supports") = Support::DOUBLE))
      .def("defineFootCollisionTask", &defineFootCollisionTask,
           bp::args("self", "costCollector"))
      .def("formulateNoThinking", &ModelMakerNoThinking::formulateNoThinking,
           (bp::arg("self"), bp::arg("supports") = Support::DOUBLE))
      .def("formulateNoThinkingTerminal", &ModelMakerNoThinking::formulateNoThinkingTerminal,
           (bp::arg("self"), bp::arg("supports") = Support::DOUBLE))
      .def("formulateHorizon", &formulateHorizon,
           (bp::arg("self"), bp::arg("supports") = bp::list(),
            bp::arg("length") = 0);
}
}  // namespace python
}  // namespace sobec
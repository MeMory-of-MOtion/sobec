///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"
// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/walk-with-traj/horizon_manager.hpp>

namespace sobec {
namespace python {
namespace bp = boost::python;

template <typename T>
inline void py_list_to_std_vector(const bp::object &iterable,
                                  std::vector<T> &out) {
  out = std::vector<T>(boost::python::stl_input_iterator<T>(iterable),
                       boost::python::stl_input_iterator<T>());
}

void initialize(HorizonManager &self, const bp::dict &settings,
                const Eigen::VectorXd &x0, const bp::list runningModels,
                const AMA &terminalModel) {
  HorizonManagerSettings conf;
  conf.leftFootName = bp::extract<std::string>(settings["leftFootName"]);
  conf.rightFootName = bp::extract<std::string>(settings["rightFootName"]);

  std::vector<AMA> horizonModels;
  py_list_to_std_vector(runningModels, horizonModels);
  self.initialize(conf, x0, horizonModels, terminalModel);
}

bp::dict get_contacts(HorizonManager &self, const unsigned long time) {
  bp::dict contacts;
  for (std::string frame : self.contacts(time)->get_active_set())
    contacts[frame] = self.contacts(time)->get_active_set().find(frame) !=
                      self.contacts(time)->get_active_set().end();

  for (std::string frame : self.contacts(time)->get_inactive_set())
    contacts[frame] = self.contacts(time)->get_active_set().find(frame) !=
                      self.contacts(time)->get_active_set().end();
  return contacts;
}

void exposeHorizonManager() {
  bp::class_<HorizonManager>("HorizonManager", bp::init<>())
      .def("initialize", &initialize,
           bp::args("self", "settings", "x0", "runningModels", "terminalModel"))
      .def("ama", &HorizonManager::ama, bp::args("self", "time"))
      .def("iam", &HorizonManager::iam, bp::args("self", "time"))
      .def("terminaliam", &HorizonManager::terminaliam, bp::args("self"))
      .def("dam", &HorizonManager::dam, bp::args("self", "time"))
      .def("terminaldam", &HorizonManager::terminaldam, bp::args("self"))
      .def("dad", &HorizonManager::dad, bp::args("self", "time"))
      .def("pinData", &HorizonManager::pinData, bp::args("self", "time"))
      .def("costs", &HorizonManager::costs, bp::args("self", "time"))
      .def("terminalCosts", &HorizonManager::terminalCosts, bp::args("self"))
      .def("contacts", &HorizonManager::contacts, bp::args("self", "time"))
      .def("state", &HorizonManager::state, bp::args("self", "time"))
      .def("actuation", &HorizonManager::actuation, bp::args("self", "time"))
      .def("ada", &HorizonManager::ada, bp::args("self", "time"))
      .def("iad", &HorizonManager::iad, bp::args("self", "time"))
      .def("setPoseReference", &HorizonManager::setPoseReference,
           bp::args("self", "time", "costName", "pose"))
      .def("setRotationReference", &HorizonManager::setRotationReference,
           bp::args("self", "time", "costName", "rotation"))
      .def("setTerminalRotationReference",
           &HorizonManager::setTerminalRotationReference,
           bp::args("self", "costName", "rotation"))
      .def("setTranslationReference", &HorizonManager::setTranslationReference,
           bp::args("self", "time", "costName", "translation"))
      .def("setTerminalPoseReference",
           &HorizonManager::setTerminalPoseReference,
           bp::args("self", "costName", "pose"))
      .def("setTerminalTranslationReference",
           &HorizonManager::setTerminalTranslationReference,
           bp::args("self"
                    "costName",
                    "translation"))
      .def("activateContactLF", &HorizonManager::activateContactLF,
           bp::args("self", "time", "contactName"))
      .def("activateContactRF", &HorizonManager::activateContactRF,
           bp::args("self", "time", "contactName"))
      .def("removeContactLF", &HorizonManager::removeContactLF,
           bp::args("self", "time", "contactName"))
      .def("removeContactRF", &HorizonManager::removeContactRF,
           bp::args("self", "time", "contactName"))
      .def("setForceReference", &HorizonManager::setForceReference,
           bp::args("self", "time", "costName", "ref_wrench"))
      .def("setWrenchReference", &HorizonManager::setWrenchReference,
           bp::args("self", "time", "costName", "ref_wrench"))
      .def("setTerminalPoseCoM", &HorizonManager::setTerminalPoseCoM,
           bp::args("self", "costName", "ref_placement"))
      .def("setTerminalDCMReference", &HorizonManager::setTerminalDCMReference,
           bp::args("self", "costName", "ref_translation"))
      .def("setVelocityRefCOM", &HorizonManager::setVelocityRefCOM,
           bp::args("self", "time", "costName", "ref_velocity"))
      .def("setSwingingLF", &HorizonManager::setSwingingLF,
           bp::args("self", "time", "contactNameLF", "contactNameRF",
                    "forceCostName"))
      .def("setSwingingRF", &HorizonManager::setSwingingRF,
           bp::args("self", "time", "contactNameLF", "contactNameRF",
                    "forceCostName"))
      .def("setDoubleSupport", &HorizonManager::setDoubleSupport,
           bp::args("self", "time", "contactNameLF", "contactNameRF"))
      .def("getFootPoseReference",
           bp::make_function(
               &HorizonManager::getFootPoseReference,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("getTerminalFootPoseReference",
           bp::make_function(
               &HorizonManager::getTerminalFootPoseReference,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def<void (HorizonManager::*)(const AMA &, const ADA &)>(
          "recede", &HorizonManager::recede, bp::args("self", "IAM", "IAD"))
      .def<void (HorizonManager::*)(const AMA &)>(
          "recede", &HorizonManager::recede, bp::args("self", "IAM"))
      .def<void (HorizonManager::*)()>("recede", &HorizonManager::recede,
                                       bp::args("self"))
      .add_property("ddp", &HorizonManager::get_ddp, &HorizonManager::set_ddp)
      .def("currentTorques",
           bp::make_function(
               &HorizonManager::currentTorques,
               bp::return_value_policy<bp::reference_existing_object>()))
      .def("solve", &HorizonManager::solve,
           (bp::arg("self"), bp::arg("x_measured"), bp::arg("ddpIteration"),
            bp::arg("is_feasible") = false))
      .def<void (HorizonManager::*)(const unsigned long, const std::string &,
                                    const std::string &)>(
          "setBalancingTorque", &HorizonManager::setBalancingTorque,
          bp::args("self", "time"))
      .def<void (HorizonManager::*)(const unsigned long, const std::string &,
                                    const Eigen::VectorXd &)>(
          "setBalancingTorque", &HorizonManager::setBalancingTorque,
          bp::args("self", "time", "x"))
      .def("size", &HorizonManager::size, (bp::arg("self")))
      .def("supportSize", &HorizonManager::supportSize, (bp::arg("self")))
      .def("setActuationReference", &HorizonManager::setActuationReference,
           bp::args("self", "time", "actuationCostName"))
      .def("get_contacts", &get_contacts, bp::args("self", "time"));
  return;
}
}  // namespace python
}  // namespace sobec

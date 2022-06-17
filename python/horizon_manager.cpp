///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////
#include "sobec/fwd.hpp"
// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/horizon_manager.hpp>

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
      .def("dam", &HorizonManager::dam, bp::args("self", "time"))
      .def("costs", &HorizonManager::costs, bp::args("self", "time"))
      .def("contacts", &HorizonManager::contacts, bp::args("self", "time"))
      .def("state", &HorizonManager::state, bp::args("self", "time"))
      .def("actuation", &HorizonManager::actuation, bp::args("self", "time"))
      .def("ada", &HorizonManager::ada, bp::args("self", "time"))
      .def("iad", &HorizonManager::iad, bp::args("self", "time"))
      .def("setPoseReferenceLF", &HorizonManager::setPoseReferenceLF,
           bp::args("self", "time", "pose", "costName"))
      .def("setPoseReferenceRF", &HorizonManager::setPoseReferenceRF,
           bp::args("self", "time", "pose", "costName"))
      .def("activateContactLF", &HorizonManager::activateContactLF,
           bp::args("self", "time", "contactName"))
      .def("activateContactRF", &HorizonManager::activateContactRF,
           bp::args("self", "time", "contactName"))
      .def("removeContactLF", &HorizonManager::removeContactLF,
           bp::args("self", "time", "contactName"))
      .def("removeContactRF", &HorizonManager::removeContactRF,
           bp::args("self", "time", "contactName"))
      .def("setForceReferenceLF", &HorizonManager::setForceReferenceLF,
           bp::args("self", "time", "costName", "ref_wrench"))
      .def("setForceReferenceRF", &HorizonManager::setForceReferenceRF,
           bp::args("self", "time", "costName", "ref_wrench"))
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
      .def<void (HorizonManager::*)(const unsigned long &, const std::string &,
                                    const std::string &)>(
          "setBalancingTorque", &HorizonManager::setBalancingTorque,
          bp::args("self", "time"))
      .def<void (HorizonManager::*)(const unsigned long &, const std::string &,
                                    const Eigen::VectorXd &)>(
          "setBalancingTorque", &HorizonManager::setBalancingTorque,
          bp::args("self", "time", "x"))
      //  .def("size", &size)
      .def("size", bp::make_function(
                       &HorizonManager::size,
                       bp::return_value_policy<bp::copy_const_reference>()))
      .def("setActuationReference", &HorizonManager::setActuationReference,
           bp::args("self", "time", "actuationCostName"))
      .def("get_contacts", &get_contacts, bp::args("self", "time"));
  return;
}
}  // namespace python
}  // namespace sobec

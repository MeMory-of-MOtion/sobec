///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <pinocchio/fwd.hpp>
// This line must be the first include
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/algorithm/model.hpp>

#include "pinocchio/multibody/model.hpp"
#include "sobec/designer.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;

template <typename T>
inline void py_list_to_std_vector(const bp::object &iterable,
                                  std::vector<T> &out) {
  out = std::vector<T>(boost::python::stl_input_iterator<T>(iterable),
                       boost::python::stl_input_iterator<T>());
}

void initialize(sobec::RobotDesigner &self, bp::dict settings) {
  RobotDesignerSettings conf;
  conf.urdfPath = bp::extract<std::string>(settings["urdfPath"]);
  conf.srdfPath = bp::extract<std::string>(settings["srdfPath"]);
  conf.leftFootName = bp::extract<std::string>(settings["leftFootName"]);
  conf.rightFootName = bp::extract<std::string>(settings["rightFootName"]);
  conf.robotDescription =
      bp::extract<std::string>(settings["robotDescription"]);
  py_list_to_std_vector(settings["controlledJointsNames"],
                        conf.controlledJointsNames);

  self.initialize(conf);
}

bp::dict get_settings(RobotDesigner &self) {
  RobotDesignerSettings conf = self.get_settings();
  bp::dict settings;
  settings["urdfPath"] = conf.urdfPath;
  settings["srdfPath"] = conf.srdfPath;
  settings["leftFootName"] = conf.leftFootName;
  settings["rightFootName"] = conf.rightFootName;
  settings["robotDescription"] = conf.robotDescription;

  return settings;
}

pinocchio::Model get_rModelComplete(RobotDesigner &self) {
  return self.get_rModelComplete();
}

// pinocchio::Model get_rModel(RobotDesigner &self) { return self.get_rModel(); }

pinocchio::Data get_rData(RobotDesigner &self) { return self.get_rData(); }

pinocchio::Data get_rDataComplete(RobotDesigner &self) {
  return self.get_rDataComplete();
}

void exposeDesigner() {
  bp::class_<RobotDesigner>("RobotDesigner", bp::init<>())
      .def("initialize", &initialize)
      .def("updateReducedModel", &RobotDesigner::updateReducedModel)
      .def("updateCompleteModel", &RobotDesigner::updateCompleteModel)
      .def("get_LF_frame", bp::make_function(&RobotDesigner::get_LF_frame, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_RF_frame", bp::make_function(&RobotDesigner::get_RF_frame, bp::return_value_policy<bp::reference_existing_object>()))
      .def("getRobotMass", bp::make_function(&RobotDesigner::getRobotMass, bp::return_value_policy<bp::copy_const_reference>()))
      .def("get_rModel", bp::make_function(&RobotDesigner::get_rModel, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_rModelComplete", bp::make_function(&RobotDesigner::get_rModelComplete, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_rData", bp::make_function(&RobotDesigner::get_rData, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_rDataComplete", bp::make_function(&RobotDesigner::get_rDataComplete, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_q0", bp::make_function(&RobotDesigner::get_q0, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_q0Complete", bp::make_function(&RobotDesigner::get_q0Complete, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_v0", bp::make_function(&RobotDesigner::get_v0, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_v0Complete", bp::make_function(&RobotDesigner::get_v0Complete, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_x0", bp::make_function(&RobotDesigner::get_x0, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_LF_name", bp::make_function(&RobotDesigner::get_LF_name, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_RF_name", bp::make_function(&RobotDesigner::get_RF_name, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_LF_id", bp::make_function(&RobotDesigner::get_LF_id, bp::return_value_policy<bp::copy_const_reference>()))
      .def("get_RF_id", bp::make_function(&RobotDesigner::get_RF_id, bp::return_value_policy<bp::copy_const_reference>()))
      .def("get_settings", bp::make_function(&RobotDesigner::get_settings, bp::return_value_policy<bp::reference_existing_object>()))
      .def("get_controlledJointsIDs", bp::make_function(&RobotDesigner::get_controlledJointsIDs, bp::return_value_policy<bp::copy_const_reference>()))
      ;

  return;
}

}  // namespace python
}  // namespace sobec

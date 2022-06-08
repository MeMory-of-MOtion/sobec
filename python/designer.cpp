///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <pinocchio/fwd.hpp>  // This line must be the first include
#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/activation-base.hpp>

#include "pinocchio/multibody/model.hpp"
#include <pinocchio/algorithm/model.hpp>
#include <eigenpy/eigenpy.hpp>

#include "sobec/designer.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;

template<typename T>
inline
void py_list_to_std_vector( const bp::object& iterable, std::vector< T > &out)
{
    out = std::vector< T >( boost::python::stl_input_iterator< T >( iterable ),
                             boost::python::stl_input_iterator< T >( ) );
}

void initialize(RobotDesigner& self, bp::dict settings){
    RobotDesignerSettings conf;
    conf.urdfPath = bp::extract<std::string>(settings["urdfPath"]);
    conf.srdfPath = bp::extract<std::string>(settings["srdfPath"]);
    conf.leftFootName = bp::extract<std::string>(settings["leftFootName"]);
    conf.rightFootName = bp::extract<std::string>(settings["rightFootName"]);
    conf.robotDescription = bp::extract<std::string>(settings["robotDescription"]);
    py_list_to_std_vector(settings["controlledJointsNames"], conf.controlledJointsNames);

    self.initialize(conf);
}

bp::dict get_settings(RobotDesigner &self){
    RobotDesignerSettings conf = self.get_settings();
    bp::dict settings;
    settings["urdfPath"] = conf.urdfPath;
    settings["srdfPath"] = conf.srdfPath;
    settings["leftFootName"] = conf.leftFootName;
    settings["rightFootName"] = conf.rightFootName; 
    settings["robotDescription"] = conf.robotDescription; 

    return settings;
}

pinocchio::Model get_rModelComplete(RobotDesigner &self){
    return self.get_rModelComplete();
}

pinocchio::Model get_rModel(RobotDesigner &self){
    return self.get_rModel();
}

pinocchio::Data get_rData(RobotDesigner &self){
  return self.get_rData();
}

pinocchio::Data get_rDataComplete(RobotDesigner &self){
  return self.get_rDataComplete();
}


void exposeDesigner() {

    bp::class_<RobotDesigner>("RobotDesigner", bp::init<>())
      .def("initialize", &initialize)
      .def("updateReducedModel", &RobotDesigner::updateReducedModel)
      .def("updateCompleteModel", &RobotDesigner::updateCompleteModel)
      .def("get_LF_frame", &RobotDesigner::get_LF_frame)
      .def("get_RF_frame", &RobotDesigner::get_RF_frame)
      .def("getRobotMass", &RobotDesigner::getRobotMass)
      .def("get_rModel", &get_rModel)
      .def("get_rModelComplete", &get_rModelComplete)
      .def("get_rData", &get_rData)
      .def("get_rDataComplete", &get_rDataComplete)
      .def("get_q0", &RobotDesigner::get_q0)
      .def("get_q0Complete", &RobotDesigner::get_q0Complete)
      .def("get_x0", &RobotDesigner::get_x0)
      .def("get_LF_name", &RobotDesigner::get_LF_name)
      .def("get_RF_name", &RobotDesigner::get_RF_name)
      .def("get_LF_id", &RobotDesigner::get_LF_id)
      .def("get_RF_id", &RobotDesigner::get_RF_id)
      .def("get_settings", &get_settings)
      ;

    return;
}

}  // namespace python
}  // namespace sobec

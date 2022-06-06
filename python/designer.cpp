///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <pinocchio/fwd.hpp> 
#include "pinocchio/multibody/model.hpp"
#include <pinocchio/algorithm/model.hpp>
#include <eigenpy/eigenpy.hpp>

#include "sobec/designer.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;

// template<typename T>
// inline
// void py_list_to_std_vector( const bp::object& iterable, std::vector< T > &out)
// {
//     out = std::vector< T >( boost::python::stl_input_iterator< T >( iterable ),
//                              boost::python::stl_input_iterator< T >( ) );
// }

// boost::shared_ptr<RobotDesigner> build_RobotDesigner(){//const bp::dict &settings){ 
//     RobotDesignerSettings conf;
//     // conf.urdfPath = bp::extract<std::string>(settings["urdfPath"]);
//     // conf.srdfPath = bp::extract<std::string>(settings["srdfPath"]);
//     // conf.leftFootName = bp::extract<std::string>(settings["leftFootName"]);
//     // conf.rightFootName = bp::extract<std::string>(settings["rightFootName"]);
//     // conf.robotDescription = bp::extract<std::string>(settings["robotDescription"]);
//     // py_list_to_std_vector(settings["controlledJointsNames"], conf.controlledJointsNames);
//     return boost::make_shared<RobotDesigner>(RobotDesigner(conf));
// }

void exposeDesigner() {

    bp::class_<RobotDesignerSettings>("RobotDesignerSettings")
      .def_readwrite("urdfPath", &RobotDesignerSettings::urdfPath)
      .def_readwrite("srdfPath", &RobotDesignerSettings::srdfPath)
      .def_readwrite("leftFootName", &RobotDesignerSettings::leftFootName)
      .def_readwrite("leftFootName", &RobotDesignerSettings::leftFootName)
      .def_readwrite("robotDescription", &RobotDesignerSettings::robotDescription)
      .def_readwrite("controlledJointsNames", &RobotDesignerSettings::controlledJointsNames)
      ;

    // bp::class_<RobotDesigner, boost::noncopyable>("RobotDesigner", bp::init<>())
    //   .def(bp::init<RobotDesignerSettings>())
    //   .def("initialize", &RobotDesigner::initialize)
    //   ;

    bp::class_<boost::shared_ptr<DesignerTest>>("DesignerTest", bp::init<>())
        .def("initialize", &DesignerTest::initialize)
        // .def("get_rmodel", &DesignerTest::get_rModel)
        ;

    return;
}

}  // namespace python
}  // namespace sobec

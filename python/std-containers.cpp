///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/bindings/python/utils/std-vector.hpp>
#include <pinocchio/bindings/python/utils/std-map.hpp>

namespace sobec {
namespace python {

namespace bp = boost::python;
namespace pp = pinocchio::python;

void exposeStdContainers() {
  pp::StdVectorPythonVisitor<pinocchio::FrameIndex>::expose("StdVectorPinocchioFrameIndex_");

  pp::StdVectorPythonVisitor<std::string>::expose("StdVectorStdStringIndex_");

  pp::StdVectorPythonVisitor<pinocchio::Force>::expose("StdVectorPinocchioForce_");
  pp::StdVectorPythonVisitor<std::vector<pinocchio::Force>>::expose("StdVectorStdVectorPinocchioForce_");

  bp::class_<std::map<pinocchio::FrameIndex, pinocchio::FrameIndex> >("StdMapPinocchioFrameIndexToPinocchioFrameIndex_")
    .def(bp::map_indexing_suite<std::map<pinocchio::FrameIndex, pinocchio::FrameIndex>, true>())
    // .def(pp::details::overload_base_get_item_for_std_map<std::map<pinocchio::FrameIndex, pinocchio::FrameIndex>>())
    ;

  bp::class_<std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> >("EigenVectorXd")
    .def_readwrite("first", &std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>::first)
    .def_readwrite("second", &std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>::second);
}

}  // namespace python
}  // namespace sobec
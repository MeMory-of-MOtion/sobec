///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/force.hpp>
#include <eigenpy/fwd.hpp>

#if EIGENPY_VERSION_AT_LEAST(2,8,0)
  #include <eigenpy/std-map.hpp>
  #include <eigenpy/std-vector.hpp>
#else
  #include <pinocchio/bindings/python/utils/std-map.hpp>
  #include <pinocchio/bindings/python/utils/std-vector.hpp>
#endif

namespace sobec {
namespace python {

namespace bp = boost::python;

#if EIGENPY_VERSION_AT_LEAST(2,8,0)
  using eigenpy::StdVectorPythonVisitor;
#else
  using pinocchio::python::StdVectorPythonVisitor;
#endif

void exposeStdContainers() {
  StdVectorPythonVisitor<std::vector<std::string>>::expose(
      "StdVectorStdStringIndex_");

  StdVectorPythonVisitor<std::vector<pinocchio::Force>>::expose(
      "StdVectorPinocchioForce_");
  StdVectorPythonVisitor<std::vector<std::vector<pinocchio::Force>>>::
      expose("StdVectorStdVectorPinocchioForce_");

  using frame_frame_map_t = std::map<pinocchio::FrameIndex, pinocchio::FrameIndex>;
  bp::class_<frame_frame_map_t>(
      "StdMapPinocchioFrameIndexToPinocchioFrameIndex_")
      .def(bp::map_indexing_suite<frame_frame_map_t, true>())
      // .def(ep::details::overload_base_get_item_for_std_map<std::map<pinocchio::FrameIndex,
      // pinocchio::FrameIndex>>())
      ;

  using StdVecVectorXd = std::vector<Eigen::VectorXd>;
  using pair_vec_vec_t = std::pair<StdVecVectorXd, StdVecVectorXd>;
  bp::class_<pair_vec_vec_t>("StdPair_StdVector_EigenVectorXd")
      .def_readwrite("first", &pair_vec_vec_t::first)
      .def_readwrite("second", &pair_vec_vec_t::second);
}

}  // namespace python
}  // namespace sobec

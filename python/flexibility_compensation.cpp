#include <sstream>

#include "sobec/fwd.hpp"
// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/flexibility_compensation.hpp>

namespace sobec {
namespace python {
namespace bp = boost::python;

void initialize(Flex &self, const bp::dict &settings) {
  FlexSettings conf;

  conf.left_stiffness = bp::extract<eVector2>(settings["left_stiffness"]);
  conf.left_damping = bp::extract<eVector2>(settings["left_damping"]);
  conf.right_stiffness = bp::extract<eVector2>(settings["right_stiffness"]);
  conf.right_damping = bp::extract<eVector2>(settings["right_damping"]);
  conf.dt = bp::extract<double>(settings["dt"]);
  conf.MA_duration = bp::extract<double>(settings["MA_duration"]);

  self.initialize(conf);
}

bp::tuple correctEstimatedDeflections(const eVectorX &desiredTorque,
                                      const eVectorX &q, const eVectorX &dq,
                                      const Eigen::Array3i &leftHipIndices,
                                      const Eigen::Array3i &rightHipIndices) {
  eVectorX correct_q, correct_dq;
  correct_q << q;
  correct_dq << dq;

  correctEstimatedDeflections(desiredTorque, correct_q, correct_dq,
                              leftHipIndices, rightHipIndices);

  return bp::make_tuple(correct_q, correct_dq);
}

void exposeFlex() {
  bp::class_<Flex>("Flex", bp::init<>())
      .def("initialize", &initialize, bp::args("self", "settings"))
      .def("Settings", &Flex::getSettings, bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
      .def("computeDeflection", bp::make_function(
            static_cast<const eVector2 &(Flex::*)(const eArray2 &, const side)>(
              &Flex::computeDeflection), bp::return_value_policy
              <bp::reference_existing_object>(), bp::args("self", "torques", "side")))
      .def("computeDeflection", bp::make_function(static_cast<const eVector2 &(Flex::*)
                                                 (const eArray2 &, const eArray2 &,
                                                  const eArray2 &, const eArray2 &,
                                                  const double)>(&Flex::computeDeflection), 
                                                  bp::return_value_policy<bp::reference_existing_object>(), 
                                                  bp::args("self", "torques", "delta0", "stiffnes", "damping")))
      .def("correctHip", &Flex::correctHip, bp::args("self", "delta", "delta_dot", "q", "dq", "hipIndices"))
      .def("correctDeflections", &Flex::correctDeflections, bp::args("self", "leftFlexingTorque",
                                                                     "rightFlexingTorque","q",
                                                                     "dq","leftHipIndices",
                                                                     "rightHipIndices"))
      .def("correctEstimatedDeflections", &Flex::correctEstimatedDeflections, bp::args("self", "esiredTorque", "q",
                                                                                       "dq", "leftHipIndices", "rightHipIndices"))
      .def("movingAverage", &Flex::movingAverage, bp::return_value_policy<bp::reference_existing_object>(), 
                              bp::args("self", "value", "queue"))
      ;
}
}  // namespace python
}  // namespace sobec

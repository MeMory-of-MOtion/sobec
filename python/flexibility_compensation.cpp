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

  eVector2 left_stiffness = eVector2(15000, 15000);          // (y, x) [Nm/rad]
  eVector2 left_damping = 2 * left_stiffness.cwiseSqrt();    // (y, x) [Nm/rad]
  eVector2 right_stiffness = eVector2(15000, 15000);         // (y, x) [Nm/rad]
  eVector2 right_damping = 2 * right_stiffness.cwiseSqrt();  // (y, x) [Nm/rad]
  double dt = 0.002;

  conf.left_stiffness = bp::extract<eVector2>(settings["left_stiffness"]);
  conf.left_damping = bp::extract<eVector2>(settings["left_damping"]);
  conf.right_stiffness = bp::extract<eVector2>(settings["right_stiffness"]);
  conf.right_damping = bp::extract<eVector2>(settings["right_damping"]);
  conf.dt = bp::extract<double>(settings["dt"]);

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

void exposeWBC() {
  bp::class_<Flex>("Flex", bp::init<>())
      .def("initialize", &initialize, bp::args("self", "settings"));
}
}  // namespace python
}  // namespace sobec

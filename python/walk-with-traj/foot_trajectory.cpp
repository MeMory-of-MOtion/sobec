#include <pinocchio/multibody/fwd.hpp>  // Must be included first!
#include <sstream>

// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/walk-with-traj/foot_trajectory.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;

void exposeFootTrajectory() {
  bp::class_<FootTrajectory>("FootTrajectory",
                             bp::init<double, double, double>(
                                 bp::args("self", "swing_leg_height", "swing_pose_penetration", "landing_advance")))
      .def("generate", &FootTrajectory::generate,
           bp::args("self", "t_init", "t_end", "pose_init", "pose_end", "constant"))
      .def("compute", &FootTrajectory::compute, bp::args("self", "time"));
}
}  // namespace python
}  // namespace sobec

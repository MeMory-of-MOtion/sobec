#ifndef __sobec_pin__
#define __sobec_pin__

#include "pinocchio/spatial/se3.hpp"

namespace sobec {
typedef pinocchio::SE3 SE3;
SE3 adds(const SE3 &a, const SE3 &b);
SE3 subs(const SE3 &a, const SE3 &b);
}  // namespace sobec

#endif

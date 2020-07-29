#ifndef __example_adder_pin__
#define __example_adder_pin__

#include "pinocchio/spatial/se3.hpp"

namespace gepetto {
namespace example {
typedef pinocchio::SE3 SE3;
SE3 adds(const SE3 &a, const SE3 &b);
SE3 subs(const SE3 &a, const SE3 &b);
}  // namespace example
}  // namespace gepetto

#endif

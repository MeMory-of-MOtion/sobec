#include "example-adder/pin.hpp"

namespace gepetto {
namespace example {
SE3 adds(const SE3 &a, const SE3 &b) {
  SE3 ret;
  ret = a * b;
  return ret;
}
SE3 subs(const SE3 &a, const SE3 &b) {
  SE3 ret;
  ret = a.inverse() * b;
  return ret;
}
}  // namespace example
}  // namespace gepetto

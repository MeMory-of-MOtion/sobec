#include "example-adder/python.hpp"

namespace gepetto {
namespace example {
void exposeExampleAdder() {
  boost::python::def("add", add);
  boost::python::def("sub", sub);
}
}  // namespace example
}  // namespace gepetto

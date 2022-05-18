#include "sobec/python.hpp"

namespace sobec {
void exposeExampleAdder() {
  boost::python::def("add", add);
  boost::python::def("sub", sub);
}
}  // namespace sobec

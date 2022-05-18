#include "sobec/python.hpp"

namespace sobec {
void exposeExamplePin() {
  boost::python::def("adds", adds);
  boost::python::def("subs", subs);
}
}  // namespace sobec

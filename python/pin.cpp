#include "example-adder/python.hpp"

namespace gepetto {
namespace example {
void exposeExamplePin() {
  boost::python::def("adds", adds);
  boost::python::def("subs", subs);
}
}  // namespace example
}  // namespace gepetto

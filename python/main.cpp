#include "example-adder/python.hpp"

BOOST_PYTHON_MODULE(example_adder_pywrap) {
  gepetto::example::exposeExampleAdder();
  gepetto::example::exposeExamplePin();
  gepetto::example::exposeActionUniEx();
}

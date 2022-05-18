#include "sobec/python.hpp"

BOOST_PYTHON_MODULE(sobec_pywrap) {
  sobec::exposeExampleAdder();
  sobec::exposeExamplePin();
  sobec::exposeActionUniEx();
}

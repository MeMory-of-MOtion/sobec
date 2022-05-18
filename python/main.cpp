#include "sobec/python.hpp"
//#include "python.hpp"
#include <eigenpy/eigenpy.hpp>
//#include "com-velocity.

BOOST_PYTHON_MODULE(sobec_pywrap) {
  sobec::exposeExampleAdder();
  sobec::exposeExamplePin();
  sobec::exposeActionUniEx();
  //gepetto::sobec::exposeSobec();
  sobec::python::exposeResidualCoMVelocity();
}


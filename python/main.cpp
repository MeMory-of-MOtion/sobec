#include "sobec/python.hpp"
#include <eigenpy/eigenpy.hpp>

BOOST_PYTHON_MODULE(sobec_pywrap) {
  sobec::python::exposeResidualCoMVelocity();
  sobec::python::exposeActivationQuadRef();
}

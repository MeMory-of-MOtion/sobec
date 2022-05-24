#include <eigenpy/eigenpy.hpp>

#include "sobec/python.hpp"

BOOST_PYTHON_MODULE(sobec_pywrap) {
  sobec::python::exposeResidualCoMVelocity();
  sobec::python::exposeActivationQuadRef();
  sobec::python::exposeIntegratedActionLPF();
  sobec::python::exposeContact3D();
  sobec::python::exposeContact1D();
  sobec::python::exposeMultipleContacts();
  sobec::python::exposeDAMContactFwdDyn();
}

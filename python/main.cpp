#include <eigenpy/eigenpy.hpp>

#include "sobec/python.hpp"

BOOST_PYTHON_MODULE(sobec_pywrap) {
  sobec::python::exposeResidualVelCollision();
  sobec::python::exposeResidualCoMVelocity();
  sobec::python::exposeResidualCenterOfPressure();
  sobec::python::exposeResidualFlyHigh();
  sobec::python::exposeActivationQuadRef();
  sobec::python::exposeDesigner();
  sobec::python::exposeHorizonManager();
  sobec::python::exposeModelFactory();
  sobec::python::exposeIntegratedActionLPF();
  sobec::python::exposeContact3D();
  sobec::python::exposeContact1D();
  sobec::python::exposeMultipleContacts();
  sobec::python::exposeDAMContactFwdDyn();
  sobec::python::exposeResidualContactForce();
  sobec::python::exposeOCP();
  sobec::python::exposeWBC();
}

#include <eigenpy/eigenpy.hpp>

#include "sobec/python.hpp"

BOOST_PYTHON_MODULE(sobec_pywrap) {
  boost::python::import("crocoddyl");
  sobec::python::exposeResidualVelCollision();
  sobec::python::exposeResidualCoMVelocity();
  sobec::python::exposeResidualCenterOfPressure();
  sobec::python::exposeResidualFeetCollision();
  sobec::python::exposeResidualFlyHigh();
  sobec::python::exposeActivationQuadRef();
  sobec::python::exposeDesigner();
  sobec::python::exposeHorizonManager();
  sobec::python::exposeModelFactory();
  sobec::python::exposeIntegratedActionLPF();
  sobec::python::exposeWBC();
  sobec::python::exposeMPCWalk();

  sobec::newcontacts::python::exposeContact3D();
  sobec::newcontacts::python::exposeContact1D();
  sobec::newcontacts::python::exposeMultipleContacts();
  sobec::newcontacts::python::exposeDAMContactFwdDyn();
  sobec::newcontacts::python::exposeResidualContactForce();
}

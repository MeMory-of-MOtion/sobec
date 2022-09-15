#include <eigenpy/eigenpy.hpp>

#include "sobec/python.hpp"

BOOST_PYTHON_MODULE(sobec_pywrap) {
  boost::python::import("pinocchio");
  boost::python::import("crocoddyl");
  boost::python::import("pinocchio");
  // Enabling eigenpy support, i.e. numpy/eigen compatibility.
  eigenpy::enableEigenPy();
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::VectorXi);
  sobec::python::exposeStdContainers();
  sobec::python::exposeResidualVelCollision();
  sobec::python::exposeResidualCoMVelocity();
  sobec::python::exposeResidualCenterOfPressure();
  sobec::python::exposeResidualFeetCollision();
  sobec::python::exposeResidualFlyHigh();
  sobec::python::exposeResidual2DSurface();
  sobec::python::exposeActivationQuadRef();
  sobec::python::exposeDesigner();
  sobec::python::exposeHorizonManager();
  sobec::python::exposeModelFactory();
  sobec::python::exposeModelFactoryNoThinking();
  sobec::python::exposeFlex();
  sobec::python::exposeIntegratedActionLPF();
  sobec::python::exposeStateLPF();
  sobec::python::exposeWBC();
  sobec::python::exposeOCPWalk();
  sobec::python::exposeMPCWalk();
  sobec::python::exposeFootTrajectory();

  sobec::newcontacts::python::exposeContact3D();
  sobec::newcontacts::python::exposeContact1D();
  sobec::newcontacts::python::exposeMultipleContacts();
  sobec::newcontacts::python::exposeDAMContactFwdDyn();
  sobec::newcontacts::python::exposeResidualContactForce();
}

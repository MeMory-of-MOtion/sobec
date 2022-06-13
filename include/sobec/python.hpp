#ifndef __sobec_python__
#define __sobec_python__

#include <boost/python.hpp>

namespace sobec {
namespace python {

void exposeResidualCoMVelocity();
void exposeResidualVelCollision();
void exposeResidualCenterOfPressure();
void exposeResidualFeetCollision();
void exposeResidualFlyHigh();
void exposeActivationQuadRef();
void exposeIntegratedActionLPF();
void exposeContact3D();
void exposeContact1D();
void exposeMultipleContacts();
void exposeDAMContactFwdDyn();
void exposeResidualContactForce();
void exposeMPCWalk();
  
}  // namespace python
}  // namespace sobec

#endif  // #ifndef __sobec_python__

#ifndef __sobec_python__
#define __sobec_python__

#include <boost/python.hpp>
#include <pinocchio/fwd.hpp>

namespace sobec {
namespace python {

void exposeStdContainers();
void exposeResidualCoMVelocity();
void exposeResidualVelCollision();
void exposeResidualCenterOfPressure();
void exposeResidualFeetCollision();
void exposeResidualFlyHigh();
void exposeActivationQuadRef();
void exposeDesigner();
void exposeHorizonManager();
void exposeModelFactory();
void exposeIntegratedActionLPF();
void exposeContact3D();
void exposeContact1D();
void exposeMultipleContacts();
void exposeDAMContactFwdDyn();
void exposeResidualContactForce();
void exposeWBC();
void exposeOCPWalk();
void exposeMPCWalk();

}  // namespace python
}  // namespace sobec

#endif  // #ifndef __sobec_python__

#ifndef __sobec_python__
#define __sobec_python__

#include <boost/python.hpp>
#include <pinocchio/fwd.hpp>

namespace sobec {
namespace python {

void exposeResidualCoMVelocity();
void exposeResidualVelCollision();
void exposeActivationQuadRef();
void exposeDesigner();
void exposeHorizonManager();
void exposeModelFactory();
void exposeWBC();

}  // namespace python
}  // namespace sobec

#endif  // #ifndef __sobec_python__

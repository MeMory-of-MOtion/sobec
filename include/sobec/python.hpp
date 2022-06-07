#ifndef __sobec_python__
#define __sobec_python__

#include <boost/python.hpp>

namespace sobec {
namespace python {

void exposeResidualCoMVelocity();
void exposeResidualVelCollision();
void exposeResidualCenterOfPressure();
void exposeActivationQuadRef();
}  // namespace python
}  // namespace sobec

#endif  // #ifndef __sobec_python__

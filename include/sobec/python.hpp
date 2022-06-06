#ifndef __sobec_python__
#define __sobec_python__

#include <boost/python.hpp>
#include <pinocchio/fwd.hpp>

namespace sobec {
namespace python {

void exposeResidualCoMVelocity();
void exposeActivationQuadRef();
void exposeDesigner();

}  // namespace python
}  // namespace sobec

#endif  // #ifndef __sobec_python__

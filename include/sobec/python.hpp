#ifndef __sobec_python__
#define __sobec_python__

#include "sobec/gepadd.hpp"
#include "sobec/pin.hpp"

#include <boost/python.hpp>

#include "sobec/gepadd.hpp"

namespace sobec {
void exposeExampleAdder();
void exposeExamplePin();
void exposeActionUniEx();
namespace python{
void exposeResidualCoMVelocity();
}  // namespace python
}  // namespace sobec


#endif

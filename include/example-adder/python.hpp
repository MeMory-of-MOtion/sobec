#ifndef __example_adder_python__
#define __example_adder_python__

#include "example-adder/gepadd.hpp"
#include "example-adder/pin.hpp"

#include <boost/python.hpp>

#include "example-adder/gepadd.hpp"

namespace gepetto {
namespace example {
void exposeExampleAdder();
void exposeExamplePin();
}  // namespace example
}  // namespace gepetto

#endif

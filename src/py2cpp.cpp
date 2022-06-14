#include "sobec/py2cpp.hpp"

#include <boost/python.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <sobec/mpc-walk.hpp>

namespace bp = boost::python;

namespace sobec {
ShootingProblemPtr initShootingProblem(const char* fileName) {
  try {
    Py_Initialize();
    bp::object main_module = bp::import("__main__");
    bp::object main_namespace = main_module.attr("__dict__");
    bp::exec_file(fileName, main_namespace);
    return bp::extract<ShootingProblemPtr>(main_namespace["problem"]);
  } catch (bp::error_already_set&) {
    PyErr_Print();
    throw_pretty(__FILE__ ": python error")
  }
}
MPCWalkPtr initMPCWalk(const char* fileName) {
  try {
    Py_Initialize();
    bp::object main_module = bp::import("__main__");
    bp::object main_namespace = main_module.attr("__dict__");
    bp::exec_file(fileName, main_namespace);
    return bp::extract<MPCWalkPtr>(main_namespace["mpc"]);
  } catch (bp::error_already_set&) {
    PyErr_Print();
    throw_pretty(__FILE__ ": python error")
  }
}
}  // namespace sobec

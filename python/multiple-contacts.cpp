///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2022, LAAS-CNRS, University of Edinburgh, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/multiple-contacts.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>

namespace sobec {
namespace python {
using namespace crocoddyl;
namespace bp = boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ContactModelMultiple_addContact_wrap, sobec::ContactModelMultiple::addContact, 2, 3)

void exposeMultipleContacts() {

  bp::register_ptr_to_python<boost::shared_ptr<sobec::ContactModelMultiple>>();

  bp::class_<sobec::ContactModelMultiple, bp::bases<crocoddyl::ContactModelMultiple> >("ContactModelMultiple",
    bp::init<boost::shared_ptr<crocoddyl::StateMultibody>, bp::optional<std::size_t>>(
        bp::args("self", "state", "nu"),
        "Initialize the multiple contact model.\n\n"
        ":param state: state of the multibody system\n"
        ":param nu: dimension of control vector"));
}

}  // namespace python
}  // namespace sobec

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////
#include "sobec/fwd.hpp"
#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/activation-base.hpp>

#include <eigenpy/eigenpy.hpp>

#include <sobec/horizon_manager.hpp>


namespace sobec {
    namespace python {
    namespace bp = boost::python;

    template<typename T>
    inline
    void py_list_to_std_vector( const bp::object& iterable, std::vector< T > &out)
    {
        out = std::vector< T >( boost::python::stl_input_iterator< T >( iterable ),
                                boost::python::stl_input_iterator< T >( ) );
    }

    void initialize(HorizonManager &self, const bp::dict &settings, 
                                          const Eigen::VectorXd &x0, 
                                          const bp::list runningModels, 
                                          const AMA &terminalModel){
    HorizonManagerSettings conf;
    conf.leftFootName = bp::extract<std::string>(settings["leftFootName"]);
    conf.rightFootName = bp::extract<std::string>(settings["rightFootName"]);

    std::vector<AMA> horizonModels;
    py_list_to_std_vector(runningModels, horizonModels);
    self.initialize(conf, x0, horizonModels, terminalModel);
}

    void exposeHorizonManager() {
        bp::class_<HorizonManager>("HorizonManager", bp::init<>())
            .def("initialize", &initialize, bp::args("self", "settings", "x0", "runningModels", "terminalModel"))
            .def("iam", &HorizonManager::iam, bp::args("self", "time"))
            .def("dam", &HorizonManager::dam, bp::args("self", "time"))
            .def<Cost (HorizonManager::*)(const unsigned long&)>("costs", &HorizonManager::costs, bp::args("self", "time"))
            .def<Contact (HorizonManager::*)(const unsigned long &)>("contacts", &HorizonManager::contacts, bp::args("self", "time"))
            .def("data", &HorizonManager::data, bp::args("self", "time"))
            .def("setPoseReferenceLF", &HorizonManager::setPoseReferenceLF, bp::args("self", "time", "pose"))
            .def("setPoseReferenceRF", &HorizonManager::setPoseReferenceRF, bp::args("self", "time", "pose"))
            .def<void (HorizonManager::*)(const unsigned long &)>("activateContactLF", &HorizonManager::activateContactLF, bp::args("self", "time"))
            .def<void (HorizonManager::*)(const unsigned long &)>("activateContactRF", &HorizonManager::activateContactRF, bp::args("self", "time"))
            .def<void (HorizonManager::*)(const unsigned long &)>("removeContactLF", &HorizonManager::removeContactLF, bp::args("self", "time"))
            .def<void (HorizonManager::*)(const unsigned long &)>("removeContactRF", &HorizonManager::removeContactRF, bp::args("self", "time"))
            .def<void (HorizonManager::*)(const unsigned long &, const eVector6 &)>("setForceReferenceLF", &HorizonManager::setForceReferenceLF, bp::args("self", "time", "ref_wrench"))
            .def<void (HorizonManager::*)(const unsigned long &, const eVector6 &)>("setForceReferenceRF", &HorizonManager::setForceReferenceRF, bp::args("self", "time", "ref_wrench"))
            .def<void (HorizonManager::*)(const unsigned long &)>("setSwingingLF", &HorizonManager::setSwingingLF, bp::args("self", "time"))
            .def<void (HorizonManager::*)(const unsigned long &)>("setSwingingRF", &HorizonManager::setSwingingRF, bp::args("self", "time"))
            .def<void (HorizonManager::*)(const unsigned long &)>("setSupportingLF", &HorizonManager::setSupportingLF, bp::args("self", "time"))
            .def<void (HorizonManager::*)(const unsigned long &)>("setSupportingRF", &HorizonManager::setSupportingRF, bp::args("self", "time"))
            .def<void (HorizonManager::*)(const IAM &, const IAD &)>("recede", &HorizonManager::recede, bp::args("self", "IAM", "IAD"))
            .def<void (HorizonManager::*)(const IAM &)>("recede", &HorizonManager::recede, bp::args("self", "IAM"))
            .def<void (HorizonManager::*)()>("recede", &HorizonManager::recede, bp::args("self"))
        ;
        return;
        }
    }   
}
///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////
#include "sobec/fwd.hpp"
#include <pinocchio/fwd.hpp>  // This line must be the first include
#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/activation-base.hpp>

#include "pinocchio/multibody/model.hpp"
#include <pinocchio/algorithm/model.hpp>
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
        ;

        return;
}


    }   
}
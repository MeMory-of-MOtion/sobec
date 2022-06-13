#include "sobec/fwd.hpp"
#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/enum.hpp>
#include <crocoddyl/core/activation-base.hpp>

#include <eigenpy/eigenpy.hpp>

#include <sobec/wbc.hpp>


namespace sobec {
    namespace python {
        namespace bp = boost::python;

        void initialize(WBC &self, const bp::dict &settings, 
                                const RobotDesigner &designer,
                                const HorizonManager &horizon, 
                                const Eigen::VectorXd &q0,
                                const Eigen::VectorXd &v0) {
            WBCSettings conf;

            conf.horizonSteps = bp::extract<int>(settings["horizonSteps"]);
            conf.totalSteps = bp::extract<int>(settings["totalSteps"]);
            conf.T = bp::extract<int>(settings["T"]);
            conf.TdoubleSupport = bp::extract<int>(settings["TdoubleSupport"]);
            conf.TsingleSupport = bp::extract<int>(settings["TsingleSupport"]);
            conf.Tstep = bp::extract<int>(settings["Tstep"]);
            conf.ddpIteration = bp::extract<int>(settings["ddpIteration"]);
            conf.Dt = bp::extract<double>(settings["Dt"]);
            conf.simu_step = bp::extract<double>(settings["simu_step"]);
            conf.Nc = bp::extract<int>(settings["Nc"]);
            // conf.stepSize = bp::extract<double>(settings["stepSize"]);
            // conf.stepHeight = bp::extract<double>(settings["stepHeight"]);
            // conf.stepDepth = bp::extract<double>(settings["stepDepth"]);
            std::cout<<"Aca llegaa... "<<std::endl;
            self.initialize(conf, designer, horizon, q0, v0);
        }

        void exposeWBC() {

            bp::class_<WBC>("WBC", bp::init<>())
                .def("initialize", &initialize, bp::args("self", "settings", "design", "horizon", "q0", "v0"), "The posture required here is the full robot posture in the order of pinicchio")
                .def("shapeState", &WBC::shapeState, bp::args("self", "q", "v"))
                .def("generateFullCycle", &WBC::generateFullCycle, bp::args("self", "modelMaker"))
                .def("iterate", &WBC::iterate, (bp::arg("self"), bp::arg("iteration"), bp::arg("q_current"), bp::arg("v_current"), bp::arg("is_feasible")=false))
                .def("updateStepCycleTiming", &WBC::updateStepCycleTiming, bp::args("self"))
                .def("timeToSolveDDP", &WBC::timeToSolveDDP, bp::args("self", "iteration"))
                .def("recedeWithFullCycle", &WBC::recedeWithFullCycle, bp::args("self"))
                .add_property("x0", &WBC::get_x0, &WBC::set_x0)
                .add_property("walkingCycle", &WBC::get_walkingCycle, &WBC::set_walkingCycle)
                ;

        }

    }  // namespace
}

#ifndef SOBEC_OCP
#define SOBEC_OCP	

#include "sobec/designer.hpp"
#include "sobec/model_factory.hpp"
#include "sobec/horizon_manager.hpp"

#include <ndcurves/fwd.h>
#include <ndcurves/se3_curve.h>
#include <ndcurves/polynomial.h>
#include <ndcurves/bezier_curve.h>
#include <ndcurves/piecewise_curve.h>

namespace sobec{

    struct OCPSettings {
        public:
            
            // timing
            std::size_t totalSteps = 4;
            unsigned long T = 100;
            unsigned long TdoubleSupport = 50;
            unsigned long TsimpleSupport = 100;
            unsigned long Tstep = TdoubleSupport + TsimpleSupport;
            unsigned long ddpIteration = 1;
            
            double Dt = 1e-2;
            double simu_step = 1e-3;

            unsigned long Nc = round(Dt / simu_step);
            double stepSize = 0.1;
			double stepHeight = 0.03;
			double stepDepth = 0.0;
			double stepYCorrection = 0.005;
    };

    class OCP{

        private:

            OCPSettings OCP_settings_;
            RobotDesigner designer_;
            ModelMaker modelMaker_; 
            HorizonManager horizon_;
            
            Eigen::VectorXd x0_;
            eVector6 wrench_reference_double_;
            eVector6 wrench_reference_simple_;
            ndcurves::piecewise_SE3_t transBezierRight_;
            ndcurves::piecewise_SE3_t transBezierLeft_;
            std::vector<unsigned long> contacts_sequence_;
            
            unsigned long TswitchPhase_;
			unsigned long TswitchTraj_;
			bool swingRightPhase_;
			bool swingRightTraj_;
			std::size_t steps_;
            
            pinocchio::SE3 starting_position_left_;
            pinocchio::SE3 starting_position_right_;
            pinocchio::SE3 final_position_left_;
            pinocchio::SE3 final_position_right_;
            pinocchio::SE3 frame_placement_next_;
            ndcurves::point3_t point_now_;

        public:
            OCP();
            OCP(const OCPSettings &settings,
                const ModelMakerSettings &model_settings,
                const RobotDesignerSettings &design,
                const Eigen::VectorXd &q0,
                const Eigen::VectorXd &v0);
            
            void initialize(const OCPSettings &settings,
					        const ModelMakerSettings &model_settings,
					        const RobotDesigner &design,
					        const Eigen::VectorXd &q0,
					        const Eigen::VectorXd &v0);
                            
            void solveControlCycle(const Eigen::VectorXd &measured_x);
            
            ndcurves::piecewise_SE3_t defineBezier(const double &height, 
                                                        const double &time_init,
                                                        const double &time_final, 
                                                        const pinocchio::SE3 &placement_init,
                                                        const pinocchio::SE3 &placement_final);
            void updateEndPhase();
            void updateOCP(const Eigen::VectorXd &measured_x);
            
    };
}

#endif  // SOBEC_OCP

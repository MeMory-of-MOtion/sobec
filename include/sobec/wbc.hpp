#ifndef SOBEC_WBC
#define SOBEC_WBC	

#include "sobec/designer.hpp"
#include "sobec/model_factory.hpp"
#include "sobec/horizon_manager.hpp"

#include <ndcurves/fwd.h>
#include <ndcurves/se3_curve.h>
#include <ndcurves/polynomial.h>
#include <ndcurves/bezier_curve.h>
#include <ndcurves/piecewise_curve.h>

namespace sobec{

    struct WBCSettings {
        public:
            
            // timing
            int horizonSteps = 2;
            int totalSteps = 4;
            int T = 100;
            int TdoubleSupport = 50;
            int TsingleSupport = 100;
            int Tstep = TdoubleSupport + TsingleSupport;
            int ddpIteration = 1;
            
            double Dt = 1e-2;
            double simu_step = 1e-3;

            int Nc = (int)round(Dt / simu_step);
            // double stepSize = 0.1;
			// double stepHeight = 0.03;
			// double stepDepth = 0.0;
    };

    class WBC{

        private:
            WBCSettings settings_;
            RobotDesigner designer_;
            HorizonManager horizon_;
            HorizonManager fullCycle_;
            
            Eigen::VectorXd x0_;
            

            // timings
            Eigen::ArrayXi t_takeoff_RF_, t_takeoff_LF_, t_land_RF_, t_land_LF_;

            //Memory preallocations:
            std::vector<unsigned long> controlled_joints_id_;
            Eigen::VectorXd x_internal_;

            // eVector6 wrench_reference_double_;
            // eVector6 wrench_reference_simple_;
            // ndcurves::piecewise_SE3_t transBezierRight_;
            // ndcurves::piecewise_SE3_t transBezierLeft_;
            // std::vector<unsigned long> contacts_sequence_;
            
            // unsigned long TswitchPhase_;
			// unsigned long TswitchTraj_;
			// bool swingRightPhase_;
			// bool swingRightTraj_;
			// std::size_t steps_;
            
            // pinocchio::SE3 starting_position_left_;
            // pinocchio::SE3 starting_position_right_;
            // pinocchio::SE3 final_position_left_;
            // pinocchio::SE3 final_position_right_;
            // pinocchio::SE3 frame_placement_next_;
            // ndcurves::point3_t point_now_;

        public:
            WBC();
            WBC(const WBCSettings &settings,
					     const RobotDesigner &design,
                         const HorizonManager &horizon,
					     const Eigen::VectorXd &q0,
						 const Eigen::VectorXd &v0,
                         const std::string &actuationCostName);
            
            void initialize(const WBCSettings &settings,
					        const RobotDesigner &design,
                            const HorizonManager &horizon,
					        const Eigen::VectorXd &q0,
						    const Eigen::VectorXd &v0,
                            const std::string &actuationCostName);
            bool initialized_ = false;

            Eigen::VectorXd shapeState(Eigen::VectorXd q, Eigen::VectorXd v);

            void generateFullCycle(ModelMaker &mm);
            
            void updateStepCycleTiming();
            
            bool timeToSolveDDP(const int &iteration);

            void setDesiredFeetPoses(const int &iteration, const int &time);

            Eigen::VectorXd iterate(const int &iteration, 
								 const Eigen::VectorXd &q_current,
								 const Eigen::VectorXd &v_current,
								 const bool &is_feasible);
                            
            void recedeWithFullCycle();

            // getters and setters
            Eigen::VectorXd get_x0(){return x0_;}
            void set_x0(Eigen::VectorXd x0){x0_=x0;}

            HorizonManager get_walkingCycle(){return fullCycle_;}
            void set_walkingCycle(HorizonManager walkingCycle){fullCycle_=walkingCycle;}

            HorizonManager get_horizon(){return horizon_;}
            void set_horizon(HorizonManager horizon){horizon_= horizon;}

            RobotDesigner get_designer(){return designer_;}
            void set_designer(RobotDesigner designer){designer_=designer;}

            Eigen::VectorXd get_LF_land(){return t_land_LF_.matrix().cast<double>();}
            void set_LF_land(Eigen::VectorXi t){t_land_LF_ = t.array();}

            Eigen::VectorXd get_RF_land(){return t_land_RF_.matrix().cast<double>();}
            void set_RF_land(Eigen::VectorXi t){t_land_RF_ = t.array();}

            Eigen::VectorXd get_LF_takeoff(){return t_takeoff_LF_.matrix().cast<double>();}
            void set_LF_takeoff(Eigen::VectorXi t){t_takeoff_LF_ = t.array();}

            Eigen::VectorXd get_RF_takeoff(){return t_takeoff_RF_.matrix().cast<double>();}
            void set_RF_takeoff(Eigen::VectorXi t){t_takeoff_RF_ = t.array();}



            // void solveControlCycle(const Eigen::VectorXd &measured_x);
            
            // ndcurves::piecewise_SE3_t defineBezier(const double &height, 
            //                                             const double &time_init,
            //                                             const double &time_final, 
            //                                             const pinocchio::SE3 &placement_init,
            //                                             const pinocchio::SE3 &placement_final);
            // void updateEndPhase();
            // void updateOCP(const Eigen::VectorXd &measured_x);
            
    };
}

#endif  // SOBEC_OCP

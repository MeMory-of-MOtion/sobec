#ifndef SOBEC_DESIGNER
#define SOBEC_DESIGNER

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

namespace sobec{

    struct RobotDesignerSettings{
        public:
            std::string urdfPath = "";
            std::string srdfPath = "";
            std::string robotDescription = "";
            std::vector<std::string> controlledJointsNames;

            std::string leftFootName = "";
            std::string rightFootName = "";

    };

    class RobotDesigner{

        private:
            RobotDesignerSettings settings_;
            
            pinocchio::FrameIndex leftFootId_, rightFootId_;

            pinocchio::Model rModelComplete_, rModel_;
            pinocchio::Data rDataComplete_, rData_;
            std::vector<pinocchio::JointIndex> pinocchioControlledJoints_;

            Eigen::VectorXd q0Complete_, q0_;
            Eigen::VectorXd v0Complete_, v0_;
            Eigen::VectorXd x0_;

        public:
            RobotDesigner();
            RobotDesigner(RobotDesignerSettings settings);
            void initialize(RobotDesignerSettings settings);

            void updateReducedModel(Eigen::VectorXd q);
            void updateCompleteModel(Eigen::VectorXd q);

            pinocchio::SE3 get_LF_frame();
            pinocchio::SE3 get_RF_frame();

            double getRobotMass();

            pinocchio::Model &get_rModel(){return rModel_;}
            pinocchio::Model &get_rModelComplete(){return rModelComplete_;}
            pinocchio::Data &get_rData(){return rData_;}
            pinocchio::Data &get_rDataComplete(){return rDataComplete_;}
            Eigen::VectorXd &get_q0(){return q0_;}
            Eigen::VectorXd &get_q0Complete(){return q0Complete_;}
            Eigen::VectorXd &get_x0(){return x0_;}
            
            pinocchio::FrameIndex get_LF_id(){return leftFootId_;}
            pinocchio::FrameIndex get_RF_id(){return rightFootId_;}
    };

}  // namespace
#endif // SOBEC_DESIGNER

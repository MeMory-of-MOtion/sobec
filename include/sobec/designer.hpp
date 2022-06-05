#ifndef SOBEC_DESIGNER
#define SOBEC_DESIGNER

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace sobec{

    struct RobotDesignerSettings{
        public:
            std::string urdf_path = "";
            std::string srdf_path = "";
            std::vector<std::string> controlled_joints_names;

            std::string leftFootName = "";
            std::string rightFootName = "";

    };

    class RobotDesigner{

        private:
            RobotDesignerSettings settings_;

            //std::vector<int> pinocchioControlledJoints_;
            int leftFootId_, rightFootId_;

            pinocchio::Model rModelComplete_, rModel_;
            pinocchio::Data rDataComplete_, rData_;

            Eigen::VectorXd q0Complete_, q0_;
            Eigen::VectorXd v0Complete_, v0_;

        public:
            RobotDesigner();
            RobotDesigner(const RobotDesignerSettings &settings);
            void initialize(const RobotDesignerSettings &settings);

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

    };

}  // namespace
#endif // SOBEC_DESIGNER
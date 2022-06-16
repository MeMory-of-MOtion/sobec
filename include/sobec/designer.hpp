#ifndef SOBEC_DESIGNER
#define SOBEC_DESIGNER

#include <Eigen/Dense>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <string>
#include <vector>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

namespace sobec {

struct RobotDesignerSettings {
 public:
  std::string urdfPath = "";
  std::string srdfPath = "";
  std::string robotDescription = "";
  std::vector<std::string> controlledJointsNames;

  std::string leftFootName = "";
  std::string rightFootName = "";
};

class RobotDesigner {
 private:
  RobotDesignerSettings settings_;

  std::vector<unsigned long> controlled_joints_id_;
  unsigned long leftFootId_, rightFootId_;

  pinocchio::Model rModelComplete_, rModel_;
  pinocchio::Data rDataComplete_, rData_;
  // std::vector<pinocchio::JointIndex> pinocchioControlledJoints_;

  Eigen::VectorXd q0Complete_, q0_;
  Eigen::VectorXd v0Complete_, v0_;
  Eigen::VectorXd x0_;

  Eigen::Vector3d com_position_;
  Eigen::Vector3d LF_position_;
  Eigen::Vector3d RF_position_;

 public:
  RobotDesigner();
  RobotDesigner(const RobotDesignerSettings &settings);
  void initialize(const RobotDesignerSettings &settings);
  bool initialized_ = false;

  void updateReducedModel(Eigen::VectorXd q);
  void updateCompleteModel(Eigen::VectorXd q);

  pinocchio::SE3 get_LF_frame();
  pinocchio::SE3 get_RF_frame();

  double getRobotMass();

  pinocchio::Model &get_rModel() { return rModel_; }
  pinocchio::Model &get_rModelComplete() { return rModelComplete_; }
  pinocchio::Data &get_rData() { return rData_; }
  pinocchio::Data &get_rDataComplete() { return rDataComplete_; }
  Eigen::VectorXd get_q0() { return q0_; }
  Eigen::VectorXd get_v0() { return v0_; }
  Eigen::VectorXd get_q0Complete() { return q0Complete_; }
  Eigen::VectorXd get_v0Complete() { return v0Complete_; }
  Eigen::VectorXd get_x0() { return x0_; }

  std::string get_LF_name() { return settings_.leftFootName; }
  std::string get_RF_name() { return settings_.rightFootName; }
  pinocchio::FrameIndex get_LF_id() { return leftFootId_; }
  pinocchio::FrameIndex get_RF_id() { return rightFootId_; }
  RobotDesignerSettings &get_settings() { return settings_; }
  std::vector<unsigned long> get_controlledJointsIDs() { return controlled_joints_id_; }

  Eigen::Vector3d get_LF_position() { return LF_position_; }
  Eigen::Vector3d get_RF_position() { return RF_position_; }
  Eigen::Vector3d get_com_position() { return com_position_; }
};

}  // namespace sobec
#endif  // SOBEC_DESIGNER

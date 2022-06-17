#ifndef SOBEC_OCP
#define SOBEC_OCP

#include <pinocchio/fwd.hpp>

#include "sobec/designer.hpp"
#include "sobec/foot_trajectory.hpp"
#include "sobec/horizon_manager.hpp"
#include "sobec/model_factory.hpp"

namespace sobec {

struct OCPSettings {
 public:
  // timing
  unsigned long totalSteps = 4;
  unsigned long T = 100;
  unsigned long TdoubleSupport = 50;
  unsigned long TsimpleSupport = 100;
  unsigned long Tstep = TdoubleSupport + TsimpleSupport;
  unsigned long ddpIteration = 1;

  double Dt = 1e-2;
  double simu_step = 1e-3;

  unsigned long Nc = (unsigned long)round(Dt / simu_step);
  double stepSize = 0.1;
  double stepHeight = 0.03;
  double stepDepth = 0.0;
  double stepYCorrection = 0.005;
};

class OCP {
 private:
  OCPSettings OCP_settings_;
  RobotDesigner designer_;
  ModelMaker modelMaker_;
  HorizonManager horizon_;
  FootTrajectory_ptr swing_trajectory_left_;
  FootTrajectory_ptr swing_trajectory_right_;

  Eigen::VectorXd xc_;
  eVector6 wrench_reference_double_;
  eVector6 wrench_reference_simple_;
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

 public:
  OCP();
  OCP(const OCPSettings &settings, const ModelMakerSettings &model_settings,
      const RobotDesignerSettings &design, const Eigen::VectorXd &q0,
      const Eigen::VectorXd &v0);

  void initialize(const OCPSettings &settings,
                  const ModelMakerSettings &model_settings,
                  const RobotDesignerSettings &design,
                  const Eigen::VectorXd &q0, const Eigen::VectorXd &v0);

  void updateEndPhase();
  void updateOCP(const Eigen::VectorXd &qc, const Eigen::VectorXd &vc);
  HorizonManager get_horizon() { return horizon_; }

  eVector3 get_LF_position() { return designer_.get_LF_position(); }
  eVector3 get_RF_position() { return designer_.get_RF_position(); }
  eVector3 get_com_position() { return designer_.get_com_position(); }
};
}  // namespace sobec

#endif  // SOBEC_OCP

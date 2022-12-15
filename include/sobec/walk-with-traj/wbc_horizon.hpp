#ifndef SOBEC_WBC_HORIZON
#define SOBEC_WBC_HORIZON

#include <pinocchio/fwd.hpp>

// include pinocchio first

#include <ndcurves/bezier_curve.h>
#include <ndcurves/fwd.h>
#include <ndcurves/piecewise_curve.h>
#include <ndcurves/polynomial.h>
#include <ndcurves/se3_curve.h>

#include "sobec/walk-with-traj/designer.hpp"
#include "sobec/walk-with-traj/horizon_manager.hpp"
#include "sobec/walk-with-traj/model_factory.hpp"

namespace sobec {

/// @todo: in order to switch between locomotions safely, incorporate a terminal
/// constraint

struct WBCHorizonSettings {
  ///@todo: add the cost names as setting parameters.
 public:
  // timing
  int totalSteps = 4;
  int T = 100;
  int TdoubleSupport = 50;
  int TsingleSupport = 100;
  int Tstep = TdoubleSupport + TsingleSupport;
  int ddpIteration = 1;

  double Dt = 1e-2;
  double simu_step = 1e-3;
  
  double min_force = 150;
  double support_force = 1000;
  
  int Nc = (int)round(Dt / simu_step);
};
class WBCHorizon {
  /**
   * Form to use this class:
   * 1) The function iterate produces the torques to command.
   * 2) All cost references must be updated separtely in the control loop.
   *
   */

 protected:
  WBCHorizonSettings settings_;
  RobotDesigner designer_;
  HorizonManager horizon_;
  HorizonManager fullHorizon_;
  
  eVector3 ref_com_vel_;
  eVector3 ref_com_;
  eVector3 ref_dcm_;
  Eigen::Matrix3d ref_base_rotation_;

  Eigen::VectorXd x0_;

  int horizon_iteration_;
  double yaw_left_;
  double yaw_right_;

  // timings
  std::vector<int> takeoff_RF_, takeoff_LF_, land_RF_, land_LF_;

  // INTERNAL UPDATING functions
  void updateStepTrackerReferences();
  void updateStepTrackerLastReference();
  void updateNonThinkingReferences();

  // References for costs:
  std::vector<pinocchio::SE3> ref_LF_poses_, ref_RF_poses_;

  // Security management
  bool initialized_ = false;

  // Memory preallocations:
  std::vector<unsigned long> controlled_joints_id_;
  Eigen::VectorXd x_internal_;
  bool time_to_solve_ddp_ = false;

 public:
  WBCHorizon();
  WBCHorizon(const WBCHorizonSettings &settings, const RobotDesigner &design,
      const HorizonManager &horizon, const Eigen::VectorXd &q0, 
      const Eigen::VectorXd &v0, const std::string &actuationCostName);

  void initialize(const WBCHorizonSettings &settings, const RobotDesigner &design,
                  const HorizonManager &horizon, const Eigen::VectorXd &q0,
                  const Eigen::VectorXd &v0, const std::string &actuationCostName);

  void updateSupportTiming();

  const Eigen::VectorXd &shapeState(const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &v);
  
  void setForceAlongHorizon();
  
  std::vector<Support> generateSupportCycle();
  
  void generateFullHorizon(ModelMaker &mm, const Experiment &experiment);

  bool timeToSolveDDP(int iteration);

  void iterate(const Eigen::VectorXd &q_current,
               const Eigen::VectorXd &v_current, bool is_feasible);

  void iterate(int iteration, const Eigen::VectorXd &q_current,
               const Eigen::VectorXd &v_current, bool is_feasible);
  void iterateNoThinking(const Eigen::VectorXd &q_current,
               const Eigen::VectorXd &v_current, bool is_feasible);

  void iterateNoThinking(int iteration, const Eigen::VectorXd &q_current,
               const Eigen::VectorXd &v_current, bool is_feasible);
  void iterateNoThinkingWithDelay(const Eigen::VectorXd &q_current,
                                  const Eigen::VectorXd &v_current,  
                                  bool contact_left, 
                                  bool contact_right,
                                  bool is_feasible);
  void recedeWithCycle();
  void goToNextDoubleSupport();

  // getters and setters
  WBCHorizonSettings &get_settings() { return settings_; }

  const Eigen::VectorXd &get_x0() const { return x0_; }
  void set_x0(const Eigen::VectorXd &x0) { x0_ = x0; }

  HorizonManager &get_fullHorizon() { return fullHorizon_; }
  void set_fullHorizon(const HorizonManager &fullHorizon) {
    fullHorizon_ = fullHorizon;
  }

  HorizonManager &get_horizon() { return horizon_; }
  void set_horizon(const HorizonManager &horizon) { horizon_ = horizon; }

  RobotDesigner &get_designer() { return designer_; }
  void set_designer(const RobotDesigner &designer) { designer_ = designer; }

  const std::vector<int> &get_land_LF() { return land_LF_; }
  const std::vector<int> &get_land_RF() { return land_RF_; }
  const std::vector<int> &get_takeoff_LF() { return takeoff_LF_; }
  const std::vector<int> &get_takeoff_RF() { return takeoff_RF_; }
  
  const int &get_horizon_iteration() { return horizon_iteration_; }

  // USER REFERENCE SETTERS AND GETTERS
  const std::vector<pinocchio::SE3> &getPoseRef_LF() { return ref_LF_poses_; }
  const pinocchio::SE3 &getPoseRef_LF(unsigned long time) {
    return ref_LF_poses_[time];
  }
  void setPoseRef_LF(const std::vector<pinocchio::SE3> &ref_LF_poses) {
    ref_LF_poses_ = ref_LF_poses;
  }
  void setPoseRef_LF(const pinocchio::SE3 &ref_LF_pose, unsigned long time) {
    ref_LF_poses_[time] = ref_LF_pose;
  }

  const std::vector<pinocchio::SE3> &getPoseRef_RF() { return ref_RF_poses_; }
  const pinocchio::SE3 &getPoseRef_RF(unsigned long time) {
    return ref_RF_poses_[time];
  }
  void setPoseRef_RF(const std::vector<pinocchio::SE3> &ref_RF_poses) {
    ref_RF_poses_ = ref_RF_poses;
  }
  void setPoseRef_RF(const pinocchio::SE3 &ref_RF_pose, unsigned long time) {
    ref_RF_poses_[time] = ref_RF_pose;
  }
  
  const eVector3 &getCoMRef() { return ref_com_;}
  void setCoMRef(eVector3 ref_com) { ref_com_ = ref_com;}
  
  const Eigen::Matrix3d &getBaseRotRef() { return ref_base_rotation_;}
  void setBaseRotRef(Eigen::Matrix3d ref_base_rotation) { 
	  ref_base_rotation_ = ref_base_rotation;
  }
  
  const eVector3 &getVelRef_COM() {
    return ref_com_vel_;
  }
  void setVelRef_COM(eVector3 ref_com_vel) {
    ref_com_vel_ = ref_com_vel;
  }
  bool horizonEnd() { return horizon_iteration_ == fullHorizon_.size(); }

  // For the python bindings:
  std::vector<pinocchio::SE3> &ref_LF_poses() { return ref_LF_poses_; }
  std::vector<pinocchio::SE3> &ref_RF_poses() { return ref_RF_poses_; }
  eVector3 &ref_com() { return ref_com_; }
  Eigen::Matrix3d &ref_base_rot() { return ref_base_rotation_; }
  eVector3 &ref_com_vel() { return ref_com_vel_; }

};
}  // namespace sobec

#endif  // SOBEC_WBC_HORIZON

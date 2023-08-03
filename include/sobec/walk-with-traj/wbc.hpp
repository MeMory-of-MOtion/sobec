#ifndef SOBEC_WBC
#define SOBEC_WBC

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
/// @todo: bind these enumerations
enum LocomotionType { WALKING, STANDING };
enum supportSwitch { NO_SWITCH, LAND_LF, LAND_RF, TAKEOFF_LF, TAKEOFF_RF };

struct WBCSettings {
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

  int Nc = (int)round(Dt / simu_step);
};
class WBC {
  /**
   * Form to use this class:
   * 1) The function iterate produces the torques to command.
   * 2) All cost references must be updated separtely in the control loop.
   *
   */

 protected:
  WBCSettings settings_;
  RobotDesigner designer_;
  HorizonManager horizon_;
  HorizonManager walkingCycle_;
  HorizonManager standingCycle_;

  eVector3 ref_com_vel_;
  eVector3 ref_com_;
  eVector3 ref_dcm_;
  Eigen::Matrix3d ref_base_rotation_;

  Eigen::VectorXd x0_;

  LocomotionType now_ = WALKING;
  int nWalkingCycles_;
  double yaw_left_;
  double yaw_right_;

  // timings
  std::vector<int> takeoff_RF_, takeoff_LF_, land_RF_, land_LF_;

  int takeoff_RF_cycle_, takeoff_LF_cycle_, land_RF_cycle_, land_LF_cycle_;

  // INTERNAL UPDATING functions
  void updateStepTrackerReferences();
  void updateStepTrackerLastReference();
  void updateNonThinkingReferences();

  // References for costs:
  std::vector<pinocchio::SE3> ref_LF_poses_, ref_RF_poses_;

  // Security management
  bool initialized_ = false;
  void rewindWalkingCycle();

  // Memory preallocations:
  std::vector<unsigned long> controlled_joints_id_;
  Eigen::VectorXd x_internal_;
  bool time_to_solve_ddp_ = false;
  bool first_switch_to_stand_ = true;
  std::set<std::string> contacts_before_, contacts_after_;
  supportSwitch switch_;
  int horizon_end_;

 public:
  WBC();
  WBC(const WBCSettings &settings, const RobotDesigner &design, const HorizonManager &horizon,
      const Eigen::VectorXd &q0, const Eigen::VectorXd &v0, const std::string &actuationCostName);

  void initialize(const WBCSettings &settings, const RobotDesigner &design, const HorizonManager &horizon,
                  const Eigen::VectorXd &q0, const Eigen::VectorXd &v0, const std::string &actuationCostName);

  void initializeSupportTiming();

  void updateSupportTiming();

  const supportSwitch &getSwitches(const unsigned long before, const unsigned long after);

  void generateWalkingCycle(ModelMaker &mm);

  void generateStandingCycle(ModelMaker &mm);

  void generateWalkingCycleNoThinking(ModelMaker &mm);

  void generateStandingCycleNoThinking(ModelMaker &mm);

  void updateStepCycleTiming();

  bool timeToSolveDDP(int iteration);

  void iterate(const Eigen::VectorXd &q_current, const Eigen::VectorXd &v_current, bool is_feasible);

  void iterate(int iteration, const Eigen::VectorXd &q_current, const Eigen::VectorXd &v_current, bool is_feasible);
  void iterateNoThinking(const Eigen::VectorXd &q_current, const Eigen::VectorXd &v_current, bool is_feasible);

  void iterateNoThinking(int iteration, const Eigen::VectorXd &q_current, const Eigen::VectorXd &v_current,
                         bool is_feasible);

  void recedeWithCycle();
  void recedeWithCycle(HorizonManager &cycle);
  void goToNextDoubleSupport();

  // getters and setters
  WBCSettings &get_settings() { return settings_; }

  const Eigen::VectorXd &get_x0() const { return x0_; }
  void set_x0(const Eigen::VectorXd &x0) { x0_ = x0; }

  HorizonManager &get_walkingCycle() { return walkingCycle_; }
  void set_walkingCycle(const HorizonManager &walkingCycle) { walkingCycle_ = walkingCycle; }

  HorizonManager &get_standingCycle() { return standingCycle_; }
  void set_standingCycle(const HorizonManager &standingCycle) { standingCycle_ = standingCycle; }

  HorizonManager &get_horizon() { return horizon_; }
  void set_horizon(const HorizonManager &horizon) { horizon_ = horizon; }

  RobotDesigner &get_designer() { return designer_; }
  void set_designer(const RobotDesigner &designer) { designer_ = designer; }

  const std::vector<int> &get_land_LF() { return land_LF_; }
  const std::vector<int> &get_land_RF() { return land_RF_; }
  const std::vector<int> &get_takeoff_LF() { return takeoff_LF_; }
  const std::vector<int> &get_takeoff_RF() { return takeoff_RF_; }

  const int &get_land_LF_cycle() { return land_LF_cycle_; }
  const int &get_land_RF_cycle() { return land_RF_cycle_; }
  const int &get_takeoff_LF_cycle() { return takeoff_LF_cycle_; }
  const int &get_takeoff_RF_cycle() { return takeoff_RF_cycle_; }

  // USER REFERENCE SETTERS AND GETTERS
  const std::vector<pinocchio::SE3> &getPoseRef_LF() { return ref_LF_poses_; }
  const pinocchio::SE3 &getPoseRef_LF(unsigned long time) { return ref_LF_poses_[time]; }
  void setPoseRef_LF(const std::vector<pinocchio::SE3> &ref_LF_poses) { ref_LF_poses_ = ref_LF_poses; }
  void setPoseRef_LF(const pinocchio::SE3 &ref_LF_pose, unsigned long time) { ref_LF_poses_[time] = ref_LF_pose; }

  const std::vector<pinocchio::SE3> &getPoseRef_RF() { return ref_RF_poses_; }
  const pinocchio::SE3 &getPoseRef_RF(unsigned long time) { return ref_RF_poses_[time]; }
  void setPoseRef_RF(const std::vector<pinocchio::SE3> &ref_RF_poses) { ref_RF_poses_ = ref_RF_poses; }
  void setPoseRef_RF(const pinocchio::SE3 &ref_RF_pose, unsigned long time) { ref_RF_poses_[time] = ref_RF_pose; }

  const eVector3 &getCoMRef() { return ref_com_; }
  void setCoMRef(eVector3 ref_com) { ref_com_ = ref_com; }

  const Eigen::Matrix3d &getBaseRotRef() { return ref_base_rotation_; }
  void setBaseRotRef(Eigen::Matrix3d ref_base_rotation) { ref_base_rotation_ = ref_base_rotation; }

  const eVector3 &getVelRef_COM() { return ref_com_vel_; }
  void setVelRef_COM(eVector3 ref_com_vel) { ref_com_vel_ = ref_com_vel; }

  // For the python bindings:
  std::vector<pinocchio::SE3> &ref_LF_poses() { return ref_LF_poses_; }
  std::vector<pinocchio::SE3> &ref_RF_poses() { return ref_RF_poses_; }
  eVector3 &ref_com() { return ref_com_; }
  Eigen::Matrix3d &ref_base_rot() { return ref_base_rotation_; }
  eVector3 &ref_com_vel() { return ref_com_vel_; }

  void switchToWalk() { now_ = WALKING; }
  void switchToStand() { now_ = STANDING; }
  LocomotionType currentLocomotion() { return now_; }
};
}  // namespace sobec

#endif  // SOBEC_OCP

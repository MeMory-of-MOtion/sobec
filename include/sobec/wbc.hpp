#ifndef SOBEC_WBC
#define SOBEC_WBC

#include <ndcurves/bezier_curve.h>
#include <ndcurves/fwd.h>
#include <ndcurves/piecewise_curve.h>
#include <ndcurves/polynomial.h>
#include <ndcurves/se3_curve.h>

#include "sobec/designer.hpp"
#include "sobec/horizon_manager.hpp"
#include "sobec/model_factory.hpp"

namespace sobec {

/// @todo: in order to switch between locomotions safely, incorporate a terminal
/// constraint
/// @todo: bind these enumerations
enum ControlForm { StepTracker, NonThinking, StairClimber };
enum LocomotionType { WALKING, STANDING };

struct WBCSettings {
  ///@todo: add the cost names as setting parameters.
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
  ControlForm typeOfCommand = StepTracker;
};

class WBC {
  /**
   * Form to use this class:
   * 1) The function iterate produces the torques to command.
   * 2) All cost references must be updated separtely in the control loop.
   *
   */

 private:
  WBCSettings settings_;
  RobotDesigner designer_;
  HorizonManager horizon_;
  HorizonManager walkingCycle_;
  HorizonManager standingCycle_;

  Eigen::VectorXd x0_;

  LocomotionType now_ = WALKING;

  // timings
  Eigen::ArrayXi t_takeoff_RF_, t_takeoff_LF_, t_land_RF_, t_land_LF_;

  // INTERNAL UPDATING functions
  void updateStepTrackerReferences();
  void updateNonThinkingReferences();
  // References for costs:
  std::vector<pinocchio::SE3> ref_LF_poses_, ref_RF_poses_;
  std::vector<eVector3> ref_com_vel_;

  // Memory preallocations:
  std::vector<unsigned long> controlled_joints_id_;
  Eigen::VectorXd x_internal_;

 public:
  WBC();
  WBC(const WBCSettings &settings, const RobotDesigner &design,
      const HorizonManager &horizon, const Eigen::VectorXd &q0,
      const Eigen::VectorXd &v0, const std::string &actuationCostName);

  void initialize(const WBCSettings &settings, const RobotDesigner &design,
                  const HorizonManager &horizon, const Eigen::VectorXd &q0,
                  const Eigen::VectorXd &v0,
                  const std::string &actuationCostName);
  bool initialized_ = false;

  Eigen::VectorXd shapeState(Eigen::VectorXd q, Eigen::VectorXd v);

  void generateWalkigCycle(ModelMaker &mm);

  void generateStandingCycle(ModelMaker &mm);

  void updateStepCycleTiming();

  bool timeToSolveDDP(const int &iteration);

  void setDesiredFeetPoses(const int &iteration, const int &time);

  Eigen::VectorXd iterate(const int &iteration,
                          const Eigen::VectorXd &q_current,
                          const Eigen::VectorXd &v_current,
                          const bool &is_feasible);

  void recedeWithCycle();
  void recedeWithCycle(HorizonManager &cycle);

  // getters and setters
  Eigen::VectorXd get_x0() { return x0_; }
  void set_x0(Eigen::VectorXd x0) { x0_ = x0; }

  HorizonManager get_walkingCycle() { return walkingCycle_; }
  void set_walkingCycle(HorizonManager walkingCycle) {
    walkingCycle_ = walkingCycle;
  }

  HorizonManager get_standingCycle() { return standingCycle_; }
  void set_standingCycle(HorizonManager standingCycle) {
    standingCycle_ = standingCycle;
  }

  HorizonManager get_horizon() { return horizon_; }
  void set_horizon(HorizonManager horizon) { horizon_ = horizon; }

  RobotDesigner get_designer() { return designer_; }
  void set_designer(RobotDesigner designer) { designer_ = designer; }

  Eigen::VectorXd get_LF_land() { return t_land_LF_.matrix().cast<double>(); }
  void set_LF_land(Eigen::VectorXi t) { t_land_LF_ = t.array(); }

  Eigen::VectorXd get_RF_land() { return t_land_RF_.matrix().cast<double>(); }
  void set_RF_land(Eigen::VectorXi t) { t_land_RF_ = t.array(); }

  Eigen::VectorXd get_LF_takeoff() {
    return t_takeoff_LF_.matrix().cast<double>();
  }
  void set_LF_takeoff(Eigen::VectorXi t) { t_takeoff_LF_ = t.array(); }

  Eigen::VectorXd get_RF_takeoff() {
    return t_takeoff_RF_.matrix().cast<double>();
  }
  void set_RF_takeoff(Eigen::VectorXi t) { t_takeoff_RF_ = t.array(); }

  // REFERENCE SETTERS AND GETTERS

  const std::vector<pinocchio::SE3> &getPoseRef_LF() { return ref_LF_poses_; }
  const pinocchio::SE3 &getPoseRef_LF(const unsigned long &time) {
    return ref_LF_poses_[time];
  }
  void setPoseRef_LF(const std::vector<pinocchio::SE3> &ref_LF_poses) {
    ref_LF_poses_ = ref_LF_poses;
  }
  void setPoseRef_LF(const pinocchio::SE3 &ref_LF_pose,
                     const unsigned long &time) {
    ref_LF_poses_[time] = ref_LF_pose;
  }

  const std::vector<pinocchio::SE3> &getPoseRef_RF() { return ref_RF_poses_; }
  const pinocchio::SE3 &getPoseRef_RF(const unsigned long &time) {
    return ref_RF_poses_[time];
  }
  void setPoseRef_RF(const std::vector<pinocchio::SE3> &ref_RF_poses) {
    ref_RF_poses_ = ref_RF_poses;
  }
  void setPoseRef_RF(const pinocchio::SE3 &ref_RF_pose,
                     const unsigned long &time) {
    ref_RF_poses_[time] = ref_RF_pose;
  }

  const std::vector<eVector3> &getVelRef_COM() { return ref_com_vel_; }
  const eVector3 &getVelRef_COM(const unsigned long &time) {
    return ref_com_vel_[time];
  }
  void setVelRef_COM(const std::vector<eVector3> &ref_com_vel) {
    ref_com_vel_ = ref_com_vel;
  }
  void setVelRef_COM(const eVector3 &ref_com_vel, const unsigned long &time) {
    ref_com_vel_[time] = ref_com_vel;
  }

  void switchToWalk() { now_ = WALKING; }
  void switchToStand() { now_ = STANDING; }
  const LocomotionType &currentLocomotion() { return now_; }
};
}  // namespace sobec

#endif  // SOBEC_OCP

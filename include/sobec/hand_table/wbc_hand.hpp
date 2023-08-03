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
#include "sobec/hand_table/model_factory_hand.hpp"

namespace sobec {

/// @todo: in order to switch between locomotions safely, incorporate a terminal
/// constraint
/// @todo: bind these enumerations

struct WBCHandSettings {
  ///@todo: add the cost names as setting parameters.
 public:
  // timing
  int T = 100;
  int TtrackingToContact = 200;
  int Tcontact = 300;
  int ddpIteration = 1;

  double Dt = 1e-2;
  double simu_step = 1e-3;

  int Nc = (int)round(Dt / simu_step);
};
class WBCHand {
  /**
   * Form to use this class:
   * 1) The function iterate produces the torques to command.
   * 2) All cost references must be updated separtely in the control loop.
   *
   */

 protected:
  WBCHandSettings settings_;
  RobotDesigner designer_;
  HorizonManager horizon_;
  HorizonManager fullHorizon_;
  
  eVector3 ref_com_vel_;
  eVector3 ref_com_;
  pinocchio::Force ref_force_;

  Eigen::VectorXd x0_;

  // timings
  int land_hand_, takeoff_hand_, iteration_;

  // INTERNAL UPDATING functions
  void updateStepTrackerReferences();

  // References for costs:
  eVector3 ref_LH_pose_, ref_RH_pose_;

  // Security management
  bool initialized_ = false;

  // Memory preallocations:
  std::vector<unsigned long> controlled_joints_id_;
  Eigen::VectorXd x_internal_;
  bool time_to_solve_ddp_ = false;
  bool first_switch_to_stand_ = true;
  std::set<std::string> contacts_before_, contacts_after_;
  int horizon_end_;

 public:
  WBCHand();
  WBCHand(const WBCHandSettings &settings, const RobotDesigner &design,
      const HorizonManager &horizon, const Eigen::VectorXd &q0,
      const Eigen::VectorXd &v0, const std::string &actuationCostName);

  void initialize(const WBCHandSettings &settings, const RobotDesigner &design,
                  const HorizonManager &horizon, const Eigen::VectorXd &q0,
                  const Eigen::VectorXd &v0,
                  const std::string &actuationCostName);

  void generateFullHorizon(ModelMakerHand &mm);

  void updateStepCycleTiming();

  bool timeToSolveDDP(int iteration);

  void iterate(const Eigen::VectorXd &q_current,
               const Eigen::VectorXd &v_current, bool is_feasible);

  void iterate(int iteration, const Eigen::VectorXd &q_current,
               const Eigen::VectorXd &v_current, bool is_feasible);
  void recedeWithCycle();


  // getters and setters
  WBCHandSettings &get_settings() { return settings_; }

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
  
  const int &get_land_hand() { return land_hand_; }
  const int &get_takeoff_hand() { return takeoff_hand_; }
  const int &get_iteration() { return iteration_; }

  // USER REFERENCE SETTERS AND GETTERS
  const eVector3 &getPoseRef_LH() { return ref_LH_pose_; }
  void setPoseRef_LH(const eVector3 &ref_LH_pose) {
    ref_LH_pose_ = ref_LH_pose;
  }

  const eVector3 &getPoseRef_RH() { return ref_RH_pose_; }
  void setPoseRef_RH(const eVector3 &ref_RH_pose) {
    ref_RH_pose_ = ref_RH_pose;
  }
  
  const eVector3 &getCoMRef() { return ref_com_;}
  void setCoMRef(eVector3 ref_com) { ref_com_ = ref_com;}
  
  const eVector3 &getVelRef_COM() {
    return ref_com_vel_;
  }
  void setVelRef_COM(eVector3 ref_com_vel) {
    ref_com_vel_ = ref_com_vel;
  }
  
  const pinocchio::Force &getForceRef() { return ref_force_; }
  void setForceRef(const pinocchio::Force &ref_force) {
    ref_force_ = ref_force;
  }
  

  // For the python bindings:
  eVector3 &ref_LH_pose() { return ref_LH_pose_; }
  eVector3 &ref_RH_pose() { return ref_RH_pose_; }
  eVector3 &ref_com() { return ref_com_; }
  eVector3 &ref_com_vel() { return ref_com_vel_; }
  pinocchio::Force &ref_force() { return ref_force_; }
};
}  // namespace sobec

#endif  // SOBEC_OCP

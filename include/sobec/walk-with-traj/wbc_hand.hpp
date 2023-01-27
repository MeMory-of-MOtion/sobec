#ifndef SOBEC_WBC
#define SOBEC_WBC

#include <pinocchio/fwd.hpp>

// include pinocchio first

#include "sobec/walk-with-traj/designer.hpp"
#include "sobec/walk-with-traj/horizon_manager.hpp"
#include "sobec/walk-with-traj/model_factory.hpp"

namespace sobec {

struct WBCHandSettings {
 public:
  // timing
  int T = 100;
  int Tduration = 600;
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

  Eigen::VectorXd x0_;
  int iteration_;

  // INTERNAL UPDATING functions
  void updateTrackerReferences();

  // References for costs:
  eVector3 ref_hand_pose_;
  eVector3 ref_com_;

  // Security management
  bool initialized_ = false;

  // Memory preallocations:
  std::vector<unsigned long> controlled_joints_id_;
  Eigen::VectorXd x_internal_;
  bool time_to_solve_ddp_ = false;
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

  const Eigen::VectorXd &shapeState(const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &v);

  void generateFullHorizon(ModelMaker &mm);

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

  // USER REFERENCE SETTERS AND GETTERS
  const int &get_iteration() { return iteration_; }
  const eVector3 &getPoseRefHand() { return ref_hand_pose_; }
  void setPoseRefHand(const eVector3 &ref_hand_pose) {
    ref_hand_pose_ = ref_hand_pose;
  }
  
  const eVector3 &getCoMRef() { return ref_com_;}
  void setCoMRef(eVector3 ref_com) { ref_com_ = ref_com;}

  // For the python bindings:
  eVector3 &ref_hand_pose() { return ref_hand_pose_; }
  eVector3 &ref_com() { return ref_com_; }
};
}  // namespace sobec

#endif  // SOBEC_WBCHand

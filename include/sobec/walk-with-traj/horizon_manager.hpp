#ifndef SOBEC_HORIZON_MANAGER
#define SOBEC_HORIZON_MANAGER

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "sobec/fwd.hpp"
#include "sobec/walk-with-traj/designer.hpp"
#include "sobec/walk-with-traj/model_factory.hpp"

namespace sobec {
struct HorizonManagerSettings {
 public:
  std::string leftFootName = "left_sole_link";
  std::string rightFootName = "right_sole_link";
};

class HorizonManager {
 private:
  HorizonManagerSettings settings_;
  DDP ddp_;

  // prealocated memory:
  boost::shared_ptr<crocoddyl::CostModelResidual> cone_;
  boost::shared_ptr<crocoddyl::CostModelResidual> force_cost_;
  pinocchio::Force force_;
  std::vector<Eigen::VectorXd> warm_xs_;
  std::vector<Eigen::VectorXd> warm_us_;
  Eigen::VectorXd new_ref_;
  unsigned long size_;
  Eigen::VectorXd command_torque_;
  Eigen::VectorXd tr_error_;
  Eigen::VectorXd K_tr_error_;
  eVector3 foot_torque_, foot_force_;
  pinocchio::SE3 pose_;
  int support_size_;
  std::set<std::string> active_contacts_;

 public:
  HorizonManager();

  HorizonManager(const HorizonManagerSettings &horizonSettings, const Eigen::VectorXd &x0,
                 const std::vector<AMA> &runningModels, const AMA &terminalModel);

  void initialize(const HorizonManagerSettings &horizonSettings, const Eigen::VectorXd &x0,
                  const std::vector<AMA> &runningModels, const AMA &terminalModel);
  bool initialized_ = false;

  AMA ama(const unsigned long time);
  IAM iam(const unsigned long time);
  IAM terminaliam();
  DAM dam(const unsigned long time);
  DAM terminaldam();
  ADA ada(const unsigned long time);
  IAD iad(const unsigned long time);
  DAD dad(const unsigned long time);
  pinocchio::Data pinData(const unsigned long time);
  Cost costs(const unsigned long time);
  Cost terminalCosts();
  Contact contacts(const unsigned long time);
  boost::shared_ptr<crocoddyl::StateMultibody> state(const unsigned long time);
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation(const unsigned long time);

  void setActuationReference(const unsigned long time, const std::string &nameCostActuation,
                             const Eigen::VectorXd &reference);
  Eigen::VectorXd getActuationReference(const unsigned long time, const std::string &nameCostActuation);
  void setBalancingTorque(const unsigned long time, const std::string &nameCostActuation, const Eigen::VectorXd &x);
  void setBalancingTorque(const unsigned long time, const std::string &nameCostActuation,
                          const std::string &nameCostState);
  void setPoseReference(const unsigned long time, const std::string &nameCost, const pinocchio::SE3 &ref_placement);
  void setRotationReference(const unsigned long time, const std::string &nameCost,
                            const Eigen::Matrix3d &ref_rotation);
  void setTerminalRotationReference(const std::string &nameCost, const Eigen::Matrix3d &ref_rotation);
  void setTranslationReference(const unsigned long time, const std::string &nameCost, const eVector3 &ref_translation);
  void setTerminalPoseReference(const std::string &nameCost, const pinocchio::SE3 &ref_placement);
  void setTerminalTranslationReference(const std::string &nameCost, const eVector3 &ref_translation);
  const pinocchio::SE3 &getFootPoseReference(const unsigned long time, const std::string &nameCostFootPose);
  const pinocchio::SE3 &getTerminalFootPoseReference(const std::string &nameCostFootPose);
  void setVelocityRefCOM(const unsigned long time, const std::string &nameCost, const eVector3 &ref_placement);
  void setVelocityRefFeet(const unsigned long time, const std::string &nameCost,
                          const pinocchio::Motion &ref_velocity);
  void activateContactLF(const unsigned long time, const std::string &nameContacttLF);
  void activateContactRF(const unsigned long time, const std::string &nameContactRF);
  void removeContactLF(const unsigned long time, const std::string &nameContactLF);
  void removeContactRF(const unsigned long time, const std::string &nameContactRF);
  void changeCostStatus(const unsigned long time, 
                        const std::string &costName,
                        const bool &status);
  void changeTerminalCostStatus(const std::string &costName,
                                const bool &status);                               
  void setForceReference(const unsigned long time, const std::string &nameCost, const eVector6 &reference);
  void setWrenchReference(const unsigned long time, const std::string &nameCost, const eVector6 &reference);
  void setTerminalPoseCoM(const std::string &nameCost, const eVector3 &ref_placement);
  void setSigmoidParameters(const unsigned long time, 
                            const std::string &nameFlyHigh, 
                            const double &height, 
                            const double &dist,
                            const double &height_offset);
  void setTerminalDCMReference(const std::string &nameCost, const eVector3 &ref_translation);
  void setSwingingLF(const unsigned long time, const std::string &nameContactLF, const std::string &nameContactRF,
                     const std::string &nameForceContactLF);
  void setSwingingRF(const unsigned long time, const std::string &nameContactLF, const std::string &nameContactRF,
                     const std::string &nameForceContactRF);
  void setDoubleSupport(const unsigned long time, const std::string &nameContactLF, const std::string &nameContactRF);
  void setSurfaceInequality(const unsigned long time, const std::string &nameCost, const eVector2 &XYpose,
                            const double &orientation);

  const eVector3 &getFootForce(const unsigned long time, const std::string &nameFootForceCost);
  const eVector3 &getFootTorque(const unsigned long time, const std::string &nameFootForceCost);

  const std::set<std::string> &get_contacts(const unsigned long time);

  void recede(const AMA &new_model, const ADA &new_data);
  void recede(const AMA &new_model);
  void recede();

  int supportSize(const unsigned long time);
  unsigned long size();

  void solve(const Eigen::VectorXd &measured_x, const std::size_t ddpIteration, const bool is_feasible = false);
  const Eigen::VectorXd &currentTorques(const Eigen::VectorXd &measured_x);

  DDP get_ddp() { return ddp_; }
  void set_ddp(const DDP &ddp) { ddp_ = ddp; }
};
}  // namespace sobec
#endif  // SOBEC_HORIZON_MANAGER

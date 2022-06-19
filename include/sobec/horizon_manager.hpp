#ifndef SOBEC_HORIZON_MANAGER
#define SOBEC_HORIZON_MANAGER

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "sobec/designer.hpp"
#include "sobec/fwd.hpp"
#include "sobec/model_factory.hpp"

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
  std::vector<Eigen::VectorXd> warm_xs_;
  std::vector<Eigen::VectorXd> warm_us_;
  Eigen::VectorXd new_ref_;
  unsigned long size_;
  Eigen::VectorXd command_torque_;
  Eigen::VectorXd tr_error_;
  Eigen::VectorXd K_tr_error_;
  eVector3 foot_torque_, foot_force_;
  pinocchio::SE3 pose_;

 public:
  HorizonManager();

  HorizonManager(const HorizonManagerSettings &horizonSettings,
                 const Eigen::VectorXd &x0,
                 const std::vector<AMA> &runningModels,
                 const AMA &terminalModel);

  void initialize(const HorizonManagerSettings &horizonSettings,
                  const Eigen::VectorXd &x0,
                  const std::vector<AMA> &runningModels,
                  const AMA &terminalModel);
  bool initialized_ = false;

  AMA ama(unsigned long time);
  IAM iam(unsigned long time);
  DAM dam(unsigned long time);
  ADA ada(unsigned long time);
  IAD iad(unsigned long time);
  Cost costs(unsigned long time);
  Contact contacts(unsigned long time);
  boost::shared_ptr<crocoddyl::StateMultibody> state(unsigned long time);
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation(
      unsigned long time);

  void setActuationReference(unsigned long time,
                             const std::string &nameCostActuation,
                             const Eigen::VectorXd &reference);
  void setBalancingTorque(unsigned long time,
                          const std::string &nameCostActuation,
                          const Eigen::VectorXd &x);
  void setBalancingTorque(unsigned long time,
                          const std::string &nameCostActuation,
                          const std::string &nameCostState);
  void setPoseReferenceLF(unsigned long time,
                          const std::string &nameCostLF,
                          const pinocchio::SE3 &ref_placement);
  void setPoseReferenceRF(unsigned long time,
                          const std::string &nameCostRF,
                          const pinocchio::SE3 &ref_placement);
  const pinocchio::SE3 &getFootPoseReference(unsigned long time, 
                                             const std::string &nameCostFootPose);
  void setVelocityRefCOM(unsigned long time, const std::string &nameCost,
                         const eVector3 &ref_placement);
  void activateContactLF(unsigned long time,
                         const std::string &nameContacttLF);
  void activateContactRF(unsigned long time,
                         const std::string &nameContactRF);
  void removeContactLF(unsigned long time,
                       const std::string &nameContactLF);
  void removeContactRF(unsigned long time,
                       const std::string &nameContactRF);
  void setForceReferenceLF(unsigned long time,
                           const std::string &nameCostLF,
                           const eVector6 &reference);
  void setForceReferenceRF(unsigned long time,
                           const std::string &nameCostRF,
                           const eVector6 &reference);

  void setSwingingLF(unsigned long time,
                     const std::string &nameContactLF,
                     const std::string &nameContactRF,
                     const std::string &nameForceContactLF);
  void setSwingingRF(unsigned long time,
                     const std::string &nameContactLF,
                     const std::string &nameContactRF,
                     const std::string &nameForceContactRF);
  void setDoubleSupport(unsigned long time,
                        const std::string &nameContactLF,
                        const std::string &nameContactRF);

  const eVector3 &getFootForce(unsigned long time,
                               const std::string &nameFootForceCost);
  const eVector3 &getFootTorque(unsigned long time,
                                const std::string &nameFootForceCost);

  void recede(const AMA &new_model, const ADA &new_data);
  void recede(const AMA &new_model);
  void recede();

  unsigned long size();

  void solve(const Eigen::VectorXd &measured_x, std::size_t ddpIteration,
             bool is_feasible = false);
  const Eigen::VectorXd &currentTorques(const Eigen::VectorXd &measured_x);

  DDP get_ddp() { return ddp_; }
  void set_ddp(const DDP &ddp) { ddp_ = ddp; }
};
}  // namespace sobec
#endif  // SOBEC_HORIZON_MANAGER

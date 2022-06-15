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

  AMA ama(const unsigned long &time);
  IAM iam(const unsigned long &time);
  DAM dam(const unsigned long &time);
  ADA ada(const unsigned long &time);
  IAD iad(const unsigned long &time);
  Cost costs(const unsigned long &time);
  Contact contacts(const unsigned long &time);
  boost::shared_ptr<crocoddyl::StateMultibody> state(const unsigned long &time);

  void setActuationReference(const unsigned long &time,
                             const std::string &nameCostActuation,
                             const Eigen::VectorXd &reference);
  void setBalancingTorque(const unsigned long &time,
                          const std::string &nameCostActuation,
                          const Eigen::VectorXd &x);
  void setBalancingTorque(const unsigned long &time,
                          const std::string &nameCostActuation,
                          const std::string &nameCostState);
  void setPoseReferenceLF(const unsigned long &time,
                          const std::string &nameCostLF,
                          const pinocchio::SE3 &ref_placement);
  void setPoseReferenceRF(const unsigned long &time,
                          const std::string &nameCostRF,
                          const pinocchio::SE3 &ref_placement);
  void setVelocityRefCOM(const unsigned long &time, const std::string &nameCost,
                         const eVector3 &ref_placement);
  void activateContactLF(const unsigned long &time,
                         const std::string &nameContacttLF);
  void activateContactRF(const unsigned long &time,
                         const std::string &nameContactRF);
  void removeContactLF(const unsigned long &time,
                       const std::string &nameContactLF);
  void removeContactRF(const unsigned long &time,
                       const std::string &nameContactRF);
  void setForceReferenceLF(const unsigned long &time,
                           const std::string &nameCostLF,
                           const eVector6 &reference);
  void setForceReferenceRF(const unsigned long &time,
                           const std::string &nameCostRF,
                           const eVector6 &reference);

  void setSwingingLF(const unsigned long &time,
                     const std::string &nameContactLF,
                     const std::string &nameContactRF,
                     const std::string &nameForceContactLF);
  void setSwingingRF(const unsigned long &time,
                     const std::string &nameContactLF,
                     const std::string &nameContactRF,
                     const std::string &nameForceContactRF);
  void setDoubleSupport(const unsigned long &time,
                        const std::string &nameContactLF,
                        const std::string &nameContactRF);

  eVector3 getFootForce(const unsigned long &time, const std::string &nameFootForceCost);
  eVector3 getFootTorque(const unsigned long &time, const std::string &nameFootForceCost);

  void recede(const AMA &new_model, const ADA &new_data);
  void recede(const AMA &new_model);
  void recede();

  unsigned long size();

  void solve(const Eigen::VectorXd &measured_x, const std::size_t &ddpIteration,
             const bool &is_feasible = false);
  Eigen::VectorXd currentTorques(const Eigen::VectorXd &measured_x);

  DDP get_ddp() { return ddp_; }
  void set_ddp(const DDP &ddp) { ddp_ = ddp; }
};
}  // namespace sobec
#endif  // SOBEC_HORIZON_MANAGER

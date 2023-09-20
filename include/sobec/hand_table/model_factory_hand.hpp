#ifndef SOBEC_MODEL_FACTORY_HAND
#define SOBEC_MODEL_FACTORY_HAND

#include <pinocchio/fwd.hpp>
// include pinocchio first
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "sobec/fwd.hpp"
#include "sobec/walk-with-traj/designer.hpp"
#include "sobec/walk-with-traj/model_factory.hpp"
#include "sobec/crocomplements/residual-dcm-position.hpp"

namespace sobec {

enum Phase { TRACKING_LEFT, TRACKING_RIGHT, CONTACT_RIGHT, CONTACT_LEFT, NO_HAND };

struct ModelMakerHandSettings {
 public:
  // Timing
  double timeStep = 0.01;

  // physics
  eVector3 gravity = eVector3(0, 0, -9.81);
  double mu = 0.3;
  eVector2 coneBox = eVector2(0.1, 0.05);  // half lenght and width
  double minNforce = 200.0;
  double maxNforce = 1200;

  // geometry
  double comHeight = 0.87;
  double omega = -comHeight / gravity(2);
  double obstacleRadius = 0.05;
  eVector3 obstaclePosition = eVector3(0.6,-0.3,1);
  double obstacleHeight = 2;

  // Croco configuration
  double wHandTranslation = 0;  // 1000;
  double wHandRotation = 0;  // 1000;
  double wHandCollision = 0;  // 1000;
  double wHandVelocity = 0;
  double wStateReg = 0;       // 100;
  double wControlReg = 0;     // 0.001;
  double wLimit = 0;          // 1e3;
  double wForceHand = 0;      // 0.05
  double wFrictionHand = 0;      // 0.05
  double wWrenchCone = 0;
  double wDCM = 0;
  double wCoM = 0;

  Eigen::VectorXd stateWeights;
  Eigen::VectorXd controlWeights;
  
  Eigen::VectorXd lowKinematicLimits;
  Eigen::VectorXd highKinematicLimits;

  double th_stop = 1e-6;  // threshold for stopping criterion
  double th_grad = 1e-9;  // threshold for zero gradient.
};
class ModelMakerHand {
 protected:
  ModelMakerHandSettings settings_;
  RobotDesigner designer_;

  boost::shared_ptr<crocoddyl::StateMultibody> state_;
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;
  Eigen::VectorXd x0_;

 public:
  ModelMakerHand();
  ModelMakerHand(const ModelMakerHandSettings &settings, const RobotDesigner &design);
  void initialize(const ModelMakerHandSettings &settings,
                  const RobotDesigner &design);
  bool initialized_ = false;

  AMA formulateHandTracker(const Phase &phase = Phase::NO_HAND);
  AMA formulateTerminalHandTracker(const Phase &phase = Phase::NO_HAND); 
  AMA formulatePointingTask();
  AMA formulateColFullTask();
  AMA formulateTerminalColFullTask();

  std::vector<AMA> formulateHorizon(const std::vector<Phase> &phases);
  std::vector<AMA> formulateHorizon(const int &T);
  ModelMakerHandSettings &get_settings() { return settings_; }

  // formulation parts:
  void defineFeetContact(Contact &contactCollector);
  void defineHandContact(Contact &contactCollector,
                         const Phase &phase = Phase::CONTACT_RIGHT);
  void defineFeetWrenchCost(Cost &costCollector);
  void defineHandTranslation(Cost &costCollector,
                          const Phase &phase = Phase::CONTACT_RIGHT);
  void defineHandForceTask(Cost &costCollector,
                           const Phase &phase = Phase::CONTACT_RIGHT);
  void defineHandFrictionTask(Cost &costCollector, 
                              const Phase &phase = Phase::CONTACT_RIGHT);
  void defineHandRotation(Cost &costCollector);
  void defineHandVelocity(Cost &costCollector);
  void defineHandCollisionTask(Cost &costCollector);
  void defineCoMPosition(Cost &costCollector);
  void definePostureTask(Cost &costCollector);
  void defineActuationTask(Cost &costCollector);
  void defineJointLimits(Cost &costCollector);
  void defineDCMTask(Cost &costCollector, 
                     const Phase &phase = Phase::NO_HAND);
                     
  boost::shared_ptr<crocoddyl::StateMultibody> getState() { return state_; }
  void setState(const boost::shared_ptr<crocoddyl::StateMultibody> &new_state) {
    state_ = new_state;
  }
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> getActuation() {
    return actuation_;
  }
  void setActuation(
      const boost::shared_ptr<crocoddyl::ActuationModelFloatingBase>
          &new_actuation) {
    actuation_ = new_actuation;
  }
};

}  // namespace sobec
#endif  // SOBEC_MODEL_FACTORY_NO_THINKING

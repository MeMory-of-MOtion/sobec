#ifndef SOBEC_MODEL_FACTORY_NOTHINKING
#define SOBEC_MODEL_FACTORY_NOTHINKING

#include <pinocchio/fwd.hpp>
// include pinocchio first
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "sobec/fwd.hpp"
#include "sobec/walk-with-traj/designer.hpp"
#include "sobec/walk-with-traj/model_factory.hpp"
#include "sobec/crocomplements/residual-feet-collision.hpp"
#include "sobec/crocomplements/residual-fly-high.hpp"
#include "sobec/crocomplements/residual-2D-surface.hpp"
#include "sobec/crocomplements/activation-weighted-log.hpp"

namespace sobec {

struct ModelMakerNoThinkingSettings {
 public:
  // Timing
  double timeStep = 0.01;

  // physics
  eVector3 gravity = eVector3(0, 0, -9.81);

  // geometry
  double footSize = 0.05;  //[m]

  double mu = 0.3;
  eVector2 coneBox = eVector2(0.1, 0.05);  // half lenght and width
  double minNforce = 200.0;
  double maxNforce = 1200;

  double comHeight = 0.87;
  double omega = -comHeight / gravity(2);

  // Croco configuration
  double wFootPlacement = 0;  // 1000;
  double wStateReg = 0;       // 100;
  double wControlReg = 0;     // 0.001;
  double wLimit = 0;          // 1e3;
  double wWrenchCone = 0;     // 0.05;
  double wCoP = 0;            // 1;
  double wVCoM = 0;           // 0;
  double wFootRot = 0;        // 100;
  double wGroundCol = 0;      // 0.05;
  double wCoM = 0;
  double wFlyHigh = 0;
  double wVelFoot = 0;
  double wColFeet = 0;
  double wSurface = 0;
  
  double flyHighSlope = 2;
  double footMinimalDistance = 0.2;

  Eigen::VectorXd stateWeights;
  Eigen::VectorXd controlWeights;
  
  Eigen::VectorXd lowKinematicLimits;
  Eigen::VectorXd highKinematicLimits;

  double th_stop = 1e-6;  // threshold for stopping criterion
  double th_grad = 1e-9;  // threshold for zero gradient.

  // Croco configuration
};
class ModelMakerNoThinking {
  
 protected:
  ModelMakerNoThinkingSettings settings_;
  RobotDesigner designer_;

  boost::shared_ptr<crocoddyl::StateMultibody> state_;
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;
  Eigen::VectorXd x0_;
 
 public:
  ModelMakerNoThinking();
  ModelMakerNoThinking(const ModelMakerNoThinkingSettings &settings, const RobotDesigner &design);
  void initialize(const ModelMakerNoThinkingSettings &settings,
                  const RobotDesigner &design);
  bool initialized_ = false;

  AMA formulateStepTracker(const Support &support = Support::DOUBLE);
  AMA formulateTerminalStepTracker(const Support &support = Support::DOUBLE);

  std::vector<AMA> formulateHorizon(const std::vector<Support> &supports);
  std::vector<AMA> formulateHorizon(const int &T);
  ModelMakerNoThinkingSettings &get_settings() { return settings_; }

  // formulation parts:
  void defineFeetContact(Contact &contactCollector,
                         const Support &support = Support::DOUBLE);
  void defineFeetWrenchCost(Cost &costCollector,
                            const Support &support = Support::DOUBLE);

  void definePostureTask(Cost &costCollector);
  void defineActuationTask(Cost &costCollector);
  void defineCoPTask(Cost &costCollector,
                     const Support &support = Support::DOUBLE);
  void defineZFeetTracking(Cost &costCollector,
                          const Support &support = Support::DOUBLE);

  void defineCoMVelocity(Cost &costCollector);
  void defineVelFootTask(Cost &costCollector,
                         const Support &support = Support::DOUBLE); 
  void defineCoMTask(Cost &costCollector);
  void defineFeetRotation(Cost &costCollector); 
  void defineGroundCollisionTask(Cost &costCollector);
  void defineFootCollisionTask(Cost &costCollector);
  void defineJointLimits(Cost &costCollector);
  void defineFlyHighTask(Cost &costCollector, 
                          const Support &support = Support::DOUBLE);
  void define2DSurfaceTask(Cost &costCollector, 
                          const Support &support = Support::DOUBLE);
                          
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
#endif  // SOBEC_MODEL_FACTORY_NOTHINKING

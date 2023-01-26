#ifndef SOBEC_MODEL_FACTORY
#define SOBEC_MODEL_FACTORY

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/fcl.hpp>
// include pinocchio first
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "sobec/crocomplements/activation-weighted-log.hpp"
#include "sobec/crocomplements/residual-2D-surface.hpp"
#include "sobec/crocomplements/residual-dcm-position.hpp"
#include "sobec/crocomplements/residual-feet-collision.hpp"
#include "sobec/crocomplements/residual-fly-angle.hpp"
#include "sobec/crocomplements/residual-fly-high.hpp"
#include "sobec/fwd.hpp"
#include "sobec/walk-with-traj/designer.hpp"

#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_object.h>
#include <crocoddyl/multibody/residuals/pair-collision.hpp>
#include <crocoddyl/core/activations/2norm-barrier.hpp>

namespace sobec {

enum Support { LEFT, RIGHT, DOUBLE };
enum Experiment { WALK, WWT, WWT_STAIRS };

struct ModelMakerSettings {
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
  double height = 0;
  double dist = 0;
  double width = 0;

  // Croco configuration
  double wFootPlacement = 0;  // 1000;
  double wStateReg = 0;       // 100;
  double wControlReg = 0;     // 0.001;
  double wLimit = 0;          // 1e3;
  double wWrenchCone = 0;     // 0.05;
  double wForceTask = 0;      // 0.05
  double wCoP = 0;            // 1;
  double wDCM = 0;
  double wBaseRot = 0;
  double wVCoM = 0;     // 0;
  double wFootRot = 0;  // 100;
  double wPCoM = 0;
  double wFlyHigh = 0;
  double wVelFoot = 0;
  double wColFeet = 0;
  double wGripperPos = 0;
  double wGripperRot = 0;
  double wGripperVel = 0;

  double flyHighSlope = 2;
  double footMinimalDistance = 0.2;

  Eigen::VectorXd stateWeights;
  Eigen::VectorXd controlWeights;
  Eigen::VectorXd forceWeights;

  Eigen::VectorXd lowKinematicLimits;
  Eigen::VectorXd highKinematicLimits;

  double th_stop = 1e-6;  // threshold for stopping criterion
  double th_grad = 1e-9;  // threshold for zero gradient.
};
class ModelMaker {
 protected:
  ModelMakerSettings settings_;
  RobotDesigner designer_;

  boost::shared_ptr<crocoddyl::StateMultibody> state_;
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;
  Eigen::VectorXd x0_;

 public:
  ModelMaker();
  ModelMaker(const ModelMakerSettings &settings, const RobotDesigner &design);
  void initialize(const ModelMakerSettings &settings, const RobotDesigner &design);
  bool initialized_ = false;

  AMA formulateStepTracker(const Support &support = Support::DOUBLE);
  AMA formulateTerminalStepTracker(const Support &support = Support::DOUBLE);
  AMA formulateWWT(const Support &support = Support::DOUBLE, const bool &stairs = false);
  AMA formulateTerminalWWT(const Support &support = Support::DOUBLE, const bool &stairs = false);
  AMA formulatePointingTask();
  AMA formulateColFullTask();
  AMA formulateTerminalColFullTask();

  std::vector<AMA> formulateHorizon(const std::vector<Support> &supports, const Experiment &experiment);
  std::vector<AMA> formulateHorizon(const int &T);
  ModelMakerSettings &get_settings() { return settings_; }

  // formulation parts:
  void defineFeetForceTask(Cost &costCollector, const Support &support = Support::DOUBLE);
  void defineFeetContact(Contact &contactCollector, const Support &support = Support::DOUBLE);
  void defineFeetWrenchCost(Cost &costCollector, const Support &support = Support::DOUBLE);
  void defineFeetTracking(Cost &costCollector, const Support &support = Support::DOUBLE);
  void defineFeetTranslation(Cost &costCollector, const Support &support = Support::DOUBLE,
                             const bool &stairs = false);
  void definePostureTask(Cost &costCollector);
  void defineRotationBase(Cost &costCollector);
  void defineActuationTask(Cost &costCollector);
  void defineJointLimits(Cost &costCollector);
  void defineCoPTask(Cost &costCollector, const Support &support = Support::DOUBLE);
  void defineDCMTask(Cost &costCollector, const Support &support = Support::DOUBLE);
  void defineVelFootTask(Cost &costCollector, const Support &support = Support::DOUBLE);
  void defineFeetRotation(Cost &costCollector);
  void defineFeetZRotation(Cost &costCollector);
  void defineFootCollisionTask(Cost &costCollector);
  void defineGripperCollisionTask(Cost &costCollector);
  void defineFlyHighTask(Cost &costCollector, const Support &support = Support::DOUBLE);

  void defineCoMPosition(Cost &costCollector);
  void defineCoMVelocity(Cost &costCollector);
  void defineGripperPlacement(Cost &costCollector);
  void defineGripperVelocity(Cost &costCollector);

  boost::shared_ptr<crocoddyl::StateMultibody> getState() { return state_; }
  void setState(const boost::shared_ptr<crocoddyl::StateMultibody> &new_state) { state_ = new_state; }
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> getActuation() { return actuation_; }
  void setActuation(const boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> &new_actuation) {
    actuation_ = new_actuation;
  }
};

}  // namespace sobec
#endif  // SOBEC_MODEL_FACTORY

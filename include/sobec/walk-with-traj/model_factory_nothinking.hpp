#ifndef SOBEC_MODEL_FACTORY_NOTHINKING
#define SOBEC_MODEL_FACTORY_NOTHINKING

#include <pinocchio/fwd.hpp>
// include pinocchio first
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "sobec/walk-with-traj/model_factory.hpp"
#include "sobec/fwd.hpp"
#include "sobec/walk-with-traj/designer.hpp"
#include "sobec/crocomplements/residual-feet-collision.hpp"
#include "sobec/crocomplements/residual-fly-high.hpp"

namespace sobec {

struct ModelMakerNoThinkingSettings : ModelMakerSettings {
 public:
  // Timing
  double flyHighSlope = 2;
  double footMinimalDistance = 0.2;

  // Croco configuration
  double wVCoM = 0;           // 0;
  double wFootRot = 0;        // 100;
  double wGroundCol = 0;      // 0.05;
  double wCoM = 0;
  double wFlyHigh = 0;
  double wVelFoot = 0;
  double wColFeet = 0;
};
class ModelMakerNoThinking : public ModelMaker  {
  
 protected:
 ModelMakerNoThinkingSettings settings_;
 
 public:
  ModelMakerNoThinking();
  ModelMakerNoThinking(const ModelMakerNoThinkingSettings &settings, const RobotDesigner &design);
  void initialize(const ModelMakerNoThinkingSettings &settings,
                  const RobotDesigner &design);
  bool initialized_ = false;

  AMA formulateNoThinking(const Support &support = Support::DOUBLE);
  AMA formulateNoThinkingTerminal(const Support &support = Support::DOUBLE);

  std::vector<AMA> formulateHorizon(const std::vector<Support> &supports);
  std::vector<AMA> formulateHorizon(const int &T);
  ModelMakerNoThinkingSettings &get_settings() { return settings_; }

  // formulation parts:
  void defineZFeetTracking(Cost &costCollector,
                          const Support &support = Support::DOUBLE);

  void defineCoMVelocity(Cost &costCollector);
  void defineVelFootTask(Cost &costCollector); 
  void defineCoMTask(Cost &costCollector);
  void defineFeetRotation(Cost &costCollector); 
  void defineGroundCollisionTask(Cost &costCollector);
  void defineFootCollisionTask(Cost &costCollector);
  void defineFlyHighTask(Cost &costCollector, 
                          const Support &support = Support::DOUBLE);

};

}  // namespace sobec
#endif  // SOBEC_MODEL_FACTORY_NOTHINKING

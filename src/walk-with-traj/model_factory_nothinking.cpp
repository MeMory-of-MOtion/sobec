#include "sobec/walk-with-traj/model_factory_nothinking.hpp"

#include <crocoddyl/multibody/fwd.hpp>

#include "sobec/walk-with-traj/designer.hpp"

namespace sobec {

ModelMakerNoThinking::ModelMakerNoThinking() {}

ModelMakerNoThinking::ModelMakerNoThinking(const ModelMakerNoThinkingSettings &settings,
                       const RobotDesigner &designer) {
  initialize(settings, designer);
}

void ModelMakerNoThinking::initialize(const ModelMakerNoThinkingSettings &settings,
                            const RobotDesigner &designer) {
  settings_ = settings;
  designer_ = designer;

  state_ = boost::make_shared<crocoddyl::StateMultibody>(
      boost::make_shared<pinocchio::Model>(designer_.get_rModel()));
  actuation_ =
      boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);

  x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
  x0_ << designer_.get_q0(), Eigen::VectorXd::Zero(designer_.get_rModel().nv);
  initialized_ = true;
}

void ModelMakerNoThinking::defineZFeetTracking(Cost &costCollector, const Support &support) {
  eVector3 ZFootTrackingVec;
  ZFootTrackingVec << 0, 0, 1;
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationZ =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(ZFootTrackingVec);

  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation>
      residual_LF_Tracking =
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
              state_, designer_.get_LF_id(), designer_.get_LF_frame().translation(),
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation>
      residual_RF_Tracking =
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
              state_, designer_.get_RF_id(), designer_.get_RF_frame().translation(),
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationZ,
                                                       residual_LF_Tracking);
  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationZ,
                                                       residual_RF_Tracking);
  
  costCollector.get()->addCost("Z_translation_LF", trackingModel_LF,
                               settings_.wFootPlacement, false);
  costCollector.get()->addCost("Z_translation_RF", trackingModel_RF,
                               settings_.wFootPlacement, false);
  if (support == Support::LEFT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("Z_translation_LF",true);
  if (support == Support::RIGHT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("Z_translation_RF",true);
}

void ModelMakerNoThinking::defineFeetRotation(Cost &costCollector) {
  boost::shared_ptr<crocoddyl::ResidualModelFrameRotation>
      residual_LF_Rotation =
          boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
              state_, designer_.get_LF_id(), designer_.get_LF_frame().rotation(),
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelFrameRotation>
      residual_RF_Rotation =
          boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
              state_, designer_.get_RF_id(), designer_.get_RF_frame().rotation(),
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> rotationModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, 
                                                       residual_LF_Rotation);
  boost::shared_ptr<crocoddyl::CostModelAbstract> rotationModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, 
                                                       residual_RF_Rotation);
  
  costCollector.get()->addCost("rotation_LF", rotationModel_LF,
                               settings_.wFootRot, true);
  costCollector.get()->addCost("rotation_RF", rotationModel_RF,
                               settings_.wFootRot, true);
}

void ModelMakerNoThinking::defineCoMTask(Cost &costCollector) {
  boost::shared_ptr<crocoddyl::CostModelAbstract> comCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelCoMPosition>(
              state_, designer_.get_com_position(), actuation_->get_nu()));
  costCollector.get()->addCost("comTask", comCost,
                               settings_.wCoM, true);
}

void ModelMakerNoThinking::defineCoMVelocity(Cost &costCollector) {
  eVector3 refVelocity = eVector3::Zero();
  boost::shared_ptr<crocoddyl::CostModelAbstract> CoMVelocityCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<ResidualModelCoMVelocity>(
                      state_, refVelocity, actuation_->get_nu()));

  costCollector.get()->addCost("comVelocity", CoMVelocityCost, settings_.wVCoM,
                               true);
}

void ModelMakerNoThinking::defineFlyHighTask(Cost &costCollector, const Support &support) {
  boost::shared_ptr<ResidualModelFlyHigh> flyHighResidualRight = 
      boost::make_shared<ResidualModelFlyHigh>(
          state_, designer_.get_RF_id(), settings_.flyHighSlope / 2.0, actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelResidual> flyHighCostRight =
          boost::make_shared<crocoddyl::CostModelResidual>(state_, flyHighResidualRight);
          
  boost::shared_ptr<ResidualModelFlyHigh> flyHighResidualLeft = 
      boost::make_shared<ResidualModelFlyHigh>(
          state_, designer_.get_LF_id(), settings_.flyHighSlope / 2.0, actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> flyHighCostLeft =
          boost::make_shared<crocoddyl::CostModelResidual>(state_, flyHighResidualLeft);
  
  costCollector.get()->addCost("flyHigh_RF", flyHighCostRight, settings_.wFlyHigh,false);
  costCollector.get()->addCost("flyHigh_LF", flyHighCostLeft, settings_.wFlyHigh,false);
  if (support == Support::LEFT)
    costCollector.get()->changeCostStatus("flyHigh_RF",true);
  if (support == Support::RIGHT)
    costCollector.get()->changeCostStatus("flyHigh_LF",true);
}

void ModelMakerNoThinking::defineVelFootTask(Cost &costCollector) {
  boost::shared_ptr<crocoddyl::ResidualModelFrameVelocity> verticalFootVelResidualLeft =
      boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(
              state_, designer_.get_LF_id(), pinocchio::Motion::Zero(),
              pinocchio::LOCAL_WORLD_ALIGNED, actuation_->get_nu());
  boost::shared_ptr<crocoddyl::ResidualModelFrameVelocity> verticalFootVelResidualRight =
      boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(
              state_, designer_.get_RF_id(), pinocchio::Motion::Zero(),
              pinocchio::LOCAL_WORLD_ALIGNED, actuation_->get_nu());
  eVector6 verticalFootVelActVec;
  verticalFootVelActVec << 0, 0, 1, 0, 0, 0;
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> verticalFootVelAct = 
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(verticalFootVelActVec);
  
  boost::shared_ptr<crocoddyl::CostModelAbstract> verticalFootVelCostRight = 
      boost::make_shared<crocoddyl::CostModelResidual>(
      state_, verticalFootVelAct, verticalFootVelResidualRight);
  boost::shared_ptr<crocoddyl::CostModelAbstract> verticalFootVelCostLeft = 
      boost::make_shared<crocoddyl::CostModelResidual>(
      state_, verticalFootVelAct, verticalFootVelResidualLeft);
  costCollector.get()->addCost("velFoot_RF", verticalFootVelCostRight, settings_.wVelFoot,true);
  costCollector.get()->addCost("velFoot_LF", verticalFootVelCostLeft, settings_.wVelFoot,true);
}

void ModelMakerNoThinking::defineFootCollisionTask(Cost &costCollector) {
	
	std::list<pinocchio::FrameIndex> leftIds = {designer_.get_LF_id(), designer_.get_LF_toe_id(),
                                                 designer_.get_LF_heel_id()};
    std::list<pinocchio::FrameIndex> rightIds = {designer_.get_RF_id(), designer_.get_RF_toe_id(),
                                                designer_.get_RF_heel_id()};
    for (pinocchio::FrameIndex id1 : leftIds) {
        for (pinocchio::FrameIndex id2 : rightIds) {
            boost::shared_ptr<sobec::ResidualModelFeetCollision> feetColResidual =
                boost::make_shared<sobec::ResidualModelFeetCollision>(
                state_, id1, id2, actuation_->get_nu());
            Eigen::VectorXd feetColLow(1), feetColUp(1);
            feetColLow << settings_.footMinimalDistance;
            feetColUp << 1000;
            const crocoddyl::ActivationBounds feetColBounds = crocoddyl::ActivationBounds(feetColLow, feetColUp);
            boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> feetColAct =
                boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(feetColBounds);
            boost::shared_ptr<crocoddyl::CostModelAbstract> feetColCost = boost::make_shared<crocoddyl::CostModelResidual>(
                state_, feetColAct, feetColResidual);
            costCollector.get()->addCost("feetcol_" + designer_.get_rModel().frames[id1].name +
                                 "_VS_" + designer_.get_rModel().frames[id2].name,
                             feetColCost, settings_.wColFeet, true);
          }
      }
}

void ModelMakerNoThinking::defineGroundCollisionTask(Cost &costCollector) {
  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation> groundColResRight =
      boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
          state_,  designer_.get_RF_id(), Eigen::Vector3d::Zero(), actuation_->get_nu());
  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation> groundColResLeft =
      boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
          state_,  designer_.get_LF_id(), Eigen::Vector3d::Zero(), actuation_->get_nu());
  Eigen::VectorXd groundColLow(3), groundColUp(3);
  groundColLow << -1000, -1000, 0;
  groundColUp << 1000, 1000, 1000;
  const crocoddyl::ActivationBounds groundColBounds = crocoddyl::ActivationBounds(groundColLow, groundColUp);
  boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> groundColAct =
      boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(groundColBounds);
  boost::shared_ptr<crocoddyl::CostModelAbstract> groundColCostRight = 
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, groundColAct, groundColResRight);
  boost::shared_ptr<crocoddyl::CostModelAbstract> groundColCostLeft = 
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, groundColAct, groundColResLeft);
  
  costCollector.get()->addCost("ground_LF", groundColCostLeft,
                               settings_.wGroundCol, true);
  costCollector.get()->addCost("ground_RF", groundColCostRight,
                               settings_.wGroundCol, true);
}

AMA ModelMakerNoThinking::formulateNoThinking(const Support &support) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);

  defineCoMVelocity(costs);
  defineJointLimits(costs);
  definePostureTask(costs);
  defineActuationTask(costs);
  defineFeetWrenchCost(costs, support);
  defineFootCollisionTask(costs);
  defineCoPTask(costs, support);
  defineVelFootTask(costs);
  defineFlyHighTask(costs, support);
  defineZFeetTracking(costs, support);
  defineGroundCollisionTask(costs);

  DAM runningDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMakerNoThinking::formulateNoThinkingTerminal(const Support &support) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);

  defineCoMVelocity(costs);
  defineJointLimits(costs);
  definePostureTask(costs);
  defineFeetWrenchCost(costs, support);
  defineFootCollisionTask(costs);
  defineCoPTask(costs, support);
  defineVelFootTask(costs);
  defineFlyHighTask(costs, support);
  defineCoMTask(costs);
  defineZFeetTracking(costs);
  defineGroundCollisionTask(costs);

  DAM runningDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      runningDAM, settings_.timeStep);

  return runningModel;
}

std::vector<sobec::AMA> ModelMakerNoThinking::formulateHorizon(
    const std::vector<Support> &supports) {
  // for loop to generate a vector of IAMs
  std::vector<sobec::AMA> models;
  for (std::size_t i = 0; i < supports.size(); i++) {
	models.push_back(formulateNoThinking(supports[i]));
  }
  return models;
}

std::vector<sobec::AMA> ModelMakerNoThinking::formulateHorizon(const int &T) {
  std::vector<Support> supports(T, DOUBLE);
  return formulateHorizon(supports);
}

}  // namespace sobec

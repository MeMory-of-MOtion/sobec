#include "sobec/walk-without-think/model_factory_nothinking.hpp"

#include <crocoddyl/multibody/fwd.hpp>

#include "sobec/crocomplements/residual-cop.hpp"
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

void ModelMakerNoThinking::defineFeetContact(Contact &contactCollector,
                                   const Support &support) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelLeft =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_LF_id(), designer_.get_LF_frame(),
          actuation_->get_nu(), eVector2(0., 50.));

  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelRight =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_RF_id(), designer_.get_RF_frame(),
          actuation_->get_nu(), eVector2(0., 50.));

  contactCollector->addContact(designer_.get_LF_name(), ContactModelLeft,
                               false);
  contactCollector->addContact(designer_.get_RF_name(), ContactModelRight,
                               false);

  if (support == Support::LEFT || support == Support::DOUBLE)
    contactCollector->changeContactStatus(designer_.get_LF_name(), true);

  if (support == Support::RIGHT || support == Support::DOUBLE)
    contactCollector->changeContactStatus(designer_.get_RF_name(), true);
}

void ModelMakerNoThinking::defineFeetWrenchCost(Cost &costCollector,
                                      const Support &support) {
  double Mg = -designer_.getRobotMass() * settings_.gravity(2);
  double Fz_ref;
  support == Support::DOUBLE ? Fz_ref = Mg / 2 : Fz_ref = Mg;

  Eigen::Matrix3d coneRotationLeft =
      designer_.get_LF_frame().rotation().transpose();
  Eigen::Matrix3d coneRotationRight =
      designer_.get_RF_frame().rotation().transpose();

  crocoddyl::WrenchCone wrenchCone_LF =
      crocoddyl::WrenchCone(coneRotationLeft, settings_.mu, settings_.coneBox,
                            4, true, settings_.minNforce, settings_.maxNforce);
  crocoddyl::WrenchCone wrenchCone_RF =
      crocoddyl::WrenchCone(coneRotationRight, settings_.mu, settings_.coneBox,
                            4, true, settings_.minNforce, settings_.maxNforce);

  eVector6 refWrench_LF = eVector6::Zero();
  eVector6 refWrench_RF = eVector6::Zero();
  if (support == Support::LEFT || support == Support::DOUBLE)
    refWrench_LF(2) = Fz_ref;
  if (support == Support::RIGHT || support == Support::DOUBLE)
    refWrench_RF(2) = Fz_ref;

  Eigen::VectorXd refCost_LF = wrenchCone_LF.get_A() * refWrench_LF;
  Eigen::VectorXd refCost_RF = wrenchCone_LF.get_A() * refWrench_RF;

  boost::shared_ptr<ActivationModelQuadRef> activation_LF_Wrench =
      boost::make_shared<ActivationModelQuadRef>(refCost_LF);
  boost::shared_ptr<ActivationModelQuadRef> activation_RF_Wrench =
      boost::make_shared<ActivationModelQuadRef>(refCost_RF);

  boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone>
      residual_LF_Wrench =
          boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(
              state_, designer_.get_LF_id(), wrenchCone_LF,
              actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activation_LF_Wrench, residual_LF_Wrench);

  boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone>
      residual_RF_Wrench =
          boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(
              state_, designer_.get_RF_id(), wrenchCone_RF,
              actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activation_RF_Wrench, residual_RF_Wrench);

  costCollector.get()->addCost("wrench_LF", wrenchModel_LF,
                               settings_.wWrenchCone, false);
  costCollector.get()->addCost("wrench_RF", wrenchModel_RF,
                               settings_.wWrenchCone, false);
  if (support == Support::LEFT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("wrench_LF",true);
  if (support == Support::RIGHT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("wrench_RF",true);
}

void ModelMakerNoThinking::defineFeetForceTask(Cost &costCollector, const Support &support) {
  double Mg = -designer_.getRobotMass() * settings_.gravity(2);
  double Fz_ref;
  support == Support::DOUBLE ? Fz_ref = Mg / 2 : Fz_ref = Mg;
  
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationForce =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
          settings_.forceWeights);

  eVector6 refWrench_LF = eVector6::Zero();
  eVector6 refWrench_RF = eVector6::Zero();
  if (support == Support::LEFT || support == Support::DOUBLE)
    refWrench_LF(2) = Fz_ref;
  if (support == Support::RIGHT || support == Support::DOUBLE)
    refWrench_RF(2) = Fz_ref;
  
  pinocchio::Force refForce_LF = pinocchio::Force(refWrench_LF);
  pinocchio::Force refForce_RF = pinocchio::Force(refWrench_RF);
  
  boost::shared_ptr<crocoddyl::ResidualModelContactForce>
      residual_LF_Force =
          boost::make_shared<crocoddyl::ResidualModelContactForce>(
              state_, designer_.get_LF_id(), refForce_LF, 6,
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelContactForce>
      residual_RF_Force =
          boost::make_shared<crocoddyl::ResidualModelContactForce>(
              state_, designer_.get_RF_id(), refForce_RF, 6,
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationForce,
                                                       residual_LF_Force);
  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationForce,
                                                       residual_RF_Force);
  
  costCollector.get()->addCost("force_LF", forceModel_LF,
                               settings_.wWrenchCone, false);
  costCollector.get()->addCost("force_RF", forceModel_RF,
                               settings_.wWrenchCone, false);
  if (support == Support::RIGHT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("force_RF",true);
  if (support == Support::LEFT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("force_LF",true);
}

void ModelMakerNoThinking::defineZFeetTracking(Cost &costCollector, const Support &support) {
  eVector3 ZFootTrackingVec;
  ZFootTrackingVec << 0, 0, 1;
  boost::shared_ptr<sobec::ActivationModelWeightedLog> activationZ =
      boost::make_shared<sobec::ActivationModelWeightedLog>(ZFootTrackingVec, 0.01);

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
  /*if (support == Support::LEFT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("Z_translation_RF",true);
  if (support == Support::RIGHT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("Z_translation_LF",true);*/
  if (support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("Z_translation_RF",true);
  if (support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("Z_translation_LF",true);
}

void ModelMakerNoThinking::definePostureTask(Cost &costCollector) {
  if (settings_.stateWeights.size() != designer_.get_rModel().nv * 2) {
    throw std::invalid_argument("State weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
          settings_.stateWeights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> postureModel =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activationWQ,
          boost::make_shared<crocoddyl::ResidualModelState>(
              state_, x0_, actuation_->get_nu()));

  costCollector.get()->addCost("postureTask", postureModel, settings_.wStateReg,
                               true);
}

void ModelMakerNoThinking::defineActuationTask(Cost &costCollector) {
  if (settings_.controlWeights.size() != (int)actuation_->get_nu()) {
    throw std::invalid_argument("Control weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
          settings_.controlWeights);  //.tail(actuation->get_nu())

  boost::shared_ptr<crocoddyl::CostModelAbstract> actuationModel =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activationWQ,
          boost::make_shared<crocoddyl::ResidualModelControl>(
              state_, actuation_->get_nu()));
  costCollector.get()->addCost("actuationTask", actuationModel,
                               settings_.wControlReg, true);
}

void ModelMakerNoThinking::defineCoPTask(Cost &costCollector, const Support &support) {
  Eigen::Vector2d w_cop;
  double value = 1.0 / (settings_.footSize * settings_.footSize);
  w_cop << value, value;

  // LEFT
  boost::shared_ptr<crocoddyl::CostModelAbstract> copCostLF =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_,
          boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(w_cop),
          boost::make_shared<ResidualModelCenterOfPressure>(
              state_, designer_.get_LF_id(), actuation_->get_nu()));

  // RIGHT
  boost::shared_ptr<crocoddyl::CostModelAbstract> copCostRF =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_,
          boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(w_cop),
          boost::make_shared<ResidualModelCenterOfPressure>(
              state_, designer_.get_RF_id(), actuation_->get_nu()));

  costCollector.get()->addCost(designer_.get_LF_name() + "_cop", copCostLF,
                         settings_.wCoP, false);
  costCollector.get()->addCost(designer_.get_RF_name() + "_cop", copCostRF,
                         settings_.wCoP, false);
  if (support == Support::LEFT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus(designer_.get_LF_name() + "_cop", true);

  if (support == Support::RIGHT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus(designer_.get_RF_name() + "_cop", true);
}

void ModelMakerNoThinking::defineFeetRotation(Cost &costCollector) {
  boost::shared_ptr<crocoddyl::ActivationModelQuadFlatLog> activationQF =
      boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(3, 0.01);
      
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
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF, 
                                                       residual_LF_Rotation);
  boost::shared_ptr<crocoddyl::CostModelAbstract> rotationModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF, 
                                                       residual_RF_Rotation);
  
  costCollector.get()->addCost("rotation_LF", rotationModel_LF,
                               settings_.wFootRot, true);
  costCollector.get()->addCost("rotation_RF", rotationModel_RF,
                               settings_.wFootRot, true);
}

void ModelMakerNoThinking::defineCoMTask(Cost &costCollector) {
  eVector3 comW;
  comW << 1, 1, 0;
  boost::shared_ptr<crocoddyl::ActivationModelWeightedLog> activation =
      boost::make_shared<crocoddyl::ActivationModelWeightedLog>(comW, 1);
  boost::shared_ptr<crocoddyl::CostModelAbstract> comCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activation, boost::make_shared<crocoddyl::ResidualModelCoMPosition>(
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

void ModelMakerNoThinking::defineVelFootTask(Cost &costCollector, const Support &support) {
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
  costCollector.get()->addCost("velFoot_RF", verticalFootVelCostRight, settings_.wVelFoot,false);
  costCollector.get()->addCost("velFoot_LF", verticalFootVelCostLeft, settings_.wVelFoot,false);
  if (support == Support::LEFT)
    costCollector.get()->changeCostStatus("velFoot_RF",true);
  if (support == Support::RIGHT)
    costCollector.get()->changeCostStatus("velFoot_LF",true);
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

void ModelMakerNoThinking::defineJointLimits(Cost &costCollector) {
  Eigen::VectorXd lower_bound(2 * state_->get_nv()),
      upper_bound(2 * state_->get_nv());

  double inf = 9999.0;
  lower_bound << Eigen::VectorXd::Constant(6, -inf),
      settings_.lowKinematicLimits,
      Eigen::VectorXd::Constant(state_->get_nv(), -inf); 

  upper_bound << Eigen::VectorXd::Constant(6, inf),
      settings_.highKinematicLimits,
      Eigen::VectorXd::Constant(state_->get_nv(), inf);

  crocoddyl::ActivationBounds bounds =
      crocoddyl::ActivationBounds(lower_bound, upper_bound, 1.);
  
  boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> activationQB =
      boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(bounds);
  boost::shared_ptr<crocoddyl::CostModelAbstract> jointLimitCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activationQB,
          boost::make_shared<crocoddyl::ResidualModelState>(
              state_, actuation_->get_nu()));

  costCollector.get()->addCost("jointLimits", jointLimitCost, settings_.wLimit,
                               true);
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

void ModelMakerNoThinking::define2DSurfaceTask(Cost &costCollector, const Support &support) {
  double yaw_left = atan2(designer_.get_LF_frame().rotation().data()[1], designer_.get_LF_frame().rotation().data()[0]);
  double yaw_right = atan2(designer_.get_RF_frame().rotation().data()[1], designer_.get_RF_frame().rotation().data()[0]);
  boost::shared_ptr<sobec::ResidualModel2DSurface> surfaceResidualLeft =
      boost::make_shared<sobec::ResidualModel2DSurface>(
              state_, designer_.get_LF_id(), designer_.get_RF_frame().translation().head(2),
              settings_.footMinimalDistance, yaw_right, settings_.angleSurface, actuation_->get_nu());
  boost::shared_ptr<sobec::ResidualModel2DSurface> surfaceResidualRight =
      boost::make_shared<sobec::ResidualModel2DSurface>(
              state_, designer_.get_RF_id(), designer_.get_LF_frame().translation().head(2),
              -settings_.footMinimalDistance, yaw_left, -settings_.angleSurface, actuation_->get_nu());

  double inf = 9999.0;

  crocoddyl::ActivationBounds boundsLeft =
      crocoddyl::ActivationBounds(Eigen::VectorXd::Constant(2, -inf), Eigen::VectorXd::Constant(2, 0), 1.);
  crocoddyl::ActivationBounds boundsRight =
      crocoddyl::ActivationBounds(Eigen::VectorXd::Constant(2, 0), Eigen::VectorXd::Constant(2, inf), 1.);
      
  boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> activationBRight =
      boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(boundsRight);
  
  boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> activationBLeft =
      boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(boundsLeft);
  
  boost::shared_ptr<crocoddyl::CostModelAbstract> surfaceCostRight = 
      boost::make_shared<crocoddyl::CostModelResidual>(
      state_, activationBRight, surfaceResidualRight);
  boost::shared_ptr<crocoddyl::CostModelAbstract> surfaceCostLeft = 
      boost::make_shared<crocoddyl::CostModelResidual>(
      state_, activationBLeft, surfaceResidualLeft);
  costCollector.get()->addCost("surface_RF", surfaceCostRight, settings_.wSurface,false);
  costCollector.get()->addCost("surface_LF", surfaceCostLeft, settings_.wSurface,false);
  if (support == Support::LEFT)
    costCollector.get()->changeCostStatus("surface_RF",true);
  if (support == Support::RIGHT)
    costCollector.get()->changeCostStatus("surface_LF",true);
}

AMA ModelMakerNoThinking::formulateStepTracker(const Support &support) {
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
  defineVelFootTask(costs, support);
  defineFeetRotation(costs);
  defineFlyHighTask(costs, support);
  define2DSurfaceTask(costs, support);
  defineZFeetTracking(costs, support);
  //defineGroundCollisionTask(costs);

  DAM runningDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMakerNoThinking::formulateTerminalStepTracker(const Support &support) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);

  defineCoMVelocity(costs);
  defineJointLimits(costs);
  definePostureTask(costs);
  defineFootCollisionTask(costs);
  defineVelFootTask(costs);
  defineFlyHighTask(costs, support);
  defineCoMTask(costs);
  defineZFeetTracking(costs);
  //defineGroundCollisionTask(costs);

  DAM terminalDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      terminalDAM, 0);

  return terminalModel;
}

std::vector<sobec::AMA> ModelMakerNoThinking::formulateHorizon(
    const std::vector<Support> &supports) {
  // for loop to generate a vector of IAMs
  std::vector<sobec::AMA> models;
  for (std::size_t i = 0; i < supports.size(); i++) {
	models.push_back(formulateStepTracker(supports[i]));
  }
  return models;
}

std::vector<sobec::AMA> ModelMakerNoThinking::formulateHorizon(const int &T) {
  std::vector<Support> supports(T, DOUBLE);
  return formulateHorizon(supports);
}

}  // namespace sobec

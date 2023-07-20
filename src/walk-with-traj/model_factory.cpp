#include "sobec/walk-with-traj/model_factory.hpp"

#include <crocoddyl/multibody/fwd.hpp>

#include "sobec/crocomplements/residual-cop.hpp"
#include "sobec/crocomplements/residual-power.hpp"
#include "sobec/crocomplements/residual-anticipated-state.hpp"
#include "sobec/walk-with-traj/designer.hpp"

namespace sobec {

ModelMaker::ModelMaker() {}

ModelMaker::ModelMaker(const ModelMakerSettings &settings, const RobotDesigner &designer) {
  initialize(settings, designer);
}

void ModelMaker::initialize(const ModelMakerSettings &settings, const RobotDesigner &designer) {
  settings_ = settings;
  designer_ = designer;

  state_ = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(designer_.get_rModel()));
  actuation_ = boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);

  x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
  x0_ << designer_.get_q0(), Eigen::VectorXd::Zero(designer_.get_rModel().nv);
  initialized_ = true;
}

void ModelMaker::defineFeetContact(Contact &contactCollector, const Support &support) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelLeft = boost::make_shared<crocoddyl::ContactModel6D>(
      state_, designer_.get_LF_id(), designer_.get_LF_frame(), actuation_->get_nu(), eVector2(0., 50.));

  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelRight = boost::make_shared<crocoddyl::ContactModel6D>(
      state_, designer_.get_RF_id(), designer_.get_RF_frame(), actuation_->get_nu(), eVector2(0., 50.));

  contactCollector->addContact(designer_.get_LF_name(), ContactModelLeft, false);
  contactCollector->addContact(designer_.get_RF_name(), ContactModelRight, false);

  if (support == Support::LEFT || support == Support::DOUBLE)
    contactCollector->changeContactStatus(designer_.get_LF_name(), true);

  if (support == Support::RIGHT || support == Support::DOUBLE)
    contactCollector->changeContactStatus(designer_.get_RF_name(), true);
}

void ModelMaker::defineFeetWrenchCost(Cost &costCollector, const Support &support) {
  double Mg = -designer_.getRobotMass() * settings_.gravity(2);
  double Fz_ref;
  support == Support::DOUBLE ? Fz_ref = Mg / 2 : Fz_ref = Mg;

  /*Eigen::Matrix3d coneRotationLeft =
      designer_.get_LF_frame().rotation().transpose();
  Eigen::Matrix3d coneRotationRight =
      designer_.get_RF_frame().rotation().transpose();*/

  crocoddyl::WrenchCone wrenchCone_LF = crocoddyl::WrenchCone(
      Eigen::Matrix3d::Identity(), settings_.mu, settings_.coneBox, 4, true, settings_.minNforce, settings_.maxNforce);
  crocoddyl::WrenchCone wrenchCone_RF = crocoddyl::WrenchCone(
      Eigen::Matrix3d::Identity(), settings_.mu, settings_.coneBox, 4, true, settings_.minNforce, settings_.maxNforce);

  eVector6 refWrench_LF = eVector6::Zero();
  eVector6 refWrench_RF = eVector6::Zero();
  if (support == Support::LEFT || support == Support::DOUBLE) refWrench_LF(2) = Fz_ref;
  if (support == Support::RIGHT || support == Support::DOUBLE) refWrench_RF(2) = Fz_ref;

  Eigen::VectorXd refCost_LF = wrenchCone_LF.get_A() * refWrench_LF;
  Eigen::VectorXd refCost_RF = wrenchCone_LF.get_A() * refWrench_RF;

  boost::shared_ptr<ActivationModelQuadRef> activation_LF_Wrench =
      boost::make_shared<ActivationModelQuadRef>(refCost_LF);
  boost::shared_ptr<ActivationModelQuadRef> activation_RF_Wrench =
      boost::make_shared<ActivationModelQuadRef>(refCost_RF);

  boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone> residual_LF_Wrench =
      boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(state_, designer_.get_LF_id(), wrenchCone_LF,
                                                                    actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activation_LF_Wrench, residual_LF_Wrench);

  boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone> residual_RF_Wrench =
      boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(state_, designer_.get_RF_id(), wrenchCone_RF,
                                                                    actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activation_RF_Wrench, residual_RF_Wrench);

  costCollector.get()->addCost("wrench_LF", wrenchModel_LF, settings_.wWrenchCone, false);
  costCollector.get()->addCost("wrench_RF", wrenchModel_RF, settings_.wWrenchCone, false);
  if (support == Support::LEFT || support == Support::DOUBLE) costCollector.get()->changeCostStatus("wrench_LF", true);
  if (support == Support::RIGHT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("wrench_RF", true);
}

void ModelMaker::defineFeetTracking(Cost &costCollector, const Support &support) {
  boost::shared_ptr<crocoddyl::ActivationModelQuadFlatLog> activationQF =
      boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(6, 0.01);

  boost::shared_ptr<crocoddyl::ResidualModelFramePlacement> residual_LF_Tracking =
      boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state_, designer_.get_LF_id(),
                                                                 designer_.get_LF_frame(), actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelFramePlacement> residual_RF_Tracking =
      boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state_, designer_.get_RF_id(),
                                                                 designer_.get_RF_frame(), actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF, residual_LF_Tracking);
  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF, residual_RF_Tracking);

  costCollector.get()->addCost("placement_LF", trackingModel_LF, settings_.wFootPlacement, false);
  costCollector.get()->addCost("placement_RF", trackingModel_RF, settings_.wFootPlacement, false);
  if (support == Support::LEFT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("placement_RF", true);
  if (support == Support::RIGHT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("placement_LF", true);
}

void ModelMaker::defineFeetTranslation(Cost &costCollector, const Support &support, const bool &stairs) {
  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation> residual_LF_Tracking =
      boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
          state_, designer_.get_LF_id(), designer_.get_LF_frame().translation(), actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation> residual_RF_Tracking =
      boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
          state_, designer_.get_RF_id(), designer_.get_RF_frame().translation(), actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ActivationModelAbstract> activationZ;
  if (stairs) {
    activationZ = boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(3, 0.01);
    //activationZ = boost::make_shared<sobec::ActivationModelSmooth1Norm>(3, 0.001);
    //activationZ = boost::make_shared<crocoddyl::ActivationModelQuad>(3);
  } else {
    eVector3 ZFootTrackingVec;
    ZFootTrackingVec << 0, 0, 1;
    activationZ = boost::make_shared<sobec::ActivationModelWeightedLog>(ZFootTrackingVec, 0.01);
  }

  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationZ, residual_LF_Tracking);
  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationZ, residual_RF_Tracking);

  costCollector.get()->addCost("translation_LF", trackingModel_LF, settings_.wFootPlacement, false);
  costCollector.get()->addCost("translation_RF", trackingModel_RF, settings_.wFootPlacement, false);

  if (stairs) {
    if (support == Support::LEFT) costCollector.get()->changeCostStatus("translation_RF", true);
    if (support == Support::RIGHT) costCollector.get()->changeCostStatus("translation_LF", true);
  } 
  if (support == Support::DOUBLE) {
    costCollector.get()->changeCostStatus("translation_RF", true);
    costCollector.get()->changeCostStatus("translation_LF", true);
  }
}

void ModelMaker::defineFeetForceTask(Cost &costCollector, const Support &support) {
  double Mg = -designer_.getRobotMass() * settings_.gravity(2);
  double Fz_ref;
  support == Support::DOUBLE ? Fz_ref = Mg / 2 : Fz_ref = Mg;

  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationForce =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(settings_.forceWeights);

  eVector6 refWrench_LF = eVector6::Zero();
  eVector6 refWrench_RF = eVector6::Zero();
  if (support == Support::LEFT || support == Support::DOUBLE) refWrench_LF(2) = Fz_ref;
  if (support == Support::RIGHT || support == Support::DOUBLE) refWrench_RF(2) = Fz_ref;

  pinocchio::Force refForce_LF = pinocchio::Force(refWrench_LF);
  pinocchio::Force refForce_RF = pinocchio::Force(refWrench_RF);

  boost::shared_ptr<crocoddyl::ResidualModelContactForce> residual_LF_Force =
      boost::make_shared<crocoddyl::ResidualModelContactForce>(state_, designer_.get_LF_id(), refForce_LF, 6,
                                                               actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelContactForce> residual_RF_Force =
      boost::make_shared<crocoddyl::ResidualModelContactForce>(state_, designer_.get_RF_id(), refForce_RF, 6,
                                                               actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationForce, residual_LF_Force);
  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationForce, residual_RF_Force);

  costCollector.get()->addCost("force_LF", forceModel_LF, settings_.wForceTask, false);
  costCollector.get()->addCost("force_RF", forceModel_RF, settings_.wForceTask, false);
  if (support == Support::RIGHT || support == Support::DOUBLE) costCollector.get()->changeCostStatus("force_RF", true);
  if (support == Support::LEFT || support == Support::DOUBLE) costCollector.get()->changeCostStatus("force_LF", true);
}

void ModelMaker::definePostureTask(Cost &costCollector) {
  if (settings_.stateWeights.size() != designer_.get_rModel().nv * 2) {
    throw std::invalid_argument("State weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(settings_.stateWeights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> postureModel = boost::make_shared<crocoddyl::CostModelResidual>(
      state_, activationWQ, boost::make_shared<crocoddyl::ResidualModelState>(state_, x0_, actuation_->get_nu()));

  costCollector.get()->addCost("postureTask", postureModel, settings_.wStateReg, true);
}

void ModelMaker::defineRotationBase(Cost &costCollector) {
  boost::shared_ptr<crocoddyl::ResidualModelFrameRotation> residual_Rotation =
      boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
          state_, designer_.get_root_id(), designer_.get_root_frame().rotation(), actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> rotationModel =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, residual_Rotation);

  costCollector.get()->addCost("rotation_base", rotationModel, settings_.wBaseRot, true);
}

void ModelMaker::defineTorqueLimits(Cost & costCollector) {
	Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(settings_.torqueLimits.size());
	crocoddyl::ActivationBounds bounds = crocoddyl::ActivationBounds(lower_bound, settings_.torqueLimits, 1.);
	
	boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> activationU =
      boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(bounds);
      
    boost::shared_ptr<crocoddyl::CostModelAbstract> torqueLimitsCost = boost::make_shared<crocoddyl::CostModelResidual>(
      state_, activationU, boost::make_shared<sobec::ResidualModelControl>(state_, actuation_->get_nu()));
  
    costCollector.get()->addCost("torqueLimits", torqueLimitsCost, settings_.wTauLimit, true);
}

void ModelMaker::defineActuationTask(Cost &costCollector) {
  if (settings_.controlWeights.size() != (int)actuation_->get_nu()) {
    throw std::invalid_argument("Control weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
          settings_.controlWeights);  //.tail(actuation->get_nu())

  boost::shared_ptr<crocoddyl::CostModelAbstract> actuationModel = boost::make_shared<crocoddyl::CostModelResidual>(
      state_, activationWQ, boost::make_shared<crocoddyl::ResidualModelControl>(state_, actuation_->get_nu()));
      //state_, activationWQ, boost::make_shared<sobec::ResidualModelPower>(state_, actuation_->get_nu()));
  costCollector.get()->addCost("actuationTask", actuationModel, settings_.wControlReg, true);
}

void ModelMaker::defineJointLimits(Cost &costCollector) {
  Eigen::VectorXd lower_bound(2 * state_->get_nv()), upper_bound(2 * state_->get_nv());

  double inf = 9999.0;
  //lower_bound << Eigen::VectorXd::Constant(6, -inf), settings_.lowKinematicLimits,
  //    Eigen::VectorXd::Constant(state_->get_nv(), -inf);
  //upper_bound << Eigen::VectorXd::Constant(6, inf), settings_.highKinematicLimits,
  //    Eigen::VectorXd::Constant(state_->get_nv(), inf);
  crocoddyl::ActivationBounds bounds = crocoddyl::ActivationBounds(settings_.lowKinematicLimits, settings_.highKinematicLimits, 1.);

  boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> activationQB =
      boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(bounds);
  //boost::shared_ptr<crocoddyl::CostModelAbstract> jointLimitCost = boost::make_shared<crocoddyl::CostModelResidual>(
  //    state_, activationQB, boost::make_shared<crocoddyl::ResidualModelState>(state_, actuation_->get_nu()));
  boost::shared_ptr<crocoddyl::CostModelAbstract> jointLimitCost = boost::make_shared<crocoddyl::CostModelResidual>(
      state_, activationQB, boost::make_shared<sobec::ResidualModelAnticipatedState>(state_, actuation_->get_nu(), 0.1));
  
  costCollector.get()->addCost("jointLimits", jointLimitCost, settings_.wLimit, true);
}

void ModelMaker::defineDCMTask(Cost &costCollector, const Support &support) {
  Eigen::Vector3d ref_position = Eigen::VectorXd::Zero(3);
  if (support == Support::LEFT) {
    ref_position = designer_.get_LF_frame().translation();
  }
  if (support == Support::RIGHT) {
    ref_position = designer_.get_RF_frame().translation();
  }
  if (support == Support::DOUBLE) {
    ref_position = (designer_.get_RF_frame().translation() + designer_.get_LF_frame().translation()) / 2.;
  }
  boost::shared_ptr<sobec::ResidualModelDCMPosition> residual_DCM =
      boost::make_shared<sobec::ResidualModelDCMPosition>(state_, ref_position, 1 / settings_.omega,
                                                          actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> DCM_model =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, residual_DCM);

  costCollector.get()->addCost("DCM", DCM_model, settings_.wDCM, true);
}

void ModelMaker::defineCoPTask(Cost &costCollector, const Support &support) {
  Eigen::Vector2d w_cop;
  double value = 1.0 / (settings_.footSize * settings_.footSize);
  w_cop << value, value;

  // LEFT
  boost::shared_ptr<crocoddyl::CostModelAbstract> copCostLF = boost::make_shared<crocoddyl::CostModelResidual>(
      state_, boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(w_cop),
      boost::make_shared<ResidualModelCenterOfPressure>(state_, designer_.get_LF_id(), actuation_->get_nu()));

  // RIGHT
  boost::shared_ptr<crocoddyl::CostModelAbstract> copCostRF = boost::make_shared<crocoddyl::CostModelResidual>(
      state_, boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(w_cop),
      boost::make_shared<ResidualModelCenterOfPressure>(state_, designer_.get_RF_id(), actuation_->get_nu()));

  costCollector.get()->addCost(designer_.get_LF_name() + "_cop", copCostLF, settings_.wCoP, false);
  costCollector.get()->addCost(designer_.get_RF_name() + "_cop", copCostRF, settings_.wCoP, false);
  if (support == Support::LEFT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus(designer_.get_LF_name() + "_cop", true);

  if (support == Support::RIGHT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus(designer_.get_RF_name() + "_cop", true);
}

void ModelMaker::defineFeetRotation(Cost &costCollector) {
  eVector3 FootRotationVec;
  FootRotationVec << 1, 1, 1;
  // boost::shared_ptr<sobec::ActivationModelWeightedLog> activationRot =
  //     boost::make_shared<sobec::ActivationModelWeightedLog>(FootRotationVec,
  //     0.01);
  boost::shared_ptr<sobec::ActivationModelQuadFlatLog> activationRot =
      boost::make_shared<sobec::ActivationModelQuadFlatLog>(3, 0.01);

  boost::shared_ptr<crocoddyl::ResidualModelFrameRotation> residual_LF_Rotation =
      boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
          state_, designer_.get_LF_id(), designer_.get_LF_frame().rotation(), actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelFrameRotation> residual_RF_Rotation =
      boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
          state_, designer_.get_RF_id(), designer_.get_RF_frame().rotation(), actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> rotationModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, residual_LF_Rotation);
  boost::shared_ptr<crocoddyl::CostModelAbstract> rotationModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, residual_RF_Rotation);

  costCollector.get()->addCost("rotation_LF", rotationModel_LF, settings_.wFootRot, true);
  costCollector.get()->addCost("rotation_RF", rotationModel_RF, settings_.wFootRot, true);
}

void ModelMaker::defineFeetZRotation(Cost &costCollector) {
  eVector3 FootRotationVec;
  FootRotationVec << 0, 0, 1;
  boost::shared_ptr<sobec::ActivationModelWeightedQuad> activationRot =
      boost::make_shared<sobec::ActivationModelWeightedQuad>(FootRotationVec);

  boost::shared_ptr<crocoddyl::ResidualModelFrameRotation> residual_LF_Rotation =
      boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
          state_, designer_.get_LF_id(), designer_.get_LF_frame().rotation(), actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelFrameRotation> residual_RF_Rotation =
      boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
          state_, designer_.get_RF_id(), designer_.get_RF_frame().rotation(), actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> rotationModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationRot, residual_LF_Rotation);
  boost::shared_ptr<crocoddyl::CostModelAbstract> rotationModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationRot, residual_RF_Rotation);

  costCollector.get()->addCost("rotation_LF", rotationModel_LF, settings_.wBaseRot, true);
  costCollector.get()->addCost("rotation_RF", rotationModel_RF, settings_.wBaseRot, true);
}

void ModelMaker::defineCoMPosition(Cost &costCollector) {
  boost::shared_ptr<sobec::ActivationModelSmooth1Norm> activation =
      boost::make_shared<sobec::ActivationModelSmooth1Norm>(3, 0.01);
  boost::shared_ptr<crocoddyl::CostModelAbstract> comCost = boost::make_shared<crocoddyl::CostModelResidual>(
      state_, activation,
      boost::make_shared<crocoddyl::ResidualModelCoMPosition>(state_, designer_.get_com_position(),
                                                              actuation_->get_nu()));
  costCollector.get()->addCost("comTask", comCost, settings_.wPCoM, true);
}

void ModelMaker::defineCoMVelocity(Cost &costCollector) {
  eVector3 refVelocity = eVector3::Zero();
  boost::shared_ptr<crocoddyl::CostModelAbstract> CoMVelocityCost = boost::make_shared<crocoddyl::CostModelResidual>(
      state_, boost::make_shared<ResidualModelCoMVelocity>(state_, refVelocity, actuation_->get_nu()));

  costCollector.get()->addCost("comVelocity", CoMVelocityCost, settings_.wVCoM, true);
}

void ModelMaker::defineFlyHighTask(Cost &costCollector, const Support &support) {
  boost::shared_ptr<ResidualModelFlyAngle> flyHighResidualRight = boost::make_shared<ResidualModelFlyAngle>(
      state_, designer_.get_RF_toe_id(), settings_.flyHighSlope / 2.0, settings_.height, settings_.dist,
      settings_.width, actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelResidual> flyHighCostRight =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, flyHighResidualRight);

  boost::shared_ptr<ResidualModelFlyAngle> flyHighResidualLeft = boost::make_shared<ResidualModelFlyAngle>(
      state_, designer_.get_LF_toe_id(), settings_.flyHighSlope / 2.0, settings_.height, settings_.dist,
      settings_.width, actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> flyHighCostLeft =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, flyHighResidualLeft);

  costCollector.get()->addCost("flyHigh_RF", flyHighCostRight, settings_.wFlyHigh, false);
  costCollector.get()->addCost("flyHigh_LF", flyHighCostLeft, settings_.wFlyHigh, false);
  if (support == Support::LEFT) costCollector.get()->changeCostStatus("flyHigh_RF", true);
  if (support == Support::RIGHT) costCollector.get()->changeCostStatus("flyHigh_LF", true);
}

void ModelMaker::defineVelFootTask(Cost &costCollector, const Support &support) {
  boost::shared_ptr<crocoddyl::ResidualModelFrameVelocity> verticalFootVelResidualLeft =
      boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(state_, designer_.get_LF_id(),
                                                                pinocchio::Motion::Zero(),
                                                                pinocchio::LOCAL_WORLD_ALIGNED, actuation_->get_nu());
  boost::shared_ptr<crocoddyl::ResidualModelFrameVelocity> verticalFootVelResidualRight =
      boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(state_, designer_.get_RF_id(),
                                                                pinocchio::Motion::Zero(),
                                                                pinocchio::LOCAL_WORLD_ALIGNED, actuation_->get_nu());
  eVector6 verticalFootVelActVec;
  verticalFootVelActVec << 1, 1, 1, 1, 1, 1;
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> verticalFootVelAct =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(verticalFootVelActVec);

  boost::shared_ptr<crocoddyl::CostModelAbstract> verticalFootVelCostRight =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, verticalFootVelAct, verticalFootVelResidualRight);
  boost::shared_ptr<crocoddyl::CostModelAbstract> verticalFootVelCostLeft =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, verticalFootVelAct, verticalFootVelResidualLeft);
  costCollector.get()->addCost("velFoot_RF", verticalFootVelCostRight, settings_.wVelFoot, false);
  costCollector.get()->addCost("velFoot_LF", verticalFootVelCostLeft, settings_.wVelFoot, false);
  if (support == Support::LEFT) costCollector.get()->changeCostStatus("velFoot_RF", true);
  if (support == Support::RIGHT) costCollector.get()->changeCostStatus("velFoot_LF", true);
}

void ModelMaker::defineFootCollisionTask(Cost &costCollector) {
  std::list<pinocchio::FrameIndex> leftIds = {designer_.get_LF_id(), designer_.get_LF_toe_id(),
                                              designer_.get_LF_heel_id()};
  std::list<pinocchio::FrameIndex> rightIds = {designer_.get_RF_id(), designer_.get_RF_toe_id(),
                                               designer_.get_RF_heel_id()};
  for (pinocchio::FrameIndex id1 : leftIds) {
    for (pinocchio::FrameIndex id2 : rightIds) {
      boost::shared_ptr<sobec::ResidualModelFeetCollision> feetColResidual =
          boost::make_shared<sobec::ResidualModelFeetCollision>(state_, id1, id2, actuation_->get_nu());
      Eigen::VectorXd feetColLow(1), feetColUp(1);
      feetColLow << settings_.footMinimalDistance;
      feetColUp << 1000;
      const crocoddyl::ActivationBounds feetColBounds = crocoddyl::ActivationBounds(feetColLow, feetColUp);
      boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> feetColAct =
          boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(feetColBounds);
      boost::shared_ptr<crocoddyl::CostModelAbstract> feetColCost =
          boost::make_shared<crocoddyl::CostModelResidual>(state_, feetColAct, feetColResidual);
      costCollector.get()->addCost(
          "feetcol_" + designer_.get_rModel().frames[id1].name + "_VS_" + designer_.get_rModel().frames[id2].name,
          feetColCost, settings_.wColFeet, true);
    }
  }
}

void ModelMaker::defineGripperCollisionTask(Cost &costCollector) {
  std::string link_name = "arm_left_4_joint";
  pinocchio::FrameIndex pin_link_id = designer_.get_rModel().getFrameId("arm_left_4_link");
  pinocchio::JointIndex pin_joint_id = designer_.get_rModel().getJointId(link_name);
  
  double RADIUS = 0.09;
  double LENGTH = 0.45;
  boost::shared_ptr<pinocchio::GeometryModel> geom_model = 
      boost::make_shared<pinocchio::GeometryModel>(pinocchio::GeometryModel());
  
  pinocchio::SE3 se3_pose_arm = pinocchio::SE3::Identity();
  se3_pose_arm.translation() = Eigen::Vector3d(-0.025,0.,-.225);
  
  pinocchio::GeomIndex ig_arm = geom_model->addGeometryObject(pinocchio::GeometryObject("simple_arm",
                                                              pin_link_id,
                                                              designer_.get_rModel().frames[pin_link_id].parent, 
                                                              std::shared_ptr<hpp::fcl::CollisionGeometry>(new hpp::fcl::Capsule(0, LENGTH)), 
                                                              se3_pose_arm));
  
  // Add obstacle in the world
  pinocchio::GeomIndex ig_obs;
  pinocchio::SE3 se3_pose_obst = pinocchio::SE3::Identity();
  se3_pose_obst.translation() = Eigen::Vector3d(0.6,0.3,1.);
  ig_obs = geom_model->addGeometryObject(pinocchio::GeometryObject("obstacle",
                                         designer_.get_rModel().getFrameId("universe"),
                                         designer_.get_rModel().frames[designer_.get_rModel().getFrameId("universe")].parent,
                                         //boost::shared_ptr <hpp::fcl::CollisionGeometry>(new hpp::fcl::Box(2*box_sizes[i])),
                                         std::shared_ptr<hpp::fcl::CollisionGeometry>(new hpp::fcl::Capsule(0,2)),
                                         se3_pose_obst)); 
  geom_model->addCollisionPair(pinocchio::CollisionPair(ig_arm,ig_obs));
  
  // Add collision cost
  double collision_radius = RADIUS + settings_.dist + 0.02;
  boost::shared_ptr<crocoddyl::ActivationModel2NormBarrier> actCollision = 
       boost::make_shared<crocoddyl::ActivationModel2NormBarrier>(3, collision_radius); // We add a threshold to activate BEFORE the collision
  boost::shared_ptr<crocoddyl::ResidualModelAbstract> residualModel = 
      boost::make_shared<crocoddyl::ResidualModelPairCollision>(state_, 
                                                                actuation_->get_nu(),
                                                                geom_model,
                                                                0, 
                                                                pin_joint_id);
  boost::shared_ptr<crocoddyl::CostModelAbstract> obstacleCost = 
      boost::make_shared<crocoddyl::CostModelResidual>(state_, actCollision, residualModel);
  
  costCollector.get()->addCost("gripper_collision", obstacleCost, settings_.wColFeet, true);
}  

void ModelMaker::defineGripperPlacement(Cost &costCollector) {
  pinocchio::SE3 goalPlacement = pinocchio::SE3::Identity();

  // Position
  boost::shared_ptr<crocoddyl::CostModelAbstract> gripperPositionCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(3, 0.1),
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
              state_, designer_.get_EndEff_id(), goalPlacement.translation(), actuation_->get_nu()));

  costCollector.get()->addCost("gripperPosition", gripperPositionCost, settings_.wGripperPos, true);

  // Orientation
  boost::shared_ptr<crocoddyl::CostModelAbstract> gripperRotationCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
                      state_, designer_.get_EndEff_id(), goalPlacement.rotation(), actuation_->get_nu()));

  costCollector.get()->addCost("gripperRotation", gripperRotationCost, settings_.wGripperRot, true);
}

void ModelMaker::defineGripperVelocity(Cost &costCollector) {
  pinocchio::Motion goalMotion = pinocchio::Motion(Eigen::VectorXd::Zero(6));
  boost::shared_ptr<crocoddyl::CostModelAbstract> gripperVelocityCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(
                      state_, designer_.get_EndEff_id(), goalMotion, pinocchio::WORLD, actuation_->get_nu()));

  costCollector.get()->addCost("gripperVelocity", gripperVelocityCost, settings_.wGripperVel, true);
}

AMA ModelMaker::formulateStepTracker(const Support &support) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);

  defineJointLimits(costs);
  defineCoMVelocity(costs);
  definePostureTask(costs);
  defineActuationTask(costs);
  defineFeetWrenchCost(costs, support);
  defineFeetForceTask(costs, support);
  defineCoPTask(costs, support);
  defineFeetTracking(costs, support);

  DAM runningDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state_, actuation_,
                                                                                            contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMaker::formulateTerminalStepTracker(const Support &support) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);
  defineJointLimits(costs);
  definePostureTask(costs);
  defineFeetTracking(costs, support);
  defineDCMTask(costs, support);

  DAM terminalDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
      state_, actuation_, contacts, costs, 0., true);
  AMA terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminalDAM, 0);

  return terminalModel;
}

AMA ModelMaker::formulateWWT(const Support &support, const bool &stairs) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);

  defineCoMVelocity(costs);
  defineJointLimits(costs);
  definePostureTask(costs);
  defineTorqueLimits(costs);
  defineActuationTask(costs);
  defineFeetWrenchCost(costs, support);
  defineFootCollisionTask(costs);
  defineCoPTask(costs, support);
  defineVelFootTask(costs, support);
  defineFeetRotation(costs);
  defineFlyHighTask(costs, support);
  defineFeetTranslation(costs, support, stairs);

  DAM runningDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state_, actuation_,
                                                                                            contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMaker::formulateTerminalWWT(const Support &support, const bool &stairs) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);

  defineCoMVelocity(costs);
  defineJointLimits(costs);
  definePostureTask(costs);
  defineFootCollisionTask(costs);
  defineVelFootTask(costs);
  defineFlyHighTask(costs, support);
  defineCoMPosition(costs);
  defineFeetTranslation(costs, support, stairs);
  defineDCMTask(costs, support);
  defineRotationBase(costs);

  DAM terminalDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
      state_, actuation_, contacts, costs, 0., true);
  AMA terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminalDAM, 0);

  return terminalModel;
}

AMA ModelMaker::formulatePointingTask() {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());
  defineFeetContact(contacts, Support::DOUBLE);

  defineFeetWrenchCost(costs, Support::DOUBLE);
  definePostureTask(costs);
  defineActuationTask(costs);
  defineJointLimits(costs);
  defineCoMPosition(costs);
  defineCoMVelocity(costs);
  defineGripperPlacement(costs);
  defineGripperVelocity(costs);

  DAM runningDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state_, actuation_,
                                                                                            contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMaker::formulateColFullTask() {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());
  defineFeetContact(contacts, Support::DOUBLE);

  definePostureTask(costs);
  defineActuationTask(costs);
  defineJointLimits(costs);
  defineCoMPosition(costs);
  defineFeetWrenchCost(costs, Support::DOUBLE);
  defineGripperPlacement(costs);
  defineGripperVelocity(costs);
  defineGripperCollisionTask(costs);

  DAM runningDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state_, actuation_,
                                                                                            contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMaker::formulateTerminalColFullTask() {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());
  defineFeetContact(contacts, Support::DOUBLE);

  definePostureTask(costs);
  defineJointLimits(costs);
  defineCoMPosition(costs);
  defineGripperPlacement(costs);
  defineGripperVelocity(costs);
  defineGripperCollisionTask(costs);

  DAM terminalDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state_, actuation_,
                                                                                            contacts, costs, 0., true);
  AMA terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminalDAM, 0);

  return terminalModel;
}

std::vector<AMA> ModelMaker::formulateHorizon(const std::vector<Support> &supports, const Experiment &experiment) {
  // for loop to generate a vector of IAMs
  std::vector<AMA> models;
  if (experiment == Experiment::WALK) {
    for (std::size_t i = 0; i < supports.size(); i++) {
      models.push_back(formulateStepTracker(supports[i]));
    }
  } else if (experiment == Experiment::WWT) {
    for (std::size_t i = 0; i < supports.size(); i++) {
      models.push_back(formulateWWT(supports[i], false));
    }
  } else if (experiment == Experiment::WWT_STAIRS) {
    for (std::size_t i = 0; i < supports.size(); i++) {
      models.push_back(formulateWWT(supports[i], true));
    }
  }

  return models;
}

std::vector<AMA> ModelMaker::formulateHorizon(const int &T) {
  std::vector<Support> supports(T, DOUBLE);
  return formulateHorizon(supports, Experiment::WALK);
}

}  // namespace sobec

#include "sobec/hand_table/model_factory_hand.hpp"

#include <crocoddyl/multibody/fwd.hpp>

#include "sobec/crocomplements/residual-cop.hpp"
#include "sobec/walk-with-traj/designer.hpp"

namespace sobec {

ModelMakerHand::ModelMakerHand() {}

ModelMakerHand::ModelMakerHand(const ModelMakerHandSettings &settings,
                               const RobotDesigner &designer) {
  initialize(settings, designer);
}

void ModelMakerHand::initialize(const ModelMakerHandSettings &settings,
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

void ModelMakerHand::defineFeetContact(Contact &contactCollector) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelLeft =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_LF_id(), designer_.get_LF_frame(),
          actuation_->get_nu(), eVector2(0., 50.));

  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelRight =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_RF_id(), designer_.get_RF_frame(),
          actuation_->get_nu(), eVector2(0., 50.));

  contactCollector->addContact(designer_.get_LF_name(), ContactModelLeft,
                               true);
  contactCollector->addContact(designer_.get_RF_name(), ContactModelRight,
                               true);
}

void ModelMakerHand::defineHandContact(Contact &contactCollector,
                                   const Phase &phase) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelHandRight =
      boost::make_shared<crocoddyl::ContactModel3D>(
          state_, designer_.get_RH_id(), designer_.get_RH_frame().translation(),
          actuation_->get_nu(), eVector2(0., 4.));
          
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelHandLeft =
      boost::make_shared<crocoddyl::ContactModel3D>(
          state_, designer_.get_LH_id(), designer_.get_LH_frame().translation(),
          actuation_->get_nu(), eVector2(0., 4.));
          
  contactCollector->addContact(designer_.get_RH_name(), ContactModelHandRight,
                               false);
  contactCollector->addContact(designer_.get_LH_name(), ContactModelHandLeft,
                               false);
                               
  if (phase == Phase::CONTACT_RIGHT)
    contactCollector->changeContactStatus(designer_.get_RH_name(), true);
  if (phase == Phase::CONTACT_LEFT)
    contactCollector->changeContactStatus(designer_.get_LH_name(), true);
}

void ModelMakerHand::defineFeetWrenchCost(Cost &costCollector) {
  double Mg = -designer_.getRobotMass() * settings_.gravity(2);
  double Fz_ref = Mg / 2;

  crocoddyl::WrenchCone wrenchCone_LF =
      crocoddyl::WrenchCone(Eigen::Matrix3d::Identity(), settings_.mu, settings_.coneBox,
                            4, true, settings_.minNforce, settings_.maxNforce);
  crocoddyl::WrenchCone wrenchCone_RF =
      crocoddyl::WrenchCone(Eigen::Matrix3d::Identity(), settings_.mu, settings_.coneBox,
                            4, true, settings_.minNforce, settings_.maxNforce);

  eVector6 refWrench_LF = eVector6::Zero();
  eVector6 refWrench_RF = eVector6::Zero();
  refWrench_LF(2) = Fz_ref;
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
                               settings_.wWrenchCone, true);
  costCollector.get()->addCost("wrench_RF", wrenchModel_RF,
                               settings_.wWrenchCone, true);
}

void ModelMakerHand::defineCoMPosition(Cost &costCollector) {
  boost::shared_ptr<crocoddyl::CostModelAbstract> comCost = boost::make_shared<crocoddyl::CostModelResidual>(
      state_, 
      boost::make_shared<crocoddyl::ResidualModelCoMPosition>(state_, designer_.get_com_position(),
                                                              actuation_->get_nu()));
  costCollector.get()->addCost("comTask", comCost, settings_.wCoM, true);
}

void ModelMakerHand::defineHandForceTask(Cost &costCollector, const Phase &phase) {
  Eigen::VectorXd pin_force(6);
  pin_force << 0, 0, 50, 0, 0, 0;
  pinocchio::Force refForce = pinocchio::Force(pin_force);
  boost::shared_ptr<crocoddyl::ResidualModelContactForce> forceResidualModelRight = 
      boost::make_shared<crocoddyl::ResidualModelContactForce>(state_,
            designer_.get_RH_id(),designer_.get_RH_frame().actInv(refForce),3,actuation_->get_nu());
  boost::shared_ptr<crocoddyl::ResidualModelContactForce> forceResidualModelLeft = 
      boost::make_shared<crocoddyl::ResidualModelContactForce>(state_,
            designer_.get_LH_id(),designer_.get_LH_frame().actInv(refForce),3,actuation_->get_nu());
  
  
  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_RH = 
            boost::make_shared<crocoddyl::CostModelResidual>(state_, forceResidualModelRight);
  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_LH = 
            boost::make_shared<crocoddyl::CostModelResidual>(state_, forceResidualModelLeft);
  
  costCollector.get()->addCost("force_LH", forceModel_LH,
                               settings_.wForceHand, false);
  costCollector.get()->addCost("force_RH", forceModel_RH,
                               settings_.wForceHand, false);
  if (phase == Phase::CONTACT_LEFT)
    costCollector.get()->changeCostStatus("force_LH",true);
  if (phase == Phase::CONTACT_RIGHT)
    costCollector.get()->changeCostStatus("force_RH",true);
}

void ModelMakerHand::definePostureTask(Cost &costCollector) {
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

void ModelMakerHand::defineActuationTask(Cost &costCollector) {
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

void ModelMakerHand::defineJointLimits(Cost &costCollector) {
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

void ModelMakerHand::defineDCMTask(Cost &costCollector, const Phase &phase) {
	
  Eigen::Vector3d ref_position = Eigen::VectorXd::Zero(3);
  ref_position = (designer_.get_RF_frame().translation() + 
	              designer_.get_LF_frame().translation()) / 2.;
  
  if (phase == Phase::CONTACT_LEFT) {
	  ref_position = (ref_position + designer_.get_LH_frame().translation()) / 2;
  }
  if (phase == Phase::CONTACT_RIGHT) {
	  ref_position = (ref_position + designer_.get_RH_frame().translation()) / 2;
  }
  boost::shared_ptr<sobec::ResidualModelDCMPosition>
      residual_DCM =
          boost::make_shared<sobec::ResidualModelDCMPosition>(
              state_, ref_position, 1 / settings_.omega,
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> DCM_model =
      boost::make_shared<crocoddyl::CostModelResidual>(state_,
                                                       residual_DCM);
  
  costCollector.get()->addCost("DCM", DCM_model,
                               settings_.wDCM, true);
}

void ModelMakerHand::defineHandTranslation(Cost &costCollector, const Phase &phase) {
  boost::shared_ptr<crocoddyl::ActivationModelQuadFlatLog> activationQF =
      boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(3, 0.1);

  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation>
      residualRH =
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
              state_, designer_.get_RH_id(), designer_.get_RH_frame().translation(),
              actuation_->get_nu());
  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation>
      residualLH =
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
              state_, designer_.get_LH_id(), designer_.get_LH_frame().translation(),
              actuation_->get_nu());
  
  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModelRH =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF,
                                                       residualRH);
  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModelLH =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF,
                                                       residualLH);
                                                       
  costCollector.get()->addCost("position_RH", trackingModelRH,
                               settings_.wHandTranslation, false);
  costCollector.get()->addCost("position_LH", trackingModelLH,
                               settings_.wHandTranslation, false);
  if (phase == Phase::CONTACT_RIGHT || phase == Phase::TRACKING_RIGHT)
    costCollector.get()->changeCostStatus("position_RH", true);
  if (phase == Phase::CONTACT_LEFT || phase == Phase::TRACKING_LEFT)
    costCollector.get()->changeCostStatus("position_LH", true);
}

void ModelMakerHand::defineHandCollisionTask(Cost &costCollector) {
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
  se3_pose_obst.translation() = settings_.obstaclePosition;
  ig_obs = geom_model->addGeometryObject(pinocchio::GeometryObject("obstacle",
                                         designer_.get_rModel().getFrameId("universe"),
                                         designer_.get_rModel().frames[designer_.get_rModel().getFrameId("universe")].parent,
                                         //boost::shared_ptr <hpp::fcl::CollisionGeometry>(new hpp::fcl::Box(2*box_sizes[i])),
                                         std::shared_ptr<hpp::fcl::CollisionGeometry>(new hpp::fcl::Capsule(0,settings_.obstacleHeight)),
                                         se3_pose_obst)); 
  geom_model->addCollisionPair(pinocchio::CollisionPair(ig_arm,ig_obs));
  
  // Add collision cost
  double collision_radius = RADIUS + settings_.obstacleRadius;
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
  
  costCollector.get()->addCost("hand_collision", obstacleCost, settings_.wHandCollision, true);
}  

void ModelMakerHand::defineHandRotation(Cost &costCollector) {
  pinocchio::SE3 goalPlacement = pinocchio::SE3::Identity();

  // Orientation
  boost::shared_ptr<crocoddyl::CostModelAbstract> handRotationCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
                      state_, designer_.get_LH_id(), goalPlacement.rotation(), actuation_->get_nu()));

  costCollector.get()->addCost("handRotation", handRotationCost, settings_.wHandRotation, true);
}

void ModelMakerHand::defineHandVelocity(Cost &costCollector) {
  pinocchio::Motion goalMotion = pinocchio::Motion(Eigen::VectorXd::Zero(6));
  boost::shared_ptr<crocoddyl::CostModelAbstract> handLeftVelocityCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(
                      state_, designer_.get_LH_id(), goalMotion, pinocchio::WORLD, actuation_->get_nu()));
  
  boost::shared_ptr<crocoddyl::CostModelAbstract> handRightVelocityCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(
                      state_, designer_.get_RH_id(), goalMotion, pinocchio::WORLD, actuation_->get_nu()));
                      
  costCollector.get()->addCost("handLeftVelocity", handLeftVelocityCost, settings_.wHandVelocity, true);
  costCollector.get()->addCost("handRightVelocity", handRightVelocityCost, settings_.wHandVelocity, true);
}

AMA ModelMakerHand::formulateHandTracker(const Phase &phase) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts);
  defineHandContact(contacts, phase);

  defineJointLimits(costs);
  definePostureTask(costs);
  defineActuationTask(costs);
  defineHandTranslation(costs, phase);
  defineHandVelocity(costs);
  defineHandForceTask(costs,phase);
  defineFeetWrenchCost(costs); 
  defineCoMPosition(costs);
  defineHandCollisionTask(costs);

  DAM runningDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMakerHand::formulateTerminalHandTracker(const Phase &phase) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts);

  defineJointLimits(costs);
  definePostureTask(costs);
  defineCoMPosition(costs);

  DAM terminalDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      terminalDAM, 0);

  return terminalModel;
}

AMA ModelMakerHand::formulatePointingTask() {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());
  defineFeetContact(contacts);

  defineFeetWrenchCost(costs);
  definePostureTask(costs);
  defineActuationTask(costs);
  defineJointLimits(costs);
  defineCoMPosition(costs);
  defineHandTranslation(costs);
  defineHandVelocity(costs);

  DAM runningDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state_, actuation_,
                                                                                            contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMakerHand::formulateColFullTask() {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());
  defineFeetContact(contacts);

  definePostureTask(costs);
  defineActuationTask(costs);
  defineJointLimits(costs);
  defineCoMPosition(costs);
  defineFeetWrenchCost(costs);
  defineHandTranslation(costs);
  defineHandVelocity(costs);
  defineHandCollisionTask(costs);

  DAM runningDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state_, actuation_,
                                                                                            contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMakerHand::formulateTerminalColFullTask() {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());
  defineFeetContact(contacts);

  definePostureTask(costs);
  defineJointLimits(costs);
  defineCoMPosition(costs);
  defineHandTranslation(costs);
  defineHandVelocity(costs);
  defineHandCollisionTask(costs);

  DAM terminalDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state_, actuation_,
                                                                                            contacts, costs, 0., true);
  AMA terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(terminalDAM, 0);

  return terminalModel;
}

std::vector<AMA> ModelMakerHand::formulateHorizon(
    const std::vector<Phase> &phases) {
  // for loop to generate a vector of IAMs
  std::vector<AMA> models;
  for (std::size_t i = 0; i < phases.size(); i++) {
	models.push_back(formulateHandTracker(phases[i]));
  }

  return models;
}

std::vector<AMA> ModelMakerHand::formulateHorizon(const int &T) {
  std::vector<Phase> phases(T, NO_HAND);
  return formulateHorizon(phases);
}

}  // namespace sobec

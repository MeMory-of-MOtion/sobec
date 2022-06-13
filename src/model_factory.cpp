#include "sobec/model_factory.hpp"
#include "sobec/designer.hpp"
#include <crocoddyl/multibody/fwd.hpp>

namespace sobec {

ModelMaker::ModelMaker(){}

ModelMaker::ModelMaker(const ModelMakerSettings &settings, const RobotDesigner &designer){ 
    initialize(settings, designer); 
    }

void ModelMaker::initialize(const ModelMakerSettings &settings, const RobotDesigner &designer){ 
    settings_ = settings;
    designer_ = designer;

    state_ = boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(designer_.get_rModel()));
    actuation_ = boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);

    x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
    x0_ << designer_.get_q0(), Eigen::VectorXd::Zero(designer_.get_rModel().nv);
}

void ModelMaker::defineFeetContact(Contact &contactCollector, const Support &support){
	
	crocoddyl::FramePlacement xrefLeft(designer_.get_LF_id(), designer_.get_LF_frame());
	boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelLeft = 
		boost::make_shared<crocoddyl::ContactModel6D>(state_, xrefLeft, actuation_->get_nu(), eVector2(0., 50.));

	crocoddyl::FramePlacement xrefRight(designer_.get_RF_id(), designer_.get_RF_frame());
	boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelRight =
		boost::make_shared<crocoddyl::ContactModel6D>(state_, xrefRight, actuation_->get_nu(), eVector2(0., 50.));
	
	contactCollector->addContact(designer_.get_LF_name(), ContactModelLeft, false);
	contactCollector->addContact(designer_.get_RF_name(), ContactModelRight, false); 

	if(support == Support::LEFT || support == Support::DOUBLE)
		contactCollector->changeContactStatus(designer_.get_LF_name(), true);
	
	if(support == Support::RIGHT || support == Support::DOUBLE)
		contactCollector->changeContactStatus(designer_.get_RF_name(), true);
}

void ModelMaker::defineFeetWrenchCost(Cost &costCollector, const Support &support){

	double Mg = -designer_.getRobotMass() * settings_.gravity(2);
	double Fz_ref;
	support == Support::DOUBLE? Fz_ref = Mg/2 : Fz_ref = Mg;

	Eigen::Matrix3d coneRotationLeft = designer_.get_LF_frame().rotation().transpose();
	Eigen::Matrix3d coneRotationRight = designer_.get_RF_frame().rotation().transpose();

	crocoddyl::WrenchCone wrenchCone_LF = crocoddyl::WrenchCone(coneRotationLeft, settings_.mu, 
	    settings_.coneBox, 4, true, settings_.minNforce, settings_.maxNforce); 
	crocoddyl::WrenchCone wrenchCone_RF = crocoddyl::WrenchCone(coneRotationRight, settings_.mu, 
	    settings_.coneBox, 4, true, settings_.minNforce, settings_.maxNforce);
	
	eVector6 refWrench_LF = eVector6::Zero();
	eVector6 refWrench_RF = eVector6::Zero();
	if(support == Support::LEFT || support == Support::DOUBLE) 
		refWrench_LF(2) = Fz_ref;
	if(support == Support::RIGHT || support == Support::DOUBLE) 
		refWrench_RF(2) = Fz_ref;

	Eigen::VectorXd refCost_LF = wrenchCone_LF.get_A() * refWrench_LF;
	Eigen::VectorXd refCost_RF = wrenchCone_LF.get_A() * refWrench_RF;

	boost::shared_ptr<ActivationModelQuadRef> activation_LF_Wrench = boost::make_shared<ActivationModelQuadRef>(refCost_LF);
	boost::shared_ptr<ActivationModelQuadRef> activation_RF_Wrench = boost::make_shared<ActivationModelQuadRef>(refCost_RF);

	boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone> residual_LF_Wrench = 
		boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(state_, designer_.get_LF_id(), wrenchCone_LF, actuation_->get_nu());
	boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_LF = 
		boost::make_shared<crocoddyl::CostModelResidual>(state_, activation_LF_Wrench, residual_LF_Wrench);

	boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone> residual_RF_Wrench = 
		boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(state_, designer_.get_RF_id(), wrenchCone_RF, actuation_->get_nu());
	boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_RF = 
		boost::make_shared<crocoddyl::CostModelResidual>(state_, activation_RF_Wrench, residual_RF_Wrench);
	
	costCollector.get()->addCost("wrench_LF", wrenchModel_LF, settings_.wWrenchCone, true);
	costCollector.get()->addCost("wrench_RF", wrenchModel_RF, settings_.wWrenchCone, true);
}

void ModelMaker::defineFeetTracking(Cost &costCollector){

	boost::shared_ptr<crocoddyl::ActivationModelQuadFlatLog> activationQF = 
		   boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(6,0.01);
	
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

	costCollector.get()->addCost("placement_LF", trackingModel_LF, settings_.wFootTrans, true);
	costCollector.get()->addCost("placement_RF", trackingModel_RF, settings_.wFootTrans, true);
}

void ModelMaker::definePostureTask(Cost &costCollector){
    if (settings_.stateWeights.size() != designer_.get_rModel().nv * 2) {
		throw std::invalid_argument("State weight size is wrong ");
	}
	boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ = 
		 boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(settings_.stateWeights);

	boost::shared_ptr<crocoddyl::CostModelAbstract> postureModel =
		boost::make_shared<crocoddyl::CostModelResidual>(state_, activationWQ,
		boost::make_shared<crocoddyl::ResidualModelState>(state_, x0_, actuation_->get_nu())); 
		
	costCollector.get()->addCost("postureTask", postureModel, settings_.wStateReg, true);
}

void ModelMaker::defineActuationTask(Cost &costCollector){
	if (settings_.controlWeights.size() != actuation_->get_nu()) {
		throw std::invalid_argument("Control weight size is wrong ");
	}
	boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ = 
		 boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(settings_.controlWeights); //.tail(actuation->get_nu())
	
	boost::shared_ptr<crocoddyl::CostModelAbstract>  actuationModel =
	    boost::make_shared<crocoddyl::CostModelResidual>(state_, activationWQ, 
		boost::make_shared<crocoddyl::ResidualModelControl>(state_,actuation_->get_nu()));
	costCollector.get()->addCost("actuationTask", actuationModel, settings_.wControlReg,true);
}

void ModelMaker::defineJointLimits(Cost &costCollector){
	
	Eigen::VectorXd lower_bound(2*state_->get_nv()), upper_bound(2*state_->get_nv());
	double inf = 9999.0;
	lower_bound << Eigen::VectorXd::Constant(6, -inf), 
				   designer_.get_rModel().lowerPositionLimit.tail(state_->get_nq() - 7),
				   Eigen::VectorXd::Constant(state_->get_nv(), -inf);
	
	upper_bound << Eigen::VectorXd::Constant(6, inf), 
				   designer_.get_rModel().upperPositionLimit.tail(state_->get_nq() - 7),
				   Eigen::VectorXd::Constant(state_->get_nv(), +inf);
	
	crocoddyl::ActivationBounds bounds = crocoddyl::ActivationBounds(lower_bound,upper_bound, 1.);
	
	boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> activationQB = 
		 boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(bounds);
	boost::shared_ptr<crocoddyl::CostModelAbstract> jointLimitCost =
		boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQB,
		boost::make_shared<crocoddyl::ResidualModelState>(state_, actuation_->get_nu()));
	
	costCollector.get()->addCost("jointLimits", jointLimitCost, settings_.wLimit, true);
}

void ModelMaker::defineCoMVelocity(Cost &costCollector){

	eVector3 refVelocity = eVector3::Zero();
	boost::shared_ptr<crocoddyl::CostModelAbstract> CoMVelocityCost =
		boost::make_shared<crocoddyl::CostModelResidual>(state_, 
		boost::make_shared<ResidualModelCoMVelocity>(state_, refVelocity, actuation_->get_nu())); 
		
	costCollector.get()->addCost("comVelocity", CoMVelocityCost, settings_.wVCoM, true);
}

AMA ModelMaker::formulateStepTracker(const Support &support){
	
	Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
	Cost costs = boost::make_shared<crocoddyl::CostModelSum>(state_,actuation_->get_nu());
	
	defineFeetContact(contacts, support);

	defineCoMVelocity(costs);
	defineJointLimits(costs);
	definePostureTask(costs);
	defineActuationTask(costs);
	defineFeetWrenchCost(costs, support);
	defineFeetTracking(costs);

	DAM runningDAM = boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>
		(state_, actuation_, contacts, costs, 0.,true);
	AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>
		(runningDAM, settings_.timeStep);  
		
	const boost::shared_ptr<crocoddyl::ActionDataAbstract> temp_data = runningModel->createData();
	Eigen::VectorXd uref = Eigen::VectorXd::Zero(Eigen::Index(actuation_->get_nu()));
	runningModel->quasiStatic(temp_data,uref,x0_);
	
	boost::shared_ptr<crocoddyl::ResidualModelControl> uResidualControl = 
	    boost::static_pointer_cast<crocoddyl::ResidualModelControl>(runningDAM->get_costs()->get_costs().at("actuationTask")->cost->get_residual());
	uResidualControl->set_reference(uref);

	return runningModel;
}

// AMA ModelMaker::formulate_flat_walker(const Support &support){
    
//     // Create contact model
// 	boost::shared_ptr<crocoddyl::ContactModelMultiple> contactModel =
// 		  boost::make_shared<crocoddyl::ContactModelMultiple>(state_, actuation_->get_nu());
	
// 	const pinocchio::SE3& contactFramePosLeft = designer_.get_rData().oMf[designer_.get_LF_id()];
// 	crocoddyl::FramePlacement xrefLeft(designer_.get_LF_id(), contactFramePosLeft);
// 	boost::shared_ptr<crocoddyl::ContactModelAbstract> singleContactModelLeft =
// 		boost::make_shared<crocoddyl::ContactModel6D>(state_, xrefLeft, actuation_->get_nu(), Eigen::Vector2d(0., 50.));
	
// 	const pinocchio::SE3& contactFramePosRight = designer_.get_rData().oMf[designer_.get_RF_id()];
// 	crocoddyl::FramePlacement xrefRight(designer_.get_RF_id(), contactFramePosRight);
// 	boost::shared_ptr<crocoddyl::ContactModelAbstract> singleContactModelRight =
// 		boost::make_shared<crocoddyl::ContactModel6D>(state_, xrefRight, actuation_->get_nu(), Eigen::Vector2d(0., 50.));
	
// 	switch(support)
// 	{
// 		case LEFT  : contactModel->addContact("leftSoleContact", singleContactModelLeft,true);
// 		             contactModel->addContact("rightSoleContact", singleContactModelRight,false);   
// 		             break;
// 		case RIGHT: contactModel->addContact("leftSoleContact", singleContactModelLeft,false);
// 		            contactModel->addContact("rightSoleContact", singleContactModelRight,true);   
// 		            break;
// 		case DOUBLE : contactModel->addContact("leftSoleContact", singleContactModelLeft,true);
// 		              contactModel->addContact("rightSoleContact", singleContactModelRight,true);   
// 		              break;
//     }

// 	//Create Costs
// 	boost::shared_ptr<crocoddyl::CostModelSum> runningCostModel = 
// 		boost::make_shared<crocoddyl::CostModelSum>(state_,actuation_->get_nu());

// 	// Wrench cone Cost  
// 	Eigen::Matrix3d coneRotationLeft = designer_.get_rData().oMf[designer_.get_LF_id()].rotation().transpose();
// 	Eigen::Matrix3d coneRotationRight = designer_.get_rData().oMf[designer_.get_RF_id()].rotation().transpose();
// 	crocoddyl::WrenchCone wrenchConeLeft = crocoddyl::WrenchCone(coneRotationLeft, settings_.mu, 
// 	    settings_.coneBox,4,true,settings_.minNforce,settings_.maxNforce); 
// 	crocoddyl::WrenchCone wrenchConeRight = crocoddyl::WrenchCone(coneRotationRight, settings_.mu, 
// 	    settings_.coneBox,4,true,settings_.minNforce,settings_.maxNforce);
	
// 	double fzRef1Contact = -designer_.getRobotMass() * settings_.gravity(2);
// 	double fzRef2Contact = fzRef1Contact/2;

// 	Eigen::VectorXd wrenchReference2Contact(6);
// 	wrenchReference2Contact << 0,0, fzRef2Contact,0,0,0;
// 	Eigen::VectorXd wrenchReference1Contact(6);
// 	wrenchReference1Contact << 0,0, fzRef1Contact,0,0,0;
	
// 	Eigen::VectorXd AwrenchRefRight2Contact = wrenchConeRight.get_A() * wrenchReference2Contact;
// 	Eigen::VectorXd AwrenchRefLeft2Contact = wrenchConeLeft.get_A() * wrenchReference2Contact;
// 	Eigen::VectorXd AwrenchRefRight1Contact = wrenchConeRight.get_A() * wrenchReference1Contact;
// 	Eigen::VectorXd AwrenchRefLeft1Contact = wrenchConeLeft.get_A() * wrenchReference1Contact;
	
// 	boost::shared_ptr<ActivationModelQuadRef> activationWrenchLeft2Contact = 
// 		boost::make_shared<ActivationModelQuadRef>(AwrenchRefLeft2Contact);
// 	boost::shared_ptr<ActivationModelQuadRef> activationWrenchRight2Contact = 
// 		boost::make_shared<ActivationModelQuadRef>(AwrenchRefRight2Contact);
	
// 	boost::shared_ptr<ActivationModelQuadRef> activationWrenchLeft1Contact = 
// 		boost::make_shared<ActivationModelQuadRef>(AwrenchRefLeft1Contact);
// 	boost::shared_ptr<ActivationModelQuadRef> activationWrenchRight1Contact = 
// 		boost::make_shared<ActivationModelQuadRef>(AwrenchRefRight1Contact);
	
// 	boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone> wrenchResidualLeft2Contact = 
// 		boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(state_,designer_.get_LF_id(),wrenchConeLeft,actuation_->get_nu());
// 	boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchConeCostLeft2Contact = 
// 		boost::make_shared<crocoddyl::CostModelResidual>(state_,activationWrenchLeft2Contact,wrenchResidualLeft2Contact);
	
// 	boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone> wrenchResidualRight2Contact = 
// 		boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(state_,designer_.get_RF_id(),wrenchConeRight,actuation_->get_nu());
// 	boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchConeCostRight2Contact = 
// 		boost::make_shared<crocoddyl::CostModelResidual>(state_,activationWrenchRight2Contact,wrenchResidualRight2Contact);
		
// 	boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone> wrenchResidualLeft1Contact = 
// 		boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(state_,designer_.get_LF_id(),wrenchConeLeft,actuation_->get_nu());
// 	boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchConeCostLeft1Contact = 
// 		boost::make_shared<crocoddyl::CostModelResidual>(state_,activationWrenchLeft1Contact,wrenchResidualLeft1Contact);
	
// 	boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone> wrenchResidualRight1Contact = 
// 		boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(state_,designer_.get_RF_id(),wrenchConeRight,actuation_->get_nu());
// 	boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchConeCostRight1Contact = 
// 		boost::make_shared<crocoddyl::CostModelResidual>(state_,activationWrenchRight1Contact,wrenchResidualRight1Contact);
	
// 	switch(support)
// 	{
// 		case LEFT  : runningCostModel.get()->addCost("wrenchLeftContact", wrenchConeCostLeft1Contact, settings_.wWrenchCone,true);
// 			    	 runningCostModel.get()->addCost("wrenchRightContact", wrenchConeCostRight1Contact, settings_.wWrenchCone,false);
// 		             break;
// 		case RIGHT: runningCostModel.get()->addCost("wrenchLeftContact", wrenchConeCostLeft1Contact, settings_.wWrenchCone,false);
// 					runningCostModel.get()->addCost("wrenchRightContact", wrenchConeCostRight1Contact, settings_.wWrenchCone,true);
// 		            break;
// 		case DOUBLE : runningCostModel.get()->addCost("wrenchLeftContact", wrenchConeCostLeft2Contact, settings_.wWrenchCone,true);
// 			    	  runningCostModel.get()->addCost("wrenchRightContact", wrenchConeCostRight2Contact, settings_.wWrenchCone,true);
// 		              break;
//     }

// 	// Cost goaltracking
// 	boost::shared_ptr<crocoddyl::ActivationModelQuadFlatLog> actGoalWeights = 
// 		   boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(6,0.01);
	
// 	boost::shared_ptr<crocoddyl::ResidualModelFramePlacement> goalTrackingResidualRight = 
// 		boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state_, designer_.get_RF_id(), 
// 		designer_.get_rData().oMf[designer_.get_RF_id()],actuation_->get_nu());
	
// 	boost::shared_ptr<crocoddyl::ResidualModelFramePlacement> goalTrackingResidualLeft = 
// 		boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state_, designer_.get_LF_id(), 
// 		designer_.get_rData().oMf[designer_.get_LF_id()],actuation_->get_nu());
	
// 	boost::shared_ptr<crocoddyl::CostModelAbstract> goalTrackingCostLeft = 
// 		boost::make_shared<crocoddyl::CostModelResidual>(state_, actGoalWeights,goalTrackingResidualLeft);
// 	boost::shared_ptr<crocoddyl::CostModelAbstract> goalTrackingCostRight = 
// 		boost::make_shared<crocoddyl::CostModelResidual>(state_, actGoalWeights,goalTrackingResidualRight);
	
// 	runningCostModel.get()->addCost("placementFootRight", goalTrackingCostRight,settings_.wFootTrans,true);
// 	runningCostModel.get()->addCost("placementFootLeft", goalTrackingCostLeft,settings_.wFootTrans,true);
	
// 	// State cost
// 	boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> actxWeights = 
// 		 boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(settings_.stateWeights);

// 	boost::shared_ptr<crocoddyl::CostModelAbstract> xRegCost =
// 		boost::make_shared<crocoddyl::CostModelResidual>(state_,actxWeights,
// 		boost::make_shared<crocoddyl::ResidualModelState>(state_, x0_,actuation_->get_nu())); 
		
// 	runningCostModel.get()->addCost("stateReg", xRegCost, settings_.wStateReg,true);
// 	// Control cost
// 	boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> actuWeights = 
// 		 boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(settings_.controlWeights); //.tail(actuation->get_nu())
	
// 	boost::shared_ptr<crocoddyl::ResidualModelControl> uResidualControl =
// 		boost::make_shared<crocoddyl::ResidualModelControl>(state_,actuation_->get_nu());
// 	boost::shared_ptr<crocoddyl::CostModelAbstract>  uRegCost =
// 	    boost::make_shared<crocoddyl::CostModelResidual>(state_,actuWeights,uResidualControl);
	  
// 	runningCostModel.get()->addCost("ctrlReg", uRegCost, settings_.wControlReg,true);
	
// 	// Kinematic limits cost
// 	crocoddyl::ActivationBounds bounds = crocoddyl::ActivationBounds(designer_.get_rModel().lowerPositionLimit.tail(designer_.get_rModel().nq - 7), 
// 																	 designer_.get_rModel().upperPositionLimit.tail(designer_.get_rModel().nq - 7));

// 	boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> activationBounded = 
// 		 boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(bounds);
// 	boost::shared_ptr<crocoddyl::CostModelAbstract> jointLimitCost =
// 		boost::make_shared<crocoddyl::CostModelResidual>(state_, activationBounded,
// 		boost::make_shared<crocoddyl::ResidualModelState>(state_, actuation_->get_nu()));
	
// 	runningCostModel.get()->addCost("limitCost", jointLimitCost, settings_.wLimit,true);
	
// 	// Velocity CoM cost
// 	Eigen::Vector3d vRef;
// 	vRef.setZero();
// 	boost::shared_ptr<crocoddyl::CostModelAbstract> CoMVelocityCost =
// 		boost::make_shared<crocoddyl::CostModelResidual>(state_, 
// 			   boost::make_shared<ResidualModelCoMVelocity>(state_, vRef,actuation_->get_nu())); 
			   
// 	runningCostModel.get()->addCost("comVelCost", CoMVelocityCost, settings_.wVCoM,true);
	
// 	boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics> runningDAM =
// 	   boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(state_, actuation_, contactModel,runningCostModel,0.,true);
// 	boost::shared_ptr<crocoddyl::ActionModelAbstract>  runningModel =
// 	   boost::make_shared<crocoddyl::IntegratedActionModelEuler>(runningDAM, settings_.timeStep);  

// 	const boost::shared_ptr<crocoddyl::ActionDataAbstract> temp_data = runningModel->createData();
// 	Eigen::VectorXd uref = Eigen::VectorXd::Zero(Eigen::Index(actuation_->get_nu()));
// 	runningModel->quasiStatic(temp_data,uref,x0_);
	
// 	uResidualControl->set_reference(uref);
	
// 	return runningModel;
// }

// AMA ModelMaker::formulate_stair_climber(const Support &support){

//     // Here goes the full definition of contacts and costs to get the IAM
//     // Use the support enumeration to set the appropriate contact and wrench cone.

//     return AMA();
// }

std::vector<AMA> ModelMaker::formulateHorizon(const std::vector<Support> &supports){

    // for loop to generate a vector of IAMs
    std::vector<AMA> models;
    for(std::size_t i = 0; i < supports.size() ;i++)
    {
		models.push_back(formulateStepTracker(supports[i]));
	}

    return models;
}


}


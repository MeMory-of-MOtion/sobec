#include "sobec/horizon_manager.hpp"
#include <crocoddyl/multibody/fwd.hpp>

#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/core/integrator/euler.hpp> 


namespace sobec {

    HorizonManager::HorizonManager(){}

    HorizonManager::HorizonManager(const HorizonManagerSettings &settings, 
                                   const Eigen::VectorXd &x0, 
                                   const std::vector<AMA> &runningModels,
                                   const AMA &terminalModel){
        initialize(settings, x0, runningModels, terminalModel);
    }

    void HorizonManager::initialize(const HorizonManagerSettings &settings, 
                                    const Eigen::VectorXd &x0, 
                                    const std::vector<AMA> &runningModels,
                                    const AMA &terminalModel){ 
        settings_ = settings;
        boost::shared_ptr<crocoddyl::ShootingProblem> shooting_problem =
                boost::make_shared<crocoddyl::ShootingProblem>(x0, runningModels, terminalModel);
        ddp_ = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);
    }

    //OLD 
    AMA HorizonManager::ama(const unsigned long &time){
        return ddp_->get_problem()->get_runningModels()[time];
    }

    IAM HorizonManager::iam(const unsigned long &time){
        return boost::static_pointer_cast< crocoddyl::IntegratedActionModelEuler >(ama(time));
    }

    DAM HorizonManager::dam(const unsigned long &time){
        return boost::static_pointer_cast<crocoddyl::DifferentialActionModelContactFwdDynamics >(iam(time)->get_differential());
    }

    Cost HorizonManager::costs(const unsigned long &time){
        return dam(time)->get_costs();
    }

    Contact HorizonManager::contacts(const unsigned long &time){
        return dam(time)->get_contacts();
    }

    IAD HorizonManager::data(const unsigned long &time){
        return boost::static_pointer_cast< crocoddyl::IntegratedActionDataEuler >(ddp_->get_problem()->get_runningDatas()[time]);
    }

    void HorizonManager::activateContactLF(const unsigned long &time){
        contacts(time)->changeContactStatus(settings_.leftFootName, true);
    }

    void HorizonManager::activateContactRF(const unsigned long &time){
        contacts(time)->changeContactStatus(settings_.rightFootName, true);
    }

    void HorizonManager::removeContactLF(const unsigned long &time){
        contacts(time)->changeContactStatus(settings_.leftFootName, false);
    }

    void HorizonManager::removeContactRF(const unsigned long &time){
        contacts(time)->changeContactStatus(settings_.rightFootName, false);
    }
    
    void HorizonManager::setPoseReferenceLF(const unsigned long &time, const pinocchio::SE3 &ref_placement){
		boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement >(costs(time)->get_costs().at("placementFootLeft")->cost->get_residual())->set_reference(ref_placement);
    }

    void HorizonManager::setPoseReferenceRF(const unsigned long &time, const pinocchio::SE3 &ref_placement){
		boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement >(costs(time)->get_costs().at("placementFootRight")->cost->get_residual())->set_reference(ref_placement);
    }

    void HorizonManager::setForceReferenceLF(const unsigned long &time, const eVector6 &reference){
        cone_ = boost::static_pointer_cast<crocoddyl::CostModelResidual>(costs(time)->get_costs().at("wrenchLeftContact")->cost);
        Eigen::VectorXd new_ref = boost::static_pointer_cast<crocoddyl::ResidualModelContactWrenchCone>(cone_->get_residual())->get_reference().get_A() * reference;
        boost::static_pointer_cast<ActivationModelQuadRef>(cone_->get_activation())->set_reference(new_ref);
    }
    
    void HorizonManager::setForceReferenceRF(const unsigned long &time, const eVector6 &reference){
        cone_ = boost::static_pointer_cast<crocoddyl::CostModelResidual>(costs(time)->get_costs().at("wrenchRightContact")->cost);
        Eigen::VectorXd new_ref = boost::static_pointer_cast<crocoddyl::ResidualModelContactWrenchCone>(cone_->get_residual())->get_reference().get_A() * reference;
        boost::static_pointer_cast<ActivationModelQuadRef>(cone_->get_activation())->set_reference(new_ref);
    }

    void HorizonManager::setSwingingLF(const unsigned long &time){
        removeContactLF(time);
        setForceReferenceLF(time, eVector6::Zero());
    }
    void HorizonManager::setSwingingRF(const unsigned long &time){
        removeContactRF(time);
        setForceReferenceRF(time, eVector6::Zero());
    }
    
    void HorizonManager::setSupportingLF(const unsigned long &time){
        activateContactLF(time);
    }
    void HorizonManager::setSupportingRF(const unsigned long &time){
        activateContactRF(time);
    }

    // end OLD



    // NEW

    void HorizonManager::updateIAM(const unsigned long &time){
		IAM_ = boost::static_pointer_cast< crocoddyl::IntegratedActionModelEuler >(ddp_->get_problem()->get_runningModels()[time]);
    }

    void HorizonManager::updateDAM(const unsigned long &time){
		updateIAM(time);
		DAM_ = boost::static_pointer_cast<crocoddyl::DifferentialActionModelContactFwdDynamics >(IAM_->get_differential());
    }

    Cost HorizonManager::costs(){
        return DAM_->get_costs();
    }

    Contact HorizonManager::contacts(){
        return DAM_->get_contacts();
    }

    void HorizonManager::setPlacementReferenceRF(const pinocchio::SE3 &ref_placement){
		goalTrackingResidual_ = 
		    boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement >(costs()->get_costs().at("placementFootRight")->cost->get_residual());
		goalTrackingResidual_->set_reference(ref_placement);
    }
    
    void HorizonManager::setPlacementReferenceLF(const pinocchio::SE3 &ref_placement){
		goalTrackingResidual_ = 
		    boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement >(costs()->get_costs().at("placementFootLeft")->cost->get_residual());
		goalTrackingResidual_->set_reference(ref_placement);
    }
    // void HorizonManager::setResidualReferences(unsigned long time, const std::string &name);

    void HorizonManager::activateContactLF(){
        contacts()->changeContactStatus(designer_.get_LF_name(), true);
    }

    void HorizonManager::activateContactRF(){
        contacts()->changeContactStatus(designer_.get_RF_name(), true);
    }

    void HorizonManager::removeContactLF(){
        contacts()->changeContactStatus(designer_.get_LF_name(), false);
    }

    void HorizonManager::removeContactRF(){
        contacts()->changeContactStatus(designer_.get_RF_name(), false);
    }

    void HorizonManager::setForceReferenceLF(const eVector6 &reference){
		quadRefActivationPtr_ = 
		    boost::static_pointer_cast<ActivationModelQuadRef>(costs()->get_costs().at("wrenchLeftContact")->cost->get_activation());
		wrenchConeResidual_ = 
		    boost::static_pointer_cast<crocoddyl::ResidualModelContactWrenchCone>(costs()->get_costs().at("wrenchLeftContact")->cost->get_residual());
		wrench_cone_ = wrenchConeResidual_->get_reference();
		
		quadRefActivationPtr_->set_reference(wrench_cone_.get_A() * reference);
    }
    
    void HorizonManager::setForceReferenceRF(const eVector6 &reference){
        quadRefActivationPtr_ = 
		    boost::static_pointer_cast<ActivationModelQuadRef>(DAM_->get_costs()->get_costs().at("wrenchRightContact")->cost->get_activation());
		wrenchConeResidual_ = 
		    boost::static_pointer_cast<crocoddyl::ResidualModelContactWrenchCone>(DAM_->get_costs()->get_costs().at("wrenchRightContact")->cost->get_residual());
		wrench_cone_ = wrenchConeResidual_->get_reference();
		
		quadRefActivationPtr_->set_reference(wrench_cone_.get_A() * reference);
    }

    void HorizonManager::setSwingingLF(const unsigned long &time, const pinocchio::SE3 &ref_placement,  const eVector6 &ref_wrench){
		updateDAM(time);
        removeContactLF();
        setForceReferenceLF(eVector6::Zero());
        setForceReferenceRF(ref_wrench);
        setPlacementReferenceLF(ref_placement);
    }
    void HorizonManager::setSwingingRF(const unsigned long &time, const pinocchio::SE3 &ref_placement, const eVector6 &ref_wrench){
		updateDAM(time);
        removeContactRF();
        setForceReferenceRF(eVector6::Zero());
        setForceReferenceLF(ref_wrench);
        setPlacementReferenceRF(ref_placement);
    }
    void HorizonManager::setSupportingFeet(const unsigned long &time, const eVector6 &ref_wrench){
		updateDAM(time);
        activateContactLF();
        activateContactRF();
        setForceReferenceRF(ref_wrench);
        setForceReferenceLF(ref_wrench);
    }

    void HorizonManager::recede(const IAM &new_model, const IAD &new_data){
        ddp_->get_problem()->circularAppend(new_model, new_data);
    }
    
    void HorizonManager::recede(const IAM &new_model){
        IAD_ = boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(new_model->createData());
        ddp_->get_problem()->circularAppend(IAM_, IAD_);
    }
    
    void HorizonManager::recede(){
		IAD_ = boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(IAM_->createData());
        ddp_->get_problem()->circularAppend(IAM_, IAD_);
    }
    
    unsigned long HorizonManager::get_size(){
        return ddp_->get_problem()->get_T();
    }
    
    void HorizonManager::solve_ddp(const std::vector<Eigen::VectorXd> xs, const std::vector<Eigen::VectorXd> us, const std::size_t &ddpIteration){
        ddp_->solve(xs,us,ddpIteration,false);
        us_ = ddp_->get_us();
        xs_ = ddp_->get_xs();
    }
    
    void HorizonManager::solveControlCycle(const Eigen::VectorXd &measured_x, const std::size_t &ddpIteration){
		xs_.erase(xs_.begin());
		xs_[0] = measured_x;
		xs_.push_back(xs_[xs_.size()-1]);

		us_.erase(us_.begin());
		us_.push_back(us_[us_.size()-1]);

		// Update initial state
		ddp_->get_problem()->set_x0(measured_x);
		ddp_->allocateData();
		
		ddp_->solve(xs_,us_,ddpIteration,false);
		us_ = ddp_->get_us();
		xs_ = ddp_->get_xs();
	}
		
}

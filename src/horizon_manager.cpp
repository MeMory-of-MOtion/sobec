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

    IAD HorizonManager::data(const unsigned long &time){
        return boost::static_pointer_cast< crocoddyl::IntegratedActionDataEuler >(ddp_->get_problem()->get_runningDatas()[time]);
    }

    void HorizonManager::setPlacementReferenceRF(const pinocchio::SE3 &ref_placement){
		goalTrackingResidual_ = 
		    boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement >(costs()->get_costs().at("placementFootRight")->cost->get_residual());
		goalTrackingResidual_->set_reference(ref_placement);
    }
    
    void HorizonManager::setPlacementReferenceLF(const pinocchio::SE3 &ref_placement){
		goalTrackingResidual_ = 
		    boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement >(costs()->get_costs().at("placementFootRight")->cost->get_residual());
		goalTrackingResidual_->set_reference(ref_placement);
    }
    // void HorizonManager::setResidualReferences(unsigned long time, const std::string &name);

    void HorizonManager::activateContactLF(){
        contacts()->changeContactStatus(settings_.leftFootName, true);
        costs()->changeCostStatus("placementFootLeft", false);
    }

    void HorizonManager::activateContactRF(){
        contacts()->changeContactStatus(settings_.rightFootName, true);
        costs()->changeCostStatus("placementFootRight", false);
    }

    void HorizonManager::removeContactLF(){
        contacts()->changeContactStatus(settings_.leftFootName, false);
        costs()->changeCostStatus("placementFootLeft", true);
    }

    void HorizonManager::removeContactRF(){
        contacts()->changeContactStatus(settings_.rightFootName, false);
        costs()->changeCostStatus("placementFootRight", true);
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
    void HorizonManager::setSupportingLF(const unsigned long &time, const eVector6 &ref_wrench){
		updateDAM(time);
        activateContactLF();
        setForceReferenceRF(ref_wrench);
        setForceReferenceLF(ref_wrench);
    }
    void HorizonManager::setSupportingRF(const unsigned long &time, const eVector6 &ref_wrench){
		updateDAM(time);
        activateContactRF();
        setForceReferenceRF(ref_wrench);
        setForceReferenceLF(ref_wrench);
    }

    // std::HorizonManager::vector<std::string> active_contacts(const unsigned long &time);
    // std::HorizonManager::vector<Eigen::VectorXd> preview_states();
    // std::HorizonManager::vector<Eigen::VectorXd> preview_actions();

    void HorizonManager::recede(IAM new_model, IAD new_data){
        ddp_->get_problem()->circularAppend(new_model, new_data);
    }
    
    void HorizonManager::recede(IAM new_model){
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
}

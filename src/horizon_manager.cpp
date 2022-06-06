#include "sobec/horizon_manager.hpp"
#include <crocoddyl/multibody/fwd.hpp>

#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/core/integrator/euler.hpp> 


namespace sobec {

    HorizonManager::HorizonManager(){}

    HorizonManager::HorizonManager(const Eigen::VectorXd &x0, 
                                   const ModelMakerSettings &settings,
                                   const RobotDesigner &design,
                                   const std::size_t horizon_length,
                                   const std::size_t ddp_iteration){
        initialize(x0, settings, design,horizon_length,ddp_iteration);
    }

    void HorizonManager::initialize(const Eigen::VectorXd &x0, 
                                    const ModelMakerSettings &settings,
                                    const RobotDesigner &design,
                                    const std::size_t horizon_length,
                                    const std::size_t ddp_iteration){ 
        
        designer_ = sobec::RobotDesigner(design);
        modelMaker_ = sobec::ModelMaker(settings, design);
        horizon_length_ = horizon_length;
        ddp_iteration_ = ddp_iteration;
        
        std::vector<Support> supports(horizon_length_, Support::DOUBLE);
        
        std::vector<AMA> runningModels = modelMaker_.formulateHorizon(supports, horizon_length);
        AMA terminalModel = modelMaker_.formulate_flat_walker(Support::DOUBLE);
        boost::shared_ptr<crocoddyl::ShootingProblem> shooting_problem =
                boost::make_shared<crocoddyl::ShootingProblem>(x0, runningModels, terminalModel);
        ddp_ = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);
        
        std::vector<Eigen::VectorXd> x_init;
        std::vector<Eigen::VectorXd> u_init;
        Eigen::VectorXd zero_u = Eigen::VectorXd::Zero(designer_.get_rModel().nv - 6);
        
        for(std::size_t i = 0; i < horizon_length_ ;i++)
        {
			x_init.push_back(x0);
			u_init.push_back(zero_u);
		}
		x_init.push_back(x0);
		ddp_->solve(x_init,u_init,500,false);
		
		us_ = ddp_->get_us();
        xs_ = ddp_->get_xs();
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
        contacts()->changeContactStatus(designer_.get_LF_name(), true);
        costs()->changeCostStatus("placementFootLeft", false);
    }

    void HorizonManager::activateContactRF(){
        contacts()->changeContactStatus(designer_.get_RF_name(), true);
        costs()->changeCostStatus("placementFootRight", false);
    }

    void HorizonManager::removeContactLF(){
        contacts()->changeContactStatus(designer_.get_LF_name(), false);
        costs()->changeCostStatus("placementFootLeft", true);
    }

    void HorizonManager::removeContactRF(){
        contacts()->changeContactStatus(designer_.get_RF_name(), false);
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
    
    void HorizonManager::solveControlCycle(const Eigen::VectorXd &xCurrent){
		xs_.erase(xs_.begin());
		xs_[0] = xCurrent;
		xs_.push_back(xs_[xs_.size()-1]);

		us_.erase(us_.begin());
		us_.push_back(us_[us_.size()-1]);

		// Update initial state
		ddp_->get_problem()->set_x0(xCurrent);
		ddp_->allocateData();
		
		ddp_->solve(xs_,us_,ddp_iteration_,false);
		us_ = ddp_->get_us();
		xs_ = ddp_->get_xs();
	}
}

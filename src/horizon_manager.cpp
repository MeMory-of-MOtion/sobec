#include "sobec/horizon_manager.hpp"
#include <crocoddyl/multibody/fwd.hpp>

#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/core/integrator/euler.hpp> 


namespace sobec {

    HorizonManager::HorizonManager(){}

    HorizonManager::HorizonManager(const HorizonManagerSettings &settings, 
                                const Eigen::VectorXd &x0, 
                                const std::vector<AMA> &runningModels,
                                const IAM &terminalModel){
        initialize(settings, x0, runningModels, terminalModel);
    }

    void HorizonManager::initialize(const HorizonManagerSettings &settings, 
                                    const Eigen::VectorXd &x0, 
                                    const std::vector<AMA> &runningModels,
                                    const IAM &terminalModel){ 
        settings_ = settings;
        boost::shared_ptr<crocoddyl::ShootingProblem> shooting_problem =
                boost::make_shared<crocoddyl::ShootingProblem>(x0, runningModels, terminalModel);
        DDP ddp_ = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);
    }

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

    // void HorizonManager::setResidualReference(unsigned long time, const std::string &name,  const auto &new_value){
    //     costs(time)->get_costs()[name].cost.referece = new_value;
    // }
    // void HorizonManager::setResidualReferences(unsigned long time, const std::string &name);

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

    void HorizonManager::setForceReferenceLF(const unsigned long &time, const eVector6 &reference){
        /** Important, set the foot name on the wrench cone cost.*/
        // boost::shared_ptr<crocoddyl::CostItem> cone = costs(time)->get_costs()[settings_.leftFootName];
    }
    void HorizonManager::setForceReferenceRF(const unsigned long &time, const eVector6 &reference){
        // TODO: Implement
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

    // std::HorizonManager::vector<std::string> active_contacts(const unsigned long &time);
    // std::HorizonManager::vector<Eigen::VectorXd> preview_states();
    // std::HorizonManager::vector<Eigen::VectorXd> preview_actions();

    void HorizonManager::recede(IAM new_model, IAD new_data){
        ddp_->get_problem()->circularAppend(new_model, new_data);
    }
    void HorizonManager::recede(IAM new_model){
        IAD new_data = boost::static_pointer_cast< crocoddyl::IntegratedActionDataEuler >(new_model->createData());
        recede(new_model, new_data);
    }
    void HorizonManager::recede(){
        IAM new_model = iam(0);
        IAD new_data = data(0);
        recede(new_model, new_data);
    }

    unsigned long HorizonManager::get_size(){
        return ddp_->get_problem()->get_T();
    }
}

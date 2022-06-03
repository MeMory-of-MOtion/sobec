#include "sobec/horizon_manager.hpp"
#include <crocoddyl/multibody/fwd.hpp>


#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/frames.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/residuals/anticipated-state.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/contact-control-gravity.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/com-velocity.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/contact-wrench-cone.hpp>
#include <crocoddyl/multibody/contacts/contact-3d.hpp>
#include <crocoddyl/multibody/contacts/contact-6d.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/contact-force.hpp>
#include <crocoddyl/multibody/actions/impulse-fwddyn.hpp>
#include <crocoddyl/multibody/impulses/impulse-6d.hpp>

#include <crocoddyl/core/activations/quadratic-ref.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/solver-base.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/quadratic-flat-log.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/activations/smooth-1norm.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/solvers/ddp.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/integrator/euler.hpp> // this one used
#include <crocoddyl/core/solvers/ddp.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/residuals/control.hpp>

namespace sobec {

HorizonManager::HorizonManager(){}

HorizonManager::HorizonManager(const HorizonManagerSettings &settings, 
                            const std::vector<IAM> &runningModels,
                           const IAM &terminalModel){
    initialize(settings, runningModels, terminalModel);
}

void HorizonManager::initialize(const HorizonManagerSettings &settings, 
                            const std::vector<IAM> &runningModels,
                            const IAM &terminalModel){ 
    settings_ = settings;

    // set the ddp_ from shootingProblem with the runningModels
}

AMA HorizonManager::ama(const unsigned long &time){
    return ddp_->get_problem()->get_runningModels()[time];
}

IAM HorizonManager::iam(const unsigned long &time){
    return boost::static_pointer_cast< crocoddyl::IntegratedActionModelEuler >(ama(time));
}

DAM HorizonManager::dam(const unsigned long &time){
    return boost::static_pointer_cast<crocoddyl::DifferentialActionModelContactFwdDynamics >(iam(time)->get_differential());
    // boost::static_pointer_cast<crocoddyl::DifferentialActionModelContactFwdDynamics >(iam(time));
}



/*Cost HorizonManager::costs(const unsigned long &time);
Contact HorizonManager::contacts(const unsigned long &time);
IAD HorizonManager::data(const unsigned long &time);

void HorizonManager::setResidualReference(unsigned long time, const std::string &name,  const auto &new_value);
void HorizonManager::setResidualReferences(unsigned long time, const std::string &name);

void HorizonManager::activateContactLF(const unsigned long &time);
void HorizonManager::activateContactRF(const unsigned long &time);
void HorizonManager::removeContactLF(const unsigned long &time);
void HorizonManager::removeContactRF(const unsigned long &time);
void HorizonManager::setForceReferenceLF(const unsigned long &time, const eVector6 &reference);
void HorizonManager::setForceReferenceRF(const unsigned long &time, const eVector6 &reference);
void HorizonManager::setSwingingLF(const unsigned long &time);
void HorizonManager::setSwingingRF(const unsigned long &time);
void HorizonManager::setSupportingLF(const unsigned long &time);
void HorizonManager::setSupportingRF(const unsigned long &time);

std::HorizonManager::vector<std::string> get_contacts(const unsigned long &time);
std::HorizonManager::vector<Eigen::VectorXd> preview_states();
std::HorizonManager::vector<Eigen::VectorXd> preview_actions();

void HorizonManager::recede(IAM new_model, IAD new_data);

unsigned HorizonManager::long get_size();
*/

}

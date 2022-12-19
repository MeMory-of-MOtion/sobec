#include "sobec/walk-with-traj/horizon_manager.hpp"

#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/fwd.hpp>

namespace sobec {

HorizonManager::HorizonManager() {}

HorizonManager::HorizonManager(const HorizonManagerSettings &settings,
                               const Eigen::VectorXd &x0,
                               const std::vector<AMA> &runningModels,
                               const AMA &terminalModel) {
  initialize(settings, x0, runningModels, terminalModel);
}

void HorizonManager::initialize(const HorizonManagerSettings &settings,
                                const Eigen::VectorXd &x0,
                                const std::vector<AMA> &runningModels,
                                const AMA &terminalModel) {
  settings_ = settings;
  boost::shared_ptr<crocoddyl::ShootingProblem> shooting_problem =
      boost::make_shared<crocoddyl::ShootingProblem>(x0, runningModels,
                                                     terminalModel);
  ddp_ = boost::make_shared<crocoddyl::SolverFDDP>(shooting_problem);
  new_ref_.resize(17);

  tr_error_.resize(state(0)->get_nx() - 1);
  K_tr_error_.resize(actuation(0)->get_nu());
  command_torque_.resize(actuation(0)->get_nu());

  initialized_ = true;
}

AMA HorizonManager::ama(const unsigned long time) {
  return ddp_->get_problem()->get_runningModels()[time];
}

IAM HorizonManager::iam(const unsigned long time) {
  return boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
      ama(time));
}

IAM HorizonManager::terminaliam() {
  return boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
      ddp_->get_problem()->get_terminalModel());
}

DAM HorizonManager::dam(const unsigned long time) {
  return boost::static_pointer_cast<
      crocoddyl::DifferentialActionModelContactFwdDynamics>(
      iam(time)->get_differential());
}

DAM HorizonManager::terminaldam() {
  return boost::static_pointer_cast<
      crocoddyl::DifferentialActionModelContactFwdDynamics>(
      terminaliam()->get_differential());
}

Cost HorizonManager::costs(const unsigned long time) {
  return dam(time)->get_costs();
}

Cost HorizonManager::terminalCosts() { return terminaldam()->get_costs(); }

Contact HorizonManager::contacts(const unsigned long time) {
  return dam(time)->get_contacts();
}

ADA HorizonManager::ada(const unsigned long time) {
  return ddp_->get_problem()->get_runningDatas()[time];
}

IAD HorizonManager::iad(const unsigned long time) {
  return boost::static_pointer_cast<crocoddyl::IntegratedActionDataEuler>(
      ada(time));
}

DAD HorizonManager::dad(const unsigned long time) {
  return boost::static_pointer_cast<
      crocoddyl::DifferentialActionDataContactFwdDynamics>(
      iad(time)->differential);
}

pinocchio::Data HorizonManager::pinData(const unsigned long time) {
  return dad(time)->pinocchio;
}

boost::shared_ptr<crocoddyl::StateMultibody> HorizonManager::state(
    const unsigned long time) {
  return boost::static_pointer_cast<crocoddyl::StateMultibody>(
      dam(time)->get_state());
}

boost::shared_ptr<crocoddyl::ActuationModelFloatingBase>
HorizonManager::actuation(const unsigned long time) {
  return boost::static_pointer_cast<crocoddyl::ActuationModelFloatingBase>(
      dam(time)->get_actuation());
}

void HorizonManager::activateContactLF(const unsigned long time,
                                       const std::string &nameContactLF) {
  contacts(time)->changeContactStatus(nameContactLF, true);
}

void HorizonManager::activateContactRF(const unsigned long time,
                                       const std::string &nameContactRF) {
  contacts(time)->changeContactStatus(nameContactRF, true);
}

void HorizonManager::removeContactLF(const unsigned long time,
                                     const std::string &nameContactLF) {
  contacts(time)->changeContactStatus(nameContactLF, false);
}

void HorizonManager::removeContactRF(const unsigned long time,
                                     const std::string &nameContactRF) {
  contacts(time)->changeContactStatus(nameContactRF, false);
}

void HorizonManager::setBalancingTorque(const unsigned long time,
                                        const std::string &nameCostActuation,
                                        const Eigen::VectorXd &x) {
  Eigen::VectorXd balancingTorque;
  balancingTorque.resize(iam(time)->get_nu());
  iam(time)->quasiStatic(ada(time), balancingTorque, x);
  setActuationReference(time, nameCostActuation, balancingTorque);
}
/// @todo: All functions using string names, should receive such names as input.

void HorizonManager::setBalancingTorque(const unsigned long time,
                                        const std::string &nameCostActuation,
                                        const std::string &nameCostState) {
  Eigen::VectorXd x =
      boost::static_pointer_cast<crocoddyl::ResidualModelState>(
          costs(time)->get_costs().at(nameCostState)->cost->get_residual())
          ->get_reference();
  setBalancingTorque(time, nameCostActuation, x);
}

void HorizonManager::setActuationReference(const unsigned long time,
                                           const std::string &nameCostActuation,
                                           const Eigen::VectorXd &reference) {
  boost::static_pointer_cast<crocoddyl::ResidualModelControl>(
      costs(time)->get_costs().at(nameCostActuation)->cost->get_residual())
      ->set_reference(reference);
}

Eigen::VectorXd HorizonManager::getActuationReference(
    const unsigned long time, const std::string &nameCostActuation) {
  return boost::static_pointer_cast<crocoddyl::ResidualModelControl>(
             costs(time)
                 ->get_costs()
                 .at(nameCostActuation)
                 ->cost->get_residual())
      ->get_reference();
}

void HorizonManager::setPoseReference(const unsigned long time,
                                      const std::string &nameCost,
                                      const pinocchio::SE3 &ref_placement) {
  boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement>(
      costs(time)->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_placement);
}

void HorizonManager::setRotationReference(const unsigned long time,
                                          const std::string &nameCost,
                                          const Eigen::Matrix3d &ref_rotation) {
  boost::static_pointer_cast<crocoddyl::ResidualModelFrameRotation>(
      costs(time)->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_rotation);
}

void HorizonManager::setTerminalRotationReference(
    const std::string &nameCost, const Eigen::Matrix3d &ref_rotation) {
  boost::static_pointer_cast<crocoddyl::ResidualModelFrameRotation>(
      terminalCosts()->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_rotation);
}

void HorizonManager::setTranslationReference(const unsigned long time,
                                             const std::string &nameCost,
                                             const eVector3 &ref_translation) {
  boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(
      costs(time)->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_translation);
}

void HorizonManager::setTerminalPoseReference(
    const std::string &nameCost, const pinocchio::SE3 &ref_placement) {
  boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement>(
      terminalCosts()->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_placement);
}

void HorizonManager::setTerminalDCMReference(const std::string &nameCost,
                                             const eVector3 &ref_translation) {
  boost::static_pointer_cast<sobec::ResidualModelDCMPosition>(
      terminalCosts()->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_translation);
}

void HorizonManager::setTerminalTranslationReference(
    const std::string &nameCost, const eVector3 &ref_translation) {
  boost::static_pointer_cast<crocoddyl::ResidualModelFrameTranslation>(
      terminalCosts()->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_translation);
}

void HorizonManager::setTerminalPoseCoM(const std::string &nameCost,
                                        const eVector3 &ref_placement) {
  boost::static_pointer_cast<crocoddyl::ResidualModelCoMPosition>(
      terminalCosts()->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_placement);
}

const pinocchio::SE3 &HorizonManager::getFootPoseReference(
    const unsigned long time, const std::string &nameCostFootPose) {
  pose_ =
      boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement>(
          costs(time)->get_costs().at(nameCostFootPose)->cost->get_residual())
          ->get_reference();
  return pose_;
}

const pinocchio::SE3 &HorizonManager::getTerminalFootPoseReference(
    const std::string &nameCostFootPose) {
  pose_ = boost::static_pointer_cast<crocoddyl::ResidualModelFramePlacement>(
              terminalCosts()
                  ->get_costs()
                  .at(nameCostFootPose)
                  ->cost->get_residual())
              ->get_reference();
  return pose_;
}

void HorizonManager::setVelocityRefCOM(const unsigned long time,
                                       const std::string &nameCost,
                                       const eVector3 &ref_velocity) {
  boost::static_pointer_cast<sobec::ResidualModelCoMVelocity>(
      costs(time)->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_velocity);
}

void HorizonManager::setVelocityRefFeet(const unsigned long time,
                                        const std::string &nameCost,
                                        const pinocchio::Motion &ref_velocity) {
  boost::static_pointer_cast<sobec::ResidualModelFrameVelocity>(
      costs(time)->get_costs().at(nameCost)->cost->get_residual())
      ->set_reference(ref_velocity);
}

void HorizonManager::setSurfaceInequality(const unsigned long time,
                                          const std::string &nameCost,
                                          const eVector2 &XYpose,
                                          const double &orientation) {
  boost::static_pointer_cast<sobec::ResidualModel2DSurface>(
      costs(time)->get_costs().at(nameCost)->cost->get_residual())
      ->set_Ab(XYpose, orientation);
}

void HorizonManager::setForceReference(const unsigned long time,
                                       const std::string &nameCost,
                                       const eVector6 &reference) {
  force_cost_ = boost::static_pointer_cast<crocoddyl::CostModelResidual>(
      costs(time)->get_costs().at(nameCost)->cost);
  force_ = pinocchio::Force(reference);
  boost::static_pointer_cast<crocoddyl::ResidualModelContactForce>(
      force_cost_->get_residual())
      ->set_reference(force_);
}

void HorizonManager::setWrenchReference(const unsigned long time,
                                        const std::string &nameCost,
                                        const eVector6 &reference) {
  cone_ = boost::static_pointer_cast<crocoddyl::CostModelResidual>(
      costs(time)->get_costs().at(nameCost)->cost);
  new_ref_ =
      boost::static_pointer_cast<crocoddyl::ResidualModelContactWrenchCone>(
          cone_->get_residual())
          ->get_reference()
          .get_A() *
      reference;
  boost::static_pointer_cast<ActivationModelQuadRef>(cone_->get_activation())
      ->set_reference(new_ref_);
}

void HorizonManager::setSwingingLF(const unsigned long time,
                                   const std::string &nameContactLF,
                                   const std::string &nameContactRF,
                                   const std::string &nameForceCostLF) {
  removeContactLF(time, nameContactLF);
  activateContactRF(time, nameContactRF);
  setWrenchReference(time, nameForceCostLF, eVector6::Zero());
}

void HorizonManager::setSwingingRF(const unsigned long time,
                                   const std::string &nameContactLF,
                                   const std::string &nameContactRF,
                                   const std::string &nameForceCostRF) {
  activateContactLF(time, nameContactLF);
  removeContactRF(time, nameContactRF);
  setWrenchReference(time, nameForceCostRF, eVector6::Zero());
}

void HorizonManager::setDoubleSupport(const unsigned long time,
                                      const std::string &nameContactLF,
                                      const std::string &nameContactRF) {
  activateContactLF(time, nameContactLF);
  activateContactRF(time, nameContactRF);
}

const eVector3 &HorizonManager::getFootForce(
    const unsigned long time, const std::string &nameFootForceCost) {
  foot_force_ = boost::static_pointer_cast<crocoddyl::ResidualDataContactForce>(
                    boost::static_pointer_cast<
                        crocoddyl::DifferentialActionDataContactFwdDynamics>(
                        iad(time)->differential)
                        ->costs->costs.find(nameFootForceCost)
                        ->second->residual)
                    ->contact->f.linear();
  return foot_force_;
}

const eVector3 &HorizonManager::getFootTorque(
    const unsigned long time, const std::string &nameFootForceCost) {
  foot_torque_ =
      boost::static_pointer_cast<crocoddyl::ResidualDataContactForce>(
          boost::static_pointer_cast<
              crocoddyl::DifferentialActionDataContactFwdDynamics>(
              iad(time)->differential)
              ->costs->costs.find(nameFootForceCost)
              ->second->residual)
          ->contact->f.angular();
  return foot_torque_;
}

void HorizonManager::recede(const AMA &new_model, const ADA &new_data) {
  ddp_->get_problem()->circularAppend(new_model, new_data);
}

void HorizonManager::recede(const AMA &new_model) {
  ddp_->get_problem()->circularAppend(new_model, new_model->createData());
}

void HorizonManager::recede() {
  ddp_->get_problem()->circularAppend(ama(0), ada(0));
}

unsigned long HorizonManager::size() {
  size_ = ddp_->get_problem()->get_T();
  return size_;
}

int HorizonManager::supportSize(const unsigned long time) {
  get_contacts(time);
  support_size_ = 2;
  if (active_contacts_.find(settings_.leftFootName) == active_contacts_.end())
    support_size_ -= 1;
  if (active_contacts_.find(settings_.rightFootName) == active_contacts_.end())
    support_size_ -= 1;
  return support_size_;
}

const std::set<std::string> &HorizonManager::get_contacts(
    const unsigned long time) {
  active_contacts_ = contacts(time)->get_active_set();
  return active_contacts_;
}

void HorizonManager::solve(const Eigen::VectorXd &measured_x,
                           const std::size_t ddpIteration,
                           const bool is_feasible) {
  warm_xs_ = ddp_->get_xs();
  warm_xs_.erase(warm_xs_.begin());
  warm_xs_[0] = measured_x;
  warm_xs_.push_back(warm_xs_[warm_xs_.size() - 1]);

  warm_us_ = ddp_->get_us();
  warm_us_.erase(warm_us_.begin());
  warm_us_.push_back(warm_us_[warm_us_.size() - 1]);

  // Update initial state
  ddp_->get_problem()->set_x0(measured_x);
  ddp_->allocateData();

  ddp_->solve(warm_xs_, warm_us_, ddpIteration, is_feasible);
}

const Eigen::VectorXd &HorizonManager::currentTorques(
    const Eigen::VectorXd &measured_x) {
  /// @todo: make a boolean -> IT is necessary to have solved at least one time
  /// to use this method.
  tr_error_ = state(0)->diff_dx(measured_x, ddp_->get_xs()[0]);
  K_tr_error_ = ddp_->get_K()[0] * tr_error_;
  command_torque_ = ddp_->get_us()[0] + K_tr_error_ * tr_error_;

  return command_torque_;
}

}  // namespace sobec

#include "sobec/hand_table/wbc_hand.hpp"

namespace sobec {

WBCHand::WBCHand() {}

WBCHand::WBCHand(const WBCHandSettings &settings, const RobotDesigner &design,
         const HorizonManager &horizon, const Eigen::VectorXd &q0,
         const Eigen::VectorXd &v0, const std::string &actuationCostName) {
  initialize(settings, design, horizon, q0, v0, actuationCostName);
}

void WBCHand::initialize(const WBCHandSettings &settings, const RobotDesigner &design,
                     const HorizonManager &horizon, const Eigen::VectorXd &q0,
                     const Eigen::VectorXd &v0,
                     const std::string &actuationCostName) {
  /** The posture required here is the full robot posture in the order of
   * pinicchio*/
  if (!design.initialized_ || !horizon.initialized_) {
    throw std::runtime_error("The designer and horizon must be initialized.");
  }
  settings_ = settings;
  designer_ = design;
  horizon_ = horizon;

  // designer settings
  controlled_joints_id_ = designer_.get_controlledJointsIDs();

  x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
  x0_ << designer_.shapeState(q0, v0);
  designer_.updateReducedModel(x0_);
  designer_.updateCompleteModel(q0);
  ref_LH_pose_ = designer_.get_LH_frame().translation();
  ref_RH_pose_ = designer_.get_RH_frame().translation();
  ref_com_ = designer_.get_com_position();
  ref_com_vel_ = eVector3::Zero();
  ref_force_ = eVector6::Zero();

  // horizon settings
  std::vector<Eigen::VectorXd> xs_init;
  std::vector<Eigen::VectorXd> us_init;
  Eigen::VectorXd zero_u = Eigen::VectorXd::Zero(designer_.get_rModel().nv - 6);

  for (std::size_t i = 0; i < horizon_.size(); i++) {
    ///@todo: Remove this from the initialization and provide it as a method.
    xs_init.push_back(x0_);
    us_init.push_back(zero_u);
    horizon_.setBalancingTorque(i, actuationCostName, x0_);
  }
  xs_init.push_back(x0_);

  horizon_.get_ddp()->solve(xs_init, us_init, 500, false);
  
  iteration_ = 0;
  initialized_ = true;
}

void WBCHand::generateFullHorizon(ModelMakerHand &mm) {
  std::vector<Phase> cycle;
  
  land_hand_ = settings_.TtrackingToContact;
  takeoff_hand_ = settings_.TtrackingToContact + settings_.Tcontact;
  
  for (int i = 0; i < takeoff_hand_ + settings_.T; i++) {
	if (i < land_hand_)
	  cycle.push_back(TRACKING_RIGHT);
	else if (i < takeoff_hand_)
	  cycle.push_back(CONTACT_RIGHT);
	else
	  cycle.push_back(NO_HAND);
  }
  std::vector<AMA> cyclicModels;
  cyclicModels = mm.formulateHorizon(cycle);
  HorizonManagerSettings names = {designer_.get_LF_name(),
                                  designer_.get_RF_name()};
  fullHorizon_ = HorizonManager(names, x0_, cyclicModels,
                                 cyclicModels.back());
}

bool WBCHand::timeToSolveDDP(int iteration) {
  time_to_solve_ddp_ = !(iteration % settings_.Nc);
  return time_to_solve_ddp_;
}

void WBCHand::iterate(const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {

  x0_ = designer_.shapeState(q_current, v_current);

  // ~~TIMING~~ //
  if (iteration_ < fullHorizon_.size())
    recedeWithCycle();
    iteration_ ++;
    land_hand_ --;
    takeoff_hand_ --;
    
  // ~~REFERENCES~~ //
  designer_.updateReducedModel(x0_);
  updateStepTrackerReferences();
  // ~~SOLVER~~ //
  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
}

void WBCHand::iterate(int iteration, const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {
  if (timeToSolveDDP(iteration)) {
    iterate(q_current, v_current, is_feasible);
  } else
    x0_ = designer_.shapeState(q_current, v_current);
}

void WBCHand::updateStepTrackerReferences() {
  for (unsigned long time = 0; time < horizon_.size(); time++) {
    horizon_.setTranslationReference(time, "position_LH", ref_LH_pose_);
    horizon_.setTranslationReference(time, "position_RH", ref_RH_pose_);
    horizon_.setForceReference(time,"force_RH", designer_.get_RH_frame().actInv(ref_force_));
  }
}

void WBCHand::recedeWithCycle() {
  horizon_.recede(fullHorizon_.ama(iteration_), fullHorizon_.ada(iteration_));
  return;
}

}  // namespace sobec

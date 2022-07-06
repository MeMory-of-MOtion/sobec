#include "sobec/walk-with-traj/wbc.hpp"

namespace sobec {

WBC::WBC() {}

WBC::WBC(const WBCSettings &settings, const RobotDesigner &design,
         const HorizonManager &horizon, const Eigen::VectorXd &q0,
         const Eigen::VectorXd &v0, const std::string &actuationCostName) {
  initialize(settings, design, horizon, q0, v0, actuationCostName);
}

void WBC::initialize(const WBCSettings &settings, const RobotDesigner &design,
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
  x_internal_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);

  x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
  x0_ << shapeState(q0, v0);
  designer_.updateReducedModel(x0_);
  designer_.updateCompleteModel(q0);

  ref_LF_poses_.reserve(horizon_.size());
  ref_RF_poses_.reserve(horizon_.size());

  for (unsigned long i = 0; i < horizon_.size(); i++) {
    ref_LF_poses_.push_back(designer_.get_LF_frame());
    ref_RF_poses_.push_back(designer_.get_RF_frame());
  }

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

  initializeSupportTiming();
  initialized_ = true;
}

void WBC::generateWalkingCycle(ModelMaker &mm) {
  std::vector<Support> cycle;

  land_LF_cycle_ = 0;
  takeoff_RF_cycle_ = land_LF_cycle_ + settings_.TdoubleSupport;
  land_RF_cycle_ = land_LF_cycle_ + settings_.Tstep;
  takeoff_LF_cycle_ = takeoff_RF_cycle_ + settings_.Tstep;

  for (int i = 0; i < 2 * settings_.Tstep; i++) {
    if (i < takeoff_RF_cycle_)
      cycle.push_back(DOUBLE);
    else if (i < land_RF_cycle_)
      cycle.push_back(LEFT);
    else if (i < takeoff_LF_cycle_)
      cycle.push_back(DOUBLE);
    else
      cycle.push_back(RIGHT);
  }
  std::vector<AMA> cyclicModels = mm.formulateHorizon(cycle);
  HorizonManagerSettings names = {designer_.get_LF_name(),
                                  designer_.get_RF_name()};
  walkingCycle_ = HorizonManager(names, x0_, cyclicModels,
                                 cyclicModels[2 * settings_.Tstep - 1]);
}

void WBC::generateStandingCycle(ModelMaker &mm) {
  ///@todo: bind it
  std::vector<Support> cycle(2 * settings_.Tstep, DOUBLE);
  std::vector<AMA> cyclicModels = mm.formulateHorizon(cycle);
  HorizonManagerSettings names = {designer_.get_LF_name(),
                                  designer_.get_RF_name()};
  standingCycle_ = HorizonManager(names, x0_, cyclicModels,
                                  cyclicModels[2 * settings_.Tstep - 1]);
}

bool WBC::timeToSolveDDP(int iteration) {
  time_to_solve_ddp_ = !(iteration % settings_.Nc);
  return time_to_solve_ddp_;
}

void WBC::iterate(const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {
  x0_ = shapeState(q_current, v_current);
  // ~~TIMING~~ //
  updateSupportTiming();
  recedeWithCycle();

  // ~~REFERENCES~~ //
  designer_.updateReducedModel(x0_);
  switch (settings_.typeOfCommand) {
    case StepTracker:
      //updateStepTrackerLastReference();
      updateStepTrackerReferences();
      break;
    case NonThinking:
      updateNonThinkingReferences();
      break;
    default:
      break;
  }
  // ~~SOLVER~~ //
  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
}

void WBC::iterate(int iteration, const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {
  if (timeToSolveDDP(iteration)) {
    iterate(q_current, v_current, is_feasible);
  } else
    x0_ = shapeState(q_current, v_current);
}

void WBC::updateStepTrackerReferences() {
  for (unsigned long time = 0; time < horizon_.size(); time++) {
    horizon_.setPoseReferenceLF(time, "placement_LF", getPoseRef_LF(time));
    horizon_.setPoseReferenceRF(time, "placement_RF", getPoseRef_RF(time));
    ///@todo: the names must be provided by the user
  }
}

void WBC::updateStepTrackerLastReference() {
  horizon_.setPoseReferenceLF(horizon_.size() - 1, "placement_LF",
                              getPoseRef_LF(horizon_.size() - 1));
  horizon_.setPoseReferenceRF(horizon_.size() - 1, "placement_RF",
                              getPoseRef_RF(horizon_.size() - 1));
  ref_LF_poses_.erase(ref_LF_poses_.begin());
  ref_LF_poses_.push_back(ref_LF_poses_[horizon_.size() - 1]);
  ref_RF_poses_.erase(ref_RF_poses_.begin());
  ref_RF_poses_.push_back(ref_RF_poses_[horizon_.size() - 1]);
}

void WBC::updateNonThinkingReferences() {
  for (unsigned long time = 0; time < horizon_.size(); time++) {
    horizon_.setVelocityRefCOM(time, "comVelocity", ref_com_vel_[time]);
    ///@todo: the names must be provided by the user
  }
}

void WBC::recedeWithCycle() {
  if (now_ == WALKING) {
    recedeWithCycle(walkingCycle_);
  } else if (now_ == STANDING &&
             horizon_.supportSize(horizon_.size() - 1) == 2) {
    recedeWithCycle(standingCycle_);
    if (first_switch_to_stand_) {
      rewindWalkingCycle();
      first_switch_to_stand_ = false;
    }
  } else {
    recedeWithCycle(walkingCycle_);
    first_switch_to_stand_ = true;
  }
  return;
}

void WBC::recedeWithCycle(HorizonManager &cycle) {
  horizon_.recede(cycle.ama(0), cycle.ada(0));
  cycle.recede();
  return;
}

void WBC::rewindWalkingCycle() {
  /** This function brings the walking cycle to the beggining of a single
   * support*/
  for (unsigned long i = 0; i < walkingCycle_.size(); i++) {
    if (horizon_.supportSize(0) == 1 && horizon_.supportSize(1) == 2) {
      walkingCycle_.recede();
      return;
    }
    walkingCycle_.recede();
  }
}

const Eigen::VectorXd &WBC::shapeState(const Eigen::VectorXd &q,
                                       const Eigen::VectorXd &v) {
  if (q.size() == designer_.get_rModelComplete().nq &&
      v.size() == designer_.get_rModelComplete().nv) {
    x_internal_.head<7>() = q.head<7>();
    x_internal_.segment<6>(designer_.get_rModel().nq) = v.head<6>();

    int i = 0;
    for (unsigned long jointID : controlled_joints_id_)
      if (jointID > 1) {
        x_internal_(i + 7) = q(jointID + 5);
        x_internal_(designer_.get_rModel().nq + i + 6) = v(jointID + 4);
        i++;
      }
    return x_internal_;
  } else if (q.size() == designer_.get_rModel().nq &&
             v.size() == designer_.get_rModel().nv) {
    x_internal_ << q, v;
    return x_internal_;
  } else
    throw std::runtime_error(
        "q and v must have the dimentions of the reduced or complete model.");
}

void WBC::initializeSupportTiming() {
  for (unsigned int i = 0; i < horizon_.size() - 1; i++) {
    getSwitches(i, i + 1);
    if (switch_ == LAND_LF)
      land_LF_.push_back(i + 1);
    else if (switch_ == LAND_RF)
      land_RF_.push_back(i + 1);
    else if (switch_ == TAKEOFF_LF)
      takeoff_LF_.push_back(i + 1);
    else if (switch_ == TAKEOFF_RF)
      takeoff_RF_.push_back(i + 1);
  }
}

void WBC::updateSupportTiming() {
  for (unsigned long i = 0; i < land_LF_.size(); i++) land_LF_[i] -= 1;
  for (unsigned long i = 0; i < land_RF_.size(); i++) land_RF_[i] -= 1;
  for (unsigned long i = 0; i < takeoff_LF_.size(); i++) takeoff_LF_[i] -= 1;
  for (unsigned long i = 0; i < takeoff_RF_.size(); i++) takeoff_RF_[i] -= 1;
  
  land_RF_cycle_ -= 1;
  land_LF_cycle_ -= 1;
  takeoff_LF_cycle_ -= 1;
  takeoff_RF_cycle_ -= 1;
  if (land_RF_cycle_ < 0) land_RF_cycle_ = 2 * settings_.Tstep - 1; 
  if (land_LF_cycle_ < 0) land_LF_cycle_ = 2 * settings_.Tstep - 1; 
  if (takeoff_LF_cycle_ < 0) takeoff_LF_cycle_ = 2 * settings_.Tstep - 1; 
  if (takeoff_RF_cycle_ < 0) takeoff_RF_cycle_ = 2 * settings_.Tstep - 1; 

  if (land_LF_.size() > 0 && land_LF_[0] < 0) land_LF_.erase(land_LF_.begin());

  if (land_RF_.size() > 0 && land_RF_[0] < 0) land_RF_.erase(land_RF_.begin());

  if (takeoff_LF_.size() > 0 && takeoff_LF_[0] < 0)
    takeoff_LF_.erase(takeoff_LF_.begin());

  if (takeoff_RF_.size() > 0 && takeoff_RF_[0] < 0)
    takeoff_RF_.erase(takeoff_RF_.begin());

  horizon_end_ = (int)horizon_.size();
  getSwitches(horizon_end_ - 2, horizon_end_ - 1);
  if (switch_ == LAND_LF)
    land_LF_.push_back(horizon_end_ - 1);
  else if (switch_ == LAND_RF)
    land_RF_.push_back(horizon_end_ - 1);
  else if (switch_ == TAKEOFF_LF)
    takeoff_LF_.push_back(horizon_end_ - 1);
  else if (switch_ == TAKEOFF_RF)
    takeoff_RF_.push_back(horizon_end_ - 1);
}

const supportSwitch &WBC::getSwitches(const unsigned long before,
                                      const unsigned long after) {
  contacts_before_ = horizon_.get_contacts(before);
  contacts_after_ = horizon_.get_contacts(after);

  if (contacts_before_ == contacts_after_) {
    switch_ = NO_SWITCH;
    return switch_;
  }
  if (horizon_.supportSize(before) == 2) {
    if (contacts_after_.find(designer_.get_LF_name()) ==
        contacts_after_.end()) {
      switch_ = TAKEOFF_LF;  // case LF not in contact_after
      return switch_;
    }
    switch_ = TAKEOFF_RF;  // case RF not in contact_after
    return switch_;
  }
  if (contacts_before_.find(designer_.get_LF_name()) ==
      contacts_before_.end()) {
    switch_ = LAND_LF;  // case LF not in contact_before
    return switch_;
  }
  switch_ = LAND_RF;  // case RF not in contact_before
  return switch_;
}

}  // namespace sobec

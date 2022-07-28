#include "sobec/walk-with-traj/wbc_nothinking.hpp"

namespace sobec {

WBCNoThinking::WBCNoThinking() {}

WBCNoThinking::WBCNoThinking(const WBCSettings &settings, const RobotDesigner &design,
         const HorizonManager &horizon, const Eigen::VectorXd &q0,
         const Eigen::VectorXd &v0, const std::string &actuationCostName) {
  initialize(settings, design, horizon, q0, v0, actuationCostName);
}

void WBCNoThinking::initialize(const WBCSettings &settings, const RobotDesigner &design,
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
  ref_com_ = designer_.get_com_position();
  ref_com_vel_ = eVector3::Zero();

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

void WBCNoThinking::generateWalkingCycle(ModelMakerNoThinking &mm) {
  std::vector<Support> cycle;
  
  takeoff_RF_cycle_ = settings_.TdoubleSupport;
  land_RF_cycle_ = takeoff_RF_cycle_ + settings_.TsingleSupport;
  takeoff_LF_cycle_ = land_RF_cycle_ + settings_.TdoubleSupport;
  land_LF_cycle_ = takeoff_LF_cycle_ + settings_.TsingleSupport;

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
  std::vector<AMA> cyclicModels;
  cyclicModels = mm.formulateHorizon(cycle);
  
  HorizonManagerSettings names = {designer_.get_LF_name(),
                                  designer_.get_RF_name()};
  walkingCycle_ = HorizonManager(names, x0_, cyclicModels,
                                 cyclicModels[2 * settings_.Tstep - 1]);
}

void WBCNoThinking::generateStandingCycle(ModelMakerNoThinking &mm) {
  ///@todo: bind it
  std::vector<Support> cycle(2 * settings_.Tstep, DOUBLE);
  std::vector<AMA> cyclicModels;
  cyclicModels = mm.formulateHorizon(cycle);
  
  HorizonManagerSettings names = {designer_.get_LF_name(),
                                  designer_.get_RF_name()};
  standingCycle_ = HorizonManager(names, x0_, cyclicModels,
                                  cyclicModels[2 * settings_.Tstep - 1]);
}

void WBCNoThinking::iterate(const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {
  x0_ = shapeState(q_current, v_current);
  // ~~TIMING~~ //
  recedeWithCycle();
  updateSupportTiming();

  // ~~REFERENCES~~ //
  designer_.updateReducedModel(x0_);
  updateNonThinkingReferences();
  // ~~SOLVER~~ //
  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
}

void WBCNoThinking::updateNonThinkingReferences() {
  horizon_.setTerminalPoseCoM("comTask", ref_com_);
  for (unsigned long time = 0; time < horizon_.size(); time++) {
	  horizon_.setVelocityRefCOM(time,"comVelocity",ref_com_vel_);
	  horizon_.setTranslationReference(time, "Z_translation_LF", getPoseRef_LF(time).translation());
	  horizon_.setTranslationReference(time, "Z_translation_RF", getPoseRef_RF(time).translation());
  }
  horizon_.setTerminalTranslationReference("Z_translation_LF", getPoseRef_LF(horizon_.size()).translation());
  horizon_.setTerminalTranslationReference("Z_translation_RF", getPoseRef_RF(horizon_.size()).translation());
    ///@todo: the names must be provided by the user
}

}  // namespace sobec

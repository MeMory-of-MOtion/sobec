#include "sobec/walk-with-traj/wbc_horizon.hpp"

namespace sobec {

WBCHorizon::WBCHorizon() {}

WBCHorizon::WBCHorizon(const WBCHorizonSettings &settings, const RobotDesigner &design,
                       const HorizonManager &horizon, const Eigen::VectorXd &q0,
                       const Eigen::VectorXd &v0, const std::string &actuationCostName) {
  initialize(settings, design, horizon, q0, v0, actuationCostName);
}

void WBCHorizon::initialize(const WBCHorizonSettings &settings, const RobotDesigner &design,
							const HorizonManager &horizon, const Eigen::VectorXd &q0,
						    const Eigen::VectorXd &v0, const std::string &actuationCostName) {
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
  //designer_.updateCompleteModel(q0);
  ref_LF_poses_.reserve(horizon_.size() + 1);
  ref_RF_poses_.reserve(horizon_.size() + 1);
  ref_com_ = designer_.get_com_position();
  ref_com_vel_ = eVector3::Zero();
  ref_base_rotation_ = Eigen::Matrix3d::Identity();
  ref_dcm_ = eVector3::Zero();
  horizon_iteration_ = 0;
  
  for (unsigned long i = 0; i < horizon_.size() + 1; i++) {
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

  initialized_ = true;
}

void WBCHorizon::setForceAlongHorizon() {
	// Set force reference
  Eigen::VectorXd wrench_reference_left(6);
  Eigen::VectorXd wrench_reference_right(6);
  double ref_force;
  double mean_force = settings_.support_force / 2.;
  double max_force = settings_.support_force - settings_.min_force;
  wrench_reference_left << 0, 0, mean_force, 0, 0, 0;
  wrench_reference_right << 0, 0, mean_force, 0, 0, 0;
  
  // Set force reference for first cycle
  for (int i = 0; i < settings_.TdoubleSupport; i++) {
	  ref_force = mean_force * (settings_.TdoubleSupport - i - 1) / static_cast<double>(settings_.TdoubleSupport)
				  + settings_.min_force * (i + 1) / static_cast<double>(settings_.TdoubleSupport);
	  wrench_reference_right[2] = ref_force;
	  wrench_reference_left[2] = settings_.support_force - ref_force;
	  fullHorizon_.setWrenchReference(i,"wrench_LF",wrench_reference_left);
	  fullHorizon_.setWrenchReference(i,"wrench_RF",wrench_reference_right);
  }
  // Set force reference for following cycles
  for (int j = 1; j < settings_.totalSteps; j++) {
	  for (int i = 0; i < settings_.TdoubleSupport; i++) {
		  ref_force = max_force * (settings_.TdoubleSupport - i - 1) / static_cast<double>(settings_.TdoubleSupport)
					  + settings_.min_force * (i + 1) / static_cast<double>(settings_.TdoubleSupport);
		  if (j % 2 == 0){
			  wrench_reference_right[2] = ref_force;
			  wrench_reference_left[2] = settings_.support_force - ref_force;
		  }
		  else {
			  wrench_reference_left[2] = ref_force;
			  wrench_reference_right[2] = settings_.support_force - ref_force;
		  }
		  fullHorizon_.setWrenchReference(i + j * settings_.Tstep,"wrench_LF",wrench_reference_left);
		  fullHorizon_.setWrenchReference(i + j * settings_.Tstep,"wrench_RF",wrench_reference_right);
	  }
  }
  // Set force reference for last cycle
  for (int i = 0; i < settings_.TdoubleSupport; i++) {
	  ref_force = max_force * (settings_.TdoubleSupport - i - 1) / static_cast<double>(settings_.TdoubleSupport)
				  + mean_force * (i + 1) / static_cast<double>(settings_.TdoubleSupport);
	  if (settings_.totalSteps % 2 == 0) {
		  wrench_reference_right[2] = ref_force;
		  wrench_reference_left[2] = settings_.support_force - ref_force;
	  }
	  else {
		  wrench_reference_right[2] = ref_force;
		  wrench_reference_left[2] = settings_.support_force - ref_force;
	  }
	  fullHorizon_.setWrenchReference(i + settings_.totalSteps * settings_.Tstep,"wrench_LF",wrench_reference_left);
	  fullHorizon_.setWrenchReference(i + settings_.totalSteps * settings_.Tstep,"wrench_RF",wrench_reference_right);
  }
}

std::vector<Support> WBCHorizon::generateSupportCycle() {
  std::vector<Support> cycle;
  
  for (int j = 0; j < settings_.totalSteps; j++) {
	if (j % 2 == 0){
	  takeoff_RF_.push_back(j * settings_.Tstep + settings_.TdoubleSupport + settings_.T);
	  land_RF_.push_back((j + 1) * settings_.Tstep + settings_.T);
	}
	else {
	  takeoff_LF_.push_back(j * settings_.Tstep + settings_.TdoubleSupport + settings_.T);
	  land_LF_.push_back((j + 1) * settings_.Tstep + settings_.T);
	}
	for (int i = 0; i < settings_.Tstep; i++) {
	  if (i < settings_.TdoubleSupport)
	    cycle.push_back(DOUBLE);
	  else {
		if (j % 2 == 0)
		  cycle.push_back(LEFT);
		else
		  cycle.push_back(RIGHT);
	  }
	}
  }
  for (int j = 0; j < settings_.T + settings_.TdoubleSupport; j++) {
	cycle.push_back(DOUBLE);  
  }
  return cycle;
}

void WBCHorizon::generateFullHorizon(ModelMaker &mm, const Experiment &experiment) {
  std::vector<Support> cycle = generateSupportCycle(); 
  std::vector<AMA> cyclicModels;
  cyclicModels = mm.formulateHorizon(cycle,experiment);
  HorizonManagerSettings names = {designer_.get_LF_name(),
                                  designer_.get_RF_name()};
  fullHorizon_ = HorizonManager(names, x0_, cyclicModels,
                                 cyclicModels.back());
  setForceAlongHorizon();
}

bool WBCHorizon::timeToSolveDDP(int iteration) {
  time_to_solve_ddp_ = !(iteration % settings_.Nc);
  return time_to_solve_ddp_;
}

void WBCHorizon::iterate(const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {
  x0_ = shapeState(q_current, v_current);

  // ~~TIMING~~ //
  recedeWithCycle();
  updateSupportTiming();

  // ~~REFERENCES~~ //
  designer_.updateReducedModel(x0_);
  //updateStepTrackerLastReference();
  updateStepTrackerReferences();

  // ~~SOLVER~~ //
  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
}

void WBCHorizon::iterate(int iteration, const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {
  if (timeToSolveDDP(iteration)) {
    iterate(q_current, v_current, is_feasible);
  } else
    x0_ = shapeState(q_current, v_current);
}

void WBCHorizon::iterateNoThinking(const Eigen::VectorXd &q_current,
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

void WBCHorizon::iterateNoThinking(int iteration, const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {
  if (timeToSolveDDP(iteration)) {
    iterateNoThinking(q_current, v_current, is_feasible);
  } else
    x0_ = shapeState(q_current, v_current);
}

void WBCHorizon::iterateNoThinkingWithDelay(const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool contact_left, bool contact_right, bool is_feasible) {
  x0_ = shapeState(q_current, v_current);
  designer_.updateReducedModel(x0_);
  
  if (land_LF_.size() > 0) {
	  if(land_LF_[0] == 0 and not(contact_left)) {
		  //std::cout << "delay left contact" << std::endl;
		  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
		  return;
	  }
	  else if (contact_left and contact_right and land_LF_[0] < settings_.TsingleSupport / 2) {
		  //std::cout << "go to next double left" << std::endl;
		  goToNextDoubleSupport();
		  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
		  return;
	  }
  }
  if (land_RF_.size() > 0) {
	  if(land_RF_[0] == 0 and not(contact_right)) {
		  //std::cout << "delay right contact" << std::endl;
		  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
		  return;
	  }
	  else if (contact_left and contact_right and land_RF_[0] < settings_.TsingleSupport / 2) {
		  //std::cout << "go to next double right" << std::endl;
		  goToNextDoubleSupport();
		  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
		  return;
	  }
  }
  //std::cout << "normal execution" << std::endl;
  recedeWithCycle();
  updateSupportTiming();

  // ~~REFERENCES~~ //
  updateNonThinkingReferences();

  // ~~SOLVER~~ //
  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
}

void WBCHorizon::updateStepTrackerReferences() {
  for (unsigned long time = 0; time < horizon_.size(); time++) {
    horizon_.setPoseReference(time, "placement_LF", getPoseRef_LF(time));
    horizon_.setPoseReference(time, "placement_RF", getPoseRef_RF(time));
    ///@todo: the names must be provided by the user
  }
  horizon_.setTerminalPoseReference("placement_LF", getPoseRef_LF(horizon_.size()));
  horizon_.setTerminalPoseReference("placement_RF", getPoseRef_RF(horizon_.size()));

  if (horizon_.contacts(horizon_.size() - 1)->getContactStatus(designer_.get_LF_name()) and horizon_.contacts(horizon_.size() - 1)->getContactStatus(designer_.get_RF_name())) {
	  ref_dcm_ = (getPoseRef_LF(horizon_.size()).translation() + getPoseRef_RF(horizon_.size()).translation()) / 2;
  }
  else if (horizon_.contacts(horizon_.size() - 1)->getContactStatus(designer_.get_LF_name())) {
	  ref_dcm_ = getPoseRef_LF(horizon_.size()).translation();
  } 
  else if (horizon_.contacts(horizon_.size() - 1)->getContactStatus(designer_.get_RF_name())) {
	  ref_dcm_ = getPoseRef_RF(horizon_.size()).translation();
  }
  ref_dcm_[2] = 0.87;
  horizon_.setTerminalDCMReference("DCM", ref_dcm_);
}

void WBCHorizon::updateStepTrackerLastReference() {
  horizon_.setPoseReference(horizon_.size() - 1, "placement_LF",
                              getPoseRef_LF(horizon_.size() - 1));
  horizon_.setPoseReference(horizon_.size() - 1, "placement_RF",
                              getPoseRef_RF(horizon_.size() - 1));
  horizon_.setTerminalPoseReference("placement_LF",
                              getPoseRef_LF(horizon_.size()));
  horizon_.setTerminalPoseReference("placement_RF",
                              getPoseRef_RF(horizon_.size()));
  ref_LF_poses_.erase(ref_LF_poses_.begin());
  ref_LF_poses_.push_back(ref_LF_poses_[horizon_.size() - 1]);
  ref_RF_poses_.erase(ref_RF_poses_.begin());
  ref_RF_poses_.push_back(ref_RF_poses_[horizon_.size() - 1]);
}

void WBCHorizon::updateNonThinkingReferences() {
  horizon_.setTerminalPoseCoM("comTask", ref_com_);
  for (unsigned long time = 0; time < horizon_.size(); time++) {
	  horizon_.setVelocityRefCOM(time,"comVelocity",ref_com_vel_);
	  horizon_.setTranslationReference(time, "translation_LF", getPoseRef_LF(time).translation());
	  horizon_.setTranslationReference(time, "translation_RF", getPoseRef_RF(time).translation());
	  //horizon_.setRotationReference(time, "rotation_LF", designer_.get_root_frame().rotation());
	  //horizon_.setRotationReference(time, "rotation_RF", designer_.get_root_frame().rotation());
  }
  horizon_.setTerminalTranslationReference("translation_LF", getPoseRef_LF(horizon_.size()).translation());
  horizon_.setTerminalTranslationReference("translation_RF", getPoseRef_RF(horizon_.size()).translation());
  //horizon_.setTerminalRotationReference("rotation_base",ref_base_rotation_);
  //horizon_.setTerminalRotationReference("rotation_LF", ref_base_rotation_);
  //horizon_.setTerminalRotationReference("rotation_RF", ref_base_rotation_);

  if (horizon_.contacts(horizon_.size() - 1)->getContactStatus(designer_.get_LF_name()) and horizon_.contacts(horizon_.size() - 1)->getContactStatus(designer_.get_RF_name())) {
	  ref_dcm_ = (getPoseRef_LF(horizon_.size()).translation() + getPoseRef_RF(horizon_.size()).translation()) / 2;
  }
  else if (horizon_.contacts(horizon_.size() - 1)->getContactStatus(designer_.get_LF_name())) {
	  ref_dcm_ = getPoseRef_LF(horizon_.size()).translation();
  } 
  else if (horizon_.contacts(horizon_.size() - 1)->getContactStatus(designer_.get_RF_name())) {
	  ref_dcm_ = getPoseRef_RF(horizon_.size()).translation();
  }
  ref_dcm_[2] = 0.87;
  horizon_.setTerminalDCMReference("DCM", ref_dcm_);
}

void WBCHorizon::recedeWithCycle() {
  if (horizon_iteration_ < fullHorizon_.size()) {
    horizon_.recede(fullHorizon_.ama(horizon_iteration_), fullHorizon_.ada(horizon_iteration_));
    horizon_iteration_ ++;
  }
  else
    horizon_.recede();
}

void WBCHorizon::goToNextDoubleSupport() {
	while (horizon_.supportSize(0) != 2 and horizon_iteration_ < fullHorizon_.size()) {
		horizon_.recede(fullHorizon_.ama(horizon_iteration_), fullHorizon_.ada(horizon_iteration_));
		horizon_iteration_ ++;
		updateSupportTiming();
	}
	horizon_.recede(fullHorizon_.ama(horizon_iteration_), fullHorizon_.ada(horizon_iteration_));
	horizon_iteration_ ++;
	updateSupportTiming();
}

const Eigen::VectorXd &WBCHorizon::shapeState(const Eigen::VectorXd &q,
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

void WBCHorizon::updateSupportTiming() {
  for (unsigned long i = 0; i < land_LF_.size(); i++) land_LF_[i] -= 1;
  for (unsigned long i = 0; i < land_RF_.size(); i++) land_RF_[i] -= 1;
  for (unsigned long i = 0; i < takeoff_LF_.size(); i++) takeoff_LF_[i] -= 1;
  for (unsigned long i = 0; i < takeoff_RF_.size(); i++) takeoff_RF_[i] -= 1;

  if (land_LF_.size() > 0 && land_LF_[0] < 0) land_LF_.erase(land_LF_.begin());

  if (land_RF_.size() > 0 && land_RF_[0] < 0) land_RF_.erase(land_RF_.begin());

  if (takeoff_LF_.size() > 0 && takeoff_LF_[0] < 0)
    takeoff_LF_.erase(takeoff_LF_.begin());

  if (takeoff_RF_.size() > 0 && takeoff_RF_[0] < 0)
    takeoff_RF_.erase(takeoff_RF_.begin());
}

}  // namespace sobec

#include "sobec/wbc.hpp"

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
  std::cout << x0_.size() << std::endl;
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
    xs_init.push_back(x0_);
    us_init.push_back(zero_u);
    horizon_.setBalancingTorque(i, actuationCostName, x0_);  // sets ref torqs.
  }
  xs_init.push_back(x0_);

  horizon_.get_ddp()->solve(xs_init, us_init, 500, false);

  // timming
  t_takeoff_RF_.setLinSpaced(
      settings_.horizonSteps, 0, 2 * settings_.horizonSteps * settings_.Tstep);
  t_takeoff_RF_.array() += (int)settings_.T;
  t_takeoff_LF_ = t_takeoff_RF_.array() + settings_.Tstep;
  t_land_RF_ = t_takeoff_RF_.array() + settings_.TsingleSupport;
  t_land_LF_ = t_takeoff_LF_.array() + settings_.TsingleSupport;

  initialized_ = true;
}

void WBC::generateWalkingCycle(ModelMaker &mm) {
  std::vector<Support> cycle;
  int takeoff_RF, land_RF, takeoff_LF, land_LF;
  takeoff_RF = 0;
  land_RF = takeoff_RF + settings_.TsingleSupport;
  takeoff_LF = takeoff_RF + settings_.Tstep;
  land_LF = takeoff_LF + settings_.TsingleSupport;

  for (int i = 0; i < 2 * settings_.Tstep; i++) {
    if (i < land_RF)
      cycle.push_back(LEFT);
    else if (i < takeoff_LF)
      cycle.push_back(DOUBLE);
    else if (i < land_LF)
      cycle.push_back(RIGHT);
    else
      cycle.push_back(DOUBLE);
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

void WBC::updateStepCycleTiming() {
  t_takeoff_RF_.array() -= 1;
  t_takeoff_LF_.array() -= 1;
  t_land_RF_.array() -= 1;
  t_land_LF_.array() -= 1;

  if (t_land_LF_(0) < 0) t_land_LF_.array() += 2 * settings_.Tstep;
  if (t_land_RF_(0) < 0) t_land_RF_.array() += 2 * settings_.Tstep;
  if (t_takeoff_LF_(0) < 0) t_takeoff_LF_.array() += 2 * settings_.Tstep;
  if (t_takeoff_LF_(0) < 0) t_takeoff_LF_.array() += 2 * settings_.Tstep;
}

bool WBC::timeToSolveDDP(const int &iteration) {
  return !(iteration % settings_.Nc);
}

void WBC::setDesiredFeetPoses(const int &/*iteration*/, const int &/*time*/)
{
  throw std::runtime_error("void WBC::setDesiredFeetPoses(const int &iteration, const int &time) is not implemented!!!");
}

Eigen::VectorXd WBC::iterate(const int &iteration,
                             const Eigen::VectorXd &q_current,
                             const Eigen::VectorXd &v_current,
                             const bool &is_feasible) {
  x0_ = shapeState(q_current, v_current);
  if (timeToSolveDDP(iteration)) {
    // ~~TIMING~~ //
    updateStepCycleTiming();
    recedeWithCycle();

    // ~~REFERENCES~~ //
    designer_.updateReducedModel(x0_);
    switch (settings_.typeOfCommand) {
      case StepTracker:
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
  return horizon_.currentTorques(x0_);
}

void WBC::updateStepTrackerReferences() {
  for (unsigned long time = 0; time < horizon_.size(); time++) {
    horizon_.setPoseReferenceLF(time, "placement_LF", getPoseRef_LF(time));
    horizon_.setPoseReferenceRF(time, "placement_RF", getPoseRef_RF(time));
    ///@todo: the names must be provided by the user
  }
}

void WBC::updateNonThinkingReferences() {
  for (unsigned long time = 0; time < horizon_.size(); time++) {
    horizon_.setVelocityRefCOM(time, "comVelocity", ref_com_vel_[time]);
    ///@todo: the names must be provided by the user
  }
}

void WBC::recedeWithCycle() {
  ///@todo: We switch from walking to standing at the beggining of the double
  /// support stage.
  /// We can evaluate later the possibility of resetting the walking cycle, to
  /// start at the end of the beginning of a single support when we switch back
  /// to walking.
  if (now_ == WALKING) {
    recedeWithCycle(walkingCycle_);
  } else if (now_ == STANDING &&
             horizon_.contacts(horizon_.size() - 1)->get_active_set().size() ==
                 2) {
    recedeWithCycle(standingCycle_);
  } else {
    recedeWithCycle(walkingCycle_);
  }
  return;
}

void WBC::recedeWithCycle(HorizonManager &cycle) {
  horizon_.recede(cycle.ama(0), cycle.ada(0));
  cycle.recede();
  return;
}

Eigen::VectorXd WBC::shapeState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {

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
}  // namespace sobec

#include "sobec/walk-with-traj/wbc_hand.hpp"

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
  x_internal_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);

  x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
  x0_ << shapeState(q0, v0);
  designer_.updateReducedModel(x0_);
  designer_.updateCompleteModel(q0);
  ref_hand_pose_ = designer_.get_EndEff_frame().translation();
  ref_com_ = designer_.get_com_position();

  // horizon settings
  std::vector<Eigen::VectorXd> xs_init;
  std::vector<Eigen::VectorXd> us_init;
  Eigen::VectorXd zero_u = Eigen::VectorXd::Zero(designer_.get_rModel().nv - 6);

  for (std::size_t i = 0; i < horizon_.size(); i++) {
    ///@todo: Remove this from the initialization and provide it as a method.
    xs_init.push_back(x0_);
    us_init.push_back(zero_u);
    horizon_.setBalancingTorque(i, actuationCostName, x0_);
    horizon_.changeCostStatus(i,"gripperPosition",false);
  }
  xs_init.push_back(x0_);
  horizon_.changeTerminalCostStatus("gripperPosition",false);

  horizon_.get_ddp()->solve(xs_init, us_init, 100, false);
  
  iteration_ = 0;
  initialized_ = true;
}

void WBCHand::generateFullHorizon(ModelMaker &mm) {
  std::vector<AMA> cyclicModels;
  
  for (int i = 0; i < settings_.Tduration; i++) {
	  cyclicModels.push_back(mm.formulateColFullTask());
  }
  HorizonManagerSettings names = {designer_.get_LF_name(),
                                  designer_.get_RF_name()};
  fullHorizon_ = HorizonManager(names, x0_, cyclicModels,
                                 cyclicModels.back());
  for (int i = settings_.Tduration - settings_.T; i < settings_.Tduration; i++) {
	  fullHorizon_.changeCostStatus(i,"gripperPosition",false);
  }
}

bool WBCHand::timeToSolveDDP(int iteration) {
  time_to_solve_ddp_ = !(iteration % settings_.Nc);
  return time_to_solve_ddp_;
}

void WBCHand::iterate(const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {
  x0_ = shapeState(q_current, v_current);

  // ~~TIMING~~ //
  if (iteration_ < fullHorizon_.size())
    recedeWithCycle();
    iteration_ ++;
    
  // ~~REFERENCES~~ //
  designer_.updateReducedModel(x0_);
  updateTrackerReferences();
  // ~~SOLVER~~ //
  horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
}

void WBCHand::iterateWithMemory(const std::vector<Eigen::VectorXd> &xs,
                                const std::vector<Eigen::VectorXd> &us, 
                                bool is_feasible) {
  x0_ = xs[0];

  // ~~TIMING~~ //
  if (iteration_ < fullHorizon_.size())
    recedeWithCycle();
    iteration_ ++;
    
  // ~~REFERENCES~~ //
  designer_.updateReducedModel(x0_);
  updateTrackerReferences();
  // ~~SOLVER~~ //
  horizon_.solveWithWarmStart(xs, us, settings_.ddpIteration, is_feasible);
}

void WBCHand::iterate(int iteration, const Eigen::VectorXd &q_current,
                  const Eigen::VectorXd &v_current, bool is_feasible) {
  if (timeToSolveDDP(iteration)) {
    iterate(q_current, v_current, is_feasible);
  } else
    x0_ = shapeState(q_current, v_current);
}

void WBCHand::updateTrackerReferences() {
  for (unsigned long time = 0; time < horizon_.size(); time++) {
    horizon_.setTranslationReference(time, "gripperPosition", ref_hand_pose_);
  }
  horizon_.setTerminalTranslationReference("gripperPosition", ref_hand_pose_);
}

void WBCHand::recedeWithCycle() {
  horizon_.recede(fullHorizon_.ama(iteration_), fullHorizon_.ada(iteration_));
  return;
}

const Eigen::VectorXd &WBCHand::shapeState(const Eigen::VectorXd &q,
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

}  // namespace sobec

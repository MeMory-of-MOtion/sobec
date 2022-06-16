#include "sobec/ocp.hpp"

namespace sobec {

OCP::OCP() {}

OCP::OCP(const OCPSettings &settings, const ModelMakerSettings &model_settings, const RobotDesignerSettings &design,
         const Eigen::VectorXd &q0, const Eigen::VectorXd &v0) {
  initialize(settings, model_settings, design, q0, v0);
}

void OCP::initialize(const OCPSettings &settings, const ModelMakerSettings &model_settings,
                     const RobotDesignerSettings &design, const Eigen::VectorXd &q0, const Eigen::VectorXd &v0) {
  OCP_settings_ = settings;
  designer_ = sobec::RobotDesigner(design);
  modelMaker_ = sobec::ModelMaker(model_settings, designer_);

  std::vector<Support> supports(OCP_settings_.T, Support::DOUBLE);
  std::vector<AMA> runningModels = modelMaker_.formulateHorizon(supports);
  AMA terminalModel = modelMaker_.formulateStepTracker(Support::DOUBLE);

  xc_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
  xc_ << q0, v0;

  std::cout << "left contact name = " << designer_.get_LF_name() << std::endl;
  sobec::HorizonManagerSettings horizonSettings = {designer_.get_LF_name(), designer_.get_RF_name()};
  horizon_ = sobec::HorizonManager(horizonSettings, xc_, runningModels, terminalModel);

  std::cout << "horizon left contact is " << horizon_.contacts(0)->getContactStatus(designer_.get_LF_name())
            << std::endl;

  std::vector<Eigen::VectorXd> x_init;
  std::vector<Eigen::VectorXd> u_init;
  Eigen::VectorXd zero_u = Eigen::VectorXd::Zero(designer_.get_rModel().nv - 6);

  for (std::size_t i = 0; i < OCP_settings_.T; i++) {
    x_init.push_back(xc_);
    u_init.push_back(zero_u);
  }
  x_init.push_back(xc_);

  horizon_.get_ddp()->solve(x_init, u_init, 500, false);

  designer_.updateReducedModel(q0);

  // Initialize first foot trajectory
  starting_position_right_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_RF_id()]);
  final_position_right_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_RF_id()]);
  final_position_right_.translation()[0] += OCP_settings_.stepSize;
  final_position_right_.translation()[1] -= OCP_settings_.stepYCorrection;

  starting_position_left_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_LF_id()]);
  final_position_left_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_LF_id()]);
  final_position_left_.translation()[0] += OCP_settings_.stepSize * 2;
  final_position_left_.translation()[1] += OCP_settings_.stepYCorrection;

  swing_trajectory_left_ = std::make_shared<sobec::FootTrajectory>(OCP_settings_.stepHeight, OCP_settings_.stepDepth);
  swing_trajectory_right_ = std::make_shared<sobec::FootTrajectory>(OCP_settings_.stepHeight, OCP_settings_.stepDepth);
  swing_trajectory_left_->generate(0., OCP_settings_.TsimpleSupport * OCP_settings_.Dt, starting_position_left_,
                                   final_position_left_);
  swing_trajectory_right_->generate(0., OCP_settings_.TsimpleSupport * OCP_settings_.Dt, starting_position_right_,
                                    final_position_right_);

  // Initialize varying parameters
  TswitchPhase_ = OCP_settings_.Tstep;
  TswitchTraj_ = OCP_settings_.Tstep + OCP_settings_.T + 5;
  swingRightPhase_ = true;
  swingRightTraj_ = false;
  steps_ = 0;

  double Mg = -designer_.getRobotMass() * model_settings.gravity(2);

  wrench_reference_double_ << 0, 0, Mg / 2., 0, 0, 0;
  wrench_reference_simple_ << 0, 0, Mg, 0, 0, 0;

  // Initialize the whole sequence of contacts
  std::vector<unsigned long> simple_contacts;
  for (std::size_t i = 0; i < OCP_settings_.TsimpleSupport; i++) {
    simple_contacts.push_back(i + 1);
  }

  std::vector<unsigned long> double_contacts(OCP_settings_.TdoubleSupport, 0);

  for (std::size_t i = 0; i < OCP_settings_.totalSteps; i++) {
    contacts_sequence_.insert(contacts_sequence_.end(), double_contacts.begin(), double_contacts.end());
    contacts_sequence_.insert(contacts_sequence_.end(), simple_contacts.begin(), simple_contacts.end());
  }
  std::vector<unsigned long> end_contacts(OCP_settings_.T, 0);
  contacts_sequence_.insert(contacts_sequence_.end(), end_contacts.begin(), end_contacts.end());
}

void OCP::updateEndPhase() {
  // If this is the end of a phase, update phase
  if (TswitchPhase_ == 0) {
    TswitchPhase_ = OCP_settings_.Tstep;
    swingRightPhase_ = not(swingRightPhase_);
    std::cout << "TswitchPhase = 0" << std::endl;
  }
  // If this is the end of a step, update next foot trajectory
  if (TswitchTraj_ == 0) {
    steps_ += 1;
    if (swingRightTraj_) {
      starting_position_left_ = designer_.get_rData().oMf[designer_.get_LF_id()];
      final_position_left_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_LF_id()]);
      final_position_left_.translation()[0] += OCP_settings_.stepSize;
      final_position_left_.translation()[1] += OCP_settings_.stepYCorrection;
      swing_trajectory_left_.reset(new sobec::FootTrajectory(OCP_settings_.stepHeight, OCP_settings_.stepDepth));
      swing_trajectory_left_->generate(0., OCP_settings_.TsimpleSupport * OCP_settings_.Dt, starting_position_left_,
                                       final_position_left_);

      std::cout << "TswitchTraj = 0 update left traj" << std::endl;
    } else {
      starting_position_right_ = designer_.get_rData().oMf[designer_.get_RF_id()];
      final_position_right_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_RF_id()]);
      final_position_right_.translation()[0] += OCP_settings_.stepSize;
      final_position_right_.translation()[1] -= OCP_settings_.stepYCorrection;
      swing_trajectory_right_.reset(new sobec::FootTrajectory(OCP_settings_.stepHeight, OCP_settings_.stepDepth));
      swing_trajectory_right_->generate(0., OCP_settings_.TsimpleSupport * OCP_settings_.Dt, starting_position_right_,
                                        final_position_right_);

      std::cout << "TswitchTraj = 0 update right traj" << std::endl;
    }

    TswitchTraj_ = OCP_settings_.Tstep + 5;
    swingRightTraj_ = not(swingRightTraj_);
  }
}

void OCP::updateOCP(const Eigen::VectorXd &qc, const Eigen::VectorXd &vc) {
  designer_.updateReducedModel(qc);
  xc_ << qc, vc;
  if (!contacts_sequence_.empty()) {
    TswitchTraj_--;
    TswitchPhase_--;
    // Take first action model
    // std::cout << "Contact sequence " << contacts_sequence_[0] << " at Tswitch
    // " << Tswitch_ << " and iteration " << iteration_ << std::endl;
    // If contacts_sequence[0] > 0 , this is a swing phase
    if (contacts_sequence_[0] > 0) {
      std::cout << "Simple support phase with TswitchTraj = " << TswitchTraj_
                << ", and TswitchPhase = " << TswitchPhase_ << std::endl;
      // Get desired foot reference for the end of the horizon
      if (swingRightPhase_) {
        starting_position_right_ =
            swing_trajectory_right_->compute(static_cast<double>(contacts_sequence_[0]) * OCP_settings_.Dt);
        horizon_.setSwingingRF(0, starting_position_right_, starting_position_left_, wrench_reference_simple_);
      } else {
        starting_position_left_ =
            swing_trajectory_left_->compute(static_cast<double>(contacts_sequence_[0]) * OCP_settings_.Dt);
        horizon_.setSwingingLF(0, starting_position_right_, starting_position_left_, wrench_reference_simple_);
      }
    }
    // else, this is a double support phase
    else {
      std::cout << "Double support phase with TswitchTraj = " << TswitchTraj_
                << ", and TswitchPhase = " << TswitchPhase_ << std::endl;
      horizon_.setDoubleSupport(0, starting_position_right_, starting_position_left_, wrench_reference_double_);
    }
    updateEndPhase();
    // Put first model in last position
    horizon_.recede();

    // Update contact sequence
    contacts_sequence_.erase(contacts_sequence_.begin());
  }
  // Solve ddp
  horizon_.solve(xc_, OCP_settings_.ddpIteration);
}
}  // namespace sobec

#include <iostream>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sobec/fwd.hpp>
#include <sobec/mpc-walk.hpp>
#include <sobec/ocp-walk.hpp>
#include <sobec/py2cpp.hpp>

#include "crocoddyl/core/utils/callbacks.hpp"
#include "mpc-walk-automaticallygeneratedinit.hpp"

int main() {
  using namespace sobec;
  using namespace crocoddyl;

  // --- LOAD FROM URDF+SRDF
  // Load full Talos model
  const std::string urdf =
      "/opt/openrobots/share/example-robot-data/robots/talos_data/robots/"
      "talos_reduced.urdf";
  const std::string srdf =
      "/opt/openrobots/share/example-robot-data/robots/talos_data/srdf/"
      "talos.srdf";
  auto fullmodel = boost::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf, pinocchio::JointModelFreeFlyer(),
                              *fullmodel);

  // -- REDUCED MODEL
  // Choose to lock the upper body
  std::vector<pinocchio::JointIndex> jointToLock_ids =
      // talos 14
      {14, 15, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 28, 29, 30, 31, 32, 33};
  // talos 12
  //{14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
  // 33};
  std::cout << "I am going to lock the following joints: " << std::endl;
  for (pinocchio::JointIndex i : jointToLock_ids) {
    std::cout << i << " => " << fullmodel->names[i] << std::endl;
  }
  pinocchio::srdf::loadReferenceConfigurations(*fullmodel, srdf);
  const Eigen::VectorXd q0 = fullmodel->referenceConfigurations["half_sitting"];
  // Build new model by locking joints
  auto model = boost::make_shared<pinocchio::Model>();
  pinocchio::buildReducedModel(*fullmodel, jointToLock_ids, q0, *model);
  pinocchio::srdf::loadReferenceConfigurations(*model, srdf);

  // --- ROBOT WRAPPER
  auto robot = boost::make_shared<sobec::OCPRobotWrapper>(model, "sole_link",
                                                          "half_sitting");
  if (!checkAutomaticallyGeneratedCodeCompatibility(robot)) {
    std::cout << "Robot dimension not compatible with param file. "
              << std::endl;
    std::cout << ">>> The robot build here as nq=" << robot->model->nq
              << " and nv=" << robot->model->nv << "." << std::endl;
    std::cout << ">>> Robot name is '" << robot->model->name << "'."
              << std::endl;
    std::cout << "Exit!" << std::endl;
    std::exit(-1);
  }

  // --- PARAMS
  std::cout << "Initializing params for nv = " << model->nv << std::endl;
  auto params = boost::make_shared<sobec::OCPWalkParams>();
  initParamsFromAutomaticallyGeneratedCode(params);
  auto mpcparams = boost::make_shared<MPCWalkParams>();
  initMPCFromAutomaticallyGeneratedCode(mpcparams);

  // --- CONTACT PATTERN
  std::cout << "Create contact pattern" << std::endl;
  Eigen::MatrixXd patternStart(2, mpcparams->Tstart);
  patternStart.fill(1);
  Eigen::MatrixXd patternDouble(2, mpcparams->Tdouble);
  patternDouble.fill(1);
  Eigen::MatrixXd patternEnd(2,
                             mpcparams->Tend + 1);  // +1 for the terminal node
  patternEnd.fill(1);
  Eigen::MatrixXd patternLeft(2, mpcparams->Tsingle);
  patternLeft.topRows<1>().fill(0);
  patternLeft.bottomRows<1>().fill(1);
  Eigen::MatrixXd patternRight(2, mpcparams->Tsingle);
  patternRight.topRows<1>().fill(1);
  patternRight.bottomRows<1>().fill(0);

  int T = mpcparams->Tstart + mpcparams->Tdouble * 3 + mpcparams->Tsingle * 2 +
          mpcparams->Tend + 1;
  Eigen::MatrixXd contactPattern(2, T);
  contactPattern << patternStart, patternDouble, patternLeft, patternDouble,
      patternRight, patternDouble, patternEnd;
  std::cout << "Contact pattern = \n"
            << contactPattern.transpose() << std::endl;

  // --- OCP
  std::cout << "Init OCP" << std::endl;
  auto ocp = boost::make_shared<OCPWalk>(robot, params, contactPattern);
  ocp->buildSolver();
  std::cout << "Build guess" << std::endl;

  // std::vector<Eigen::VectorXd> x0s, u0s;
  // const auto models = ocp->problem->get_runningModels();
  // const auto datas = ocp->problem->get_runningDatas();
  // for (int i = 0; i < ocp->problem->get_T() ; i++) {
  //   x0s.push_back(ocp->problem->get_x0());
  //   u0s.push_back(models[i]->quasiStatic_x(datas[i],
  //   ocp->problem->get_x0()));
  // }

  auto guess = ocp->buildInitialGuess();
  std::cout << "X ... " << std::endl;
  auto x0s = guess.first;
  for (auto x : x0s) std::cout << x.transpose() << std::endl;
  std::cout << "U ... " << std::endl;
  auto u0s = guess.second;
  for (auto u : u0s) std::cout << u.transpose() << std::endl;
  ocp->solver->setCallbacks({boost::make_shared<crocoddyl::CallbackVerbose>()});
  ocp->solver->solve(guess.first, guess.second);

  // --- MPC
  auto mpc = boost::make_shared<MPCWalk>(mpcparams, ocp->problem);

  std::cout << "Init warm start" << std::endl;
  std::vector<Eigen::VectorXd> xs, us;
  for (int t = 0; t < mpcparams->Tmpc; ++t) {
    xs.push_back(ocp->solver->get_xs()[t]);
    us.push_back(ocp->solver->get_us()[t]);
  }
  xs.push_back(ocp->solver->get_xs()[mpcparams->Tmpc]);
  mpc->initialize(xs, us);
  mpc->solver->setCallbacks({boost::make_shared<crocoddyl::CallbackVerbose>()});

  std::cout << "Start the mpc loop" << std::endl;
  Eigen::VectorXd x = robot->x0;

  for (int t = 1; t <= 100; t++) {
    std::cout << "=== " << t << " === " << std::endl;
    mpc->calc(x, t);
    x = mpc->solver->get_xs()[1];
  }
}

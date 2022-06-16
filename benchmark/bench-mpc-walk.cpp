#include <iostream>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sobec/fwd.hpp>
#include <sobec/mpc-walk.hpp>
#include <sobec/ocp-walk.hpp>
#include <sobec/py2cpp.hpp>
#include "mpc-walk-automaticallygeneratedinit.hpp"

int main() {
  using namespace sobec;
  using namespace crocoddyl;

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

  // Reduce it
  // buildReducedModel(const ModelTpl<Scalar,Options,JointCollectionTpl> &
  // model,
  //                   std::vector<JointIndex> list_of_joints_to_lock,
  //                   const Eigen::MatrixBase<ConfigVectorType> &
  //                   reference_configuration,
  //                   ModelTpl<Scalar,Options,JointCollectionTpl> &
  //                   reduced_model);

  std::vector<pinocchio::JointIndex> jointToLock_ids =
  // talos 14
    { 14, 15, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 28, 29, 30, 31, 32, 33 };
  //talos 12
  //{14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33};

  std::cout << "I am going to lock the following joints: " << std::endl;
  for (pinocchio::JointIndex i : jointToLock_ids) {
    std::cout << i << " => " << fullmodel->names[i] << std::endl;
  }
  // TODO: read q0 from SRDF
  // Eigen::VectorXd q0(fullmodel->nq);
  // q0 << 0, 0, 1.01927e+00, 0, 0
  //   , 0, 1.00000e+00, 0, 0, -4.11354e-01
  //   , 8.59395e-01, -4.48041e-01, -1.70800e-03, 0, 0
  //   , -4.11354e-01, 8.59395e-01, -4.48041e-01, -1.70800e-03, 0
  //   , 6.76100e-03, 2.58470e-01, 1.73046e-01, -2.00000e-04, -5.25366e-01
  //   , 0, 0, 1.00000e-01, 0, -2.58470e-01
  //   , -1.73046e-01, 2.00000e-04, -5.25366e-01, 0, 0
  //   , 1.00000e-01, 0, 0, 0;

  pinocchio::srdf::loadReferenceConfigurations(*fullmodel, srdf);
  const Eigen::VectorXd q0 = fullmodel->referenceConfigurations["half_sitting"];
  std::cout << "Config q0 = " << q0 << std::endl;

  auto model = boost::make_shared<pinocchio::Model>();
  pinocchio::buildReducedModel(*fullmodel, jointToLock_ids, q0, *model);
  pinocchio::srdf::loadReferenceConfigurations(*model, srdf);

  // Robot and Params
  auto robot = boost::make_shared<sobec::OCPRobotWrapper>(model, "sole_link",
                                                          "half_sitting");
  if(!checkAutomaticallyGeneratedCodeCompatibility(robot))
    {
      std::cout << "Robot dimension not compatible with param file. " << std::endl;
      std::cout << ">>> The robot build here as nq=" << robot->model->nq
                << " and nv=" << robot->model->nv << "." << std::endl;
      std::cout << ">>> Robot name is '" << robot->model->name << "'." << std::endl;
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
  Eigen::MatrixXd patternEnd(2, mpcparams->Tend);
  patternEnd.fill(1);
  Eigen::MatrixXd patternLeft(2, mpcparams->Tsingle);
  patternLeft.topRows<1>().fill(0);
  patternLeft.bottomRows<1>().fill(1);
  Eigen::MatrixXd patternRight(2, mpcparams->Tsingle);
  patternRight.topRows<1>().fill(1);
  patternRight.bottomRows<1>().fill(0);

  int T = mpcparams->Tstart + mpcparams->Tdouble * 3 + mpcparams->Tsingle * 2 + mpcparams->Tend;
  Eigen::MatrixXd contactPattern(2, T);
  contactPattern << patternStart, patternDouble, patternLeft, patternDouble,
    patternRight, patternDouble, patternEnd;
  std::cout << "Contact pattern = \n"
            << contactPattern.transpose() << std::endl;

  // --- OCP
  std::cout << "Init OCP" << std::endl;
  auto ocp = boost::make_shared<OCPWalk>(robot, params, contactPattern);
  ocp->buildSolver();

  // --- MPC
  auto mpc = boost::make_shared<MPCWalk>(mpcparams,ocp->problem);

  std::cout << "Init warm start" << std::endl;
  std::vector<Eigen::VectorXd> xs, us;
  for (int t = 0; t < mpcparams->Tmpc; ++t) {
    xs.push_back(ocp->solver->get_xs()[t]);
    us.push_back(ocp->solver->get_us()[t]);
  }
  xs.push_back(ocp->solver->get_xs()[mpcparams->Tmpc]);
  mpc->initialize(xs, us);

  std::cout << "Start the mpc loop" << std::endl;
  Eigen::VectorXd x = robot->x0;

  for (int t = 1; t <= 100; t++) {
    std::cout << "=== " << t << " === " << std::endl;
    mpc->calc(x, t);
    x = mpc->solver->get_xs()[1];
  }
}

#include <iostream>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sobec/fwd.hpp>
#include <sobec/mpc-walk.hpp>
#include <sobec/ocp-walk.hpp>
#include <sobec/py2cpp.hpp>
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

  std::vector<pinocchio::JointIndex> jointToLock_ids
    =
  // talos 14
  // { 14, 15, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 28, 29, 30, 31, 32, 33 };
  // talos 12
    {14,  15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33};

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
  auto params = boost::make_shared<sobec::OCPWalkParams>();

  std::cout << "Initializing params for nv = " << model->nv << " ..." << std::endl;
  params->DT = 0.01;
  //params->mainJointIds = [];
  params->baumgartGains.resize(2); params->baumgartGains <<  0., 100.;
  params->stateImportance.resize(model->nv*2); params->stateImportance << 0. ,  0. ,  0. , 50. , 50. ,  0. ,  5. ,  5. ,  1. ,  2. ,  1. ,  1. ,  5. ,  5. ,  1. ,  2. ,  1. ,  1.,  0. ,  0. ,  0. ,  3. ,  3. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1.;
  params->stateTerminalImportance.resize(model->nv*2); params->stateTerminalImportance << 3. ,  3. ,  0. ,  0. ,  0. , 30. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0.,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1. ,  1.;
  params->controlImportance.resize(model->nv-6); params->controlImportance <<1. , 1. , 1. , 1. , 1. , 1. , 1. , 1. , 1. , 1. , 1. , 1.;
  params->vcomImportance.resize(3); params->vcomImportance <<0. , 0. , 1.;
  params->forceImportance.resize(6); params->forceImportance << 1. ,   1. ,   0.1,  10. ,  10. ,   2.  ;
  params->vcomRef.resize(3); params->vcomRef <<0.05, 0. ,   0.   ;
  params->footSize = 0.05;
  params->refStateWeight = 0.1;
  params->refTorqueWeight = 0.0;
  params->comWeight = 0.0;
  params->vcomWeight = 1.0;
  params->copWeight = 2.0;
  params->conePenaltyWeight = 0.0;
  params->coneAxisWeight = 0.0002;
  params->refForceWeight = 10.0;
  params->impactAltitudeWeight = 20000.0;
  params->impactVelocityWeight = 10000.0;
  params->impactRotationWeight = 200.0;
  params->refMainJointsAtImpactWeight = 0.0;
  params->verticalFootVelWeight = 20.0;
  params->flyHighSlope = 42.857142857142854;
  params->flyHighWeight = 200.0;
  params->groundColWeight = 200.0;
  params->footMinimalDistance = 0.2;
  params->feetCollisionWeight = 1000.0;
  params->kktDamping = 0.0;
  params->stateTerminalWeight = 20.0;
  params->solver_th_stop = 0.001;
  params->transitionDuration = 6;

  // Pattern
  std::cout << "Create contact pattern" << std::endl;
  int Tstart = 20, Tsingle = 40, Tdouble = 15, Tend = 20, Tmpc = 120;

  Eigen::MatrixXd patternStart(2, Tstart);
  patternStart.fill(1);
  Eigen::MatrixXd patternDouble(2, Tdouble);
  patternDouble.fill(1);
  Eigen::MatrixXd patternEnd(2, Tend);
  patternEnd.fill(1);
  Eigen::MatrixXd patternLeft(2, Tsingle);
  patternLeft.topRows<1>().fill(0);
  patternLeft.bottomRows<1>().fill(1);
  Eigen::MatrixXd patternRight(2, Tsingle);
  patternRight.topRows<1>().fill(1);
  patternRight.bottomRows<1>().fill(0);

  Eigen::MatrixXd contactPattern(2, Tstart + Tdouble * 3 + Tsingle * 2 + Tend);
  contactPattern << patternStart, patternDouble, patternLeft, patternDouble,
      patternRight, patternDouble, patternEnd;

  std::cout << "Contact pattern = \n"
            << contactPattern.transpose() << std::endl;

  // OCP
  auto ocp = boost::make_shared<OCPWalk>(robot, params, contactPattern);

  /*
  Eigen::VectorXd x = mpc->problem->get_x0();
  for (int t = 1; t <= 100; t++) {
    mpc->calc(x, t);
    x = mpc->solver->get_xs()[1];
    }*/
}

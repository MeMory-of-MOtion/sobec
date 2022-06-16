#include <sobec/ocp-walk.hpp>
#include <sobec/mpc-walk.hpp>

void initParamsFromAutomaticallyGeneratedCode(boost::shared_ptr<sobec::OCPWalkParams> params)
{
  params->DT = 0.0100000000; 
 params->mainJointIds = { "leg_left_1_joint" , "leg_left_2_joint", "leg_left_4_joint", "leg_right_1_joint", "leg_right_2_joint", "leg_right_4_joint" };
  params->baumgartGains.resize(2);params->baumgartGains <<   0.0000000000, 50.0000000000;
  params->stateImportance.resize(40);params->stateImportance <<   0.0000000000, 0.0000000000, 0.0000000000, 50.0000000000, 50.0000000000, 0.0000000000, 5.0000000000, 5.0000000000, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, 5.0000000000, 5.0000000000, 1.0000000000, 2.0000000000, 1.0000000000, 1.0000000000, 3.0000000000, 3.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 3.0000000000, 3.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 2.0000000000, 2.0000000000;
  params->stateTerminalImportance.resize(40);params->stateTerminalImportance <<   3.0000000000, 3.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 30.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000;
  params->controlImportance.resize(14);params->controlImportance <<   1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000, 1.0000000000;
  params->vcomImportance.resize(3);params->vcomImportance <<   0.0000000000, 0.0000000000, 1.0000000000;
  params->forceImportance.resize(6);params->forceImportance <<   1.0000000000, 1.0000000000, 0.1000000000, 10.0000000000, 10.0000000000, 2.0000000000;
  params->vcomRef.resize(3);params->vcomRef <<   0.1000000000, 0.0000000000, 0.0000000000;
  params->footSize = 0.0500000000; 
  params->refStateWeight = 0.1000000000; 
  params->refTorqueWeight = 0; 
  params->comWeight = 0; 
  params->vcomWeight = 1; 
  params->copWeight = 2; 
  params->conePenaltyWeight = 20; 
  params->coneAxisWeight = 0.0002000000; 
  params->refForceWeight = 10; 
  params->impactAltitudeWeight = 20000; 
  params->impactVelocityWeight = 200; 
  params->impactRotationWeight = 200; 
  params->refMainJointsAtImpactWeight = 0; 
  params->verticalFootVelWeight = 20; 
  params->flyHighSlope = 166.6666666667; 
  params->flyHighWeight = 20; 
  params->groundColWeight = 200; 
  params->footMinimalDistance = 0.2000000000; 
  params->feetCollisionWeight = 1000; 
  params->kktDamping = 0; 
  params->stateTerminalWeight = 20; 
  params->solver_th_stop = 0.0010000000; 
  params->transitionDuration = 4; 

}
void initMPCFromAutomaticallyGeneratedCode(boost::shared_ptr<sobec::MPCWalk> mpc)
{
  // *** Cannot find field <calc> in python params object.
  // *** Cannot find field <initialize> in python params object.
  mpc->Tmpc = 160; 
  mpc->Tstart = 30; 
  mpc->Tdouble = 11; 
  mpc->Tsingle = 80; 
  mpc->Tend = 30; 
  mpc->vcomRef.resize(3);mpc->vcomRef <<   0.1000000000, 0.0000000000, 0.0000000000;
  mpc->solver_th_stop = 0.0010000000; 
  mpc->solver_reg_min = 0.0000010000; 
  mpc->solver_maxiter = 2; 
  mpc->DT = 0.0100000000; 
  // *** Cannot find field <x0> in python params object.
  // *** Cannot find field <storage> in python params object.
  // *** Cannot find field <problem> in python params object.
  // *** Cannot find field <state> in python params object.
  // *** Cannot find field <solver> in python params object.

}

bool checkAutomaticallyGeneratedCodeCompatibility(boost::shared_ptr<sobec::OCPRobotWrapper> robot)
{
  bool res = true;
  res &= (robot->model->nq == 21);
  res &= (robot->model->nv == 20);
  return res;
}

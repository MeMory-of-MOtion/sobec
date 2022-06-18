///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2022, LAAS-CNRS, New York University, Max Planck
// Gesellschaft
//                          University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#define BOOST_TEST_MODULE init shooting problem
#include <boost/test/included/unit_test.hpp>
#include <iostream>

#include "sobec/mpc-walk.hpp"
#include "sobec/ocp-walk.hpp"

BOOST_AUTO_TEST_CASE(test_walk_params) {
  sobec::OCPWalkParams anOCPWalkParams;
  std::string filename(PROJECT_SOURCE_DIR "/tests/config_walk.yaml");
  anOCPWalkParams.readParamsFromYamlFile(filename);

  BOOST_CHECK(anOCPWalkParams.DT == 0.01);
}

BOOST_AUTO_TEST_CASE(test_ocp_walk_params) {
  sobec::OCPWalkParams ocpparams;
  std::string filename(PROJECT_SOURCE_DIR "/tests/config-walk.yaml");
  ocpparams.readParamsFromYamlFile(filename);

  std::cout << " == OCP Params " << std::endl;
  std::cout << "DT = " << ocpparams.DT << std::endl;
  // std::cout << "mainJointIds = " << ocpparams.mainJointIds << std::endl;
  std::cout << "stateImportance = " << ocpparams.stateImportance.transpose()
            << std::endl;
  std::cout << "stateTerminalImportance = "
            << ocpparams.stateTerminalImportance.transpose() << std::endl;
  std::cout << "controlImportance = " << ocpparams.controlImportance.transpose()
            << std::endl;
  std::cout << "vcomImportance = " << ocpparams.vcomImportance.transpose()
            << std::endl;
  std::cout << "forceImportance = " << ocpparams.forceImportance.transpose()
            << std::endl;
  std::cout << "vcomRef=" << ocpparams.vcomRef.transpose() << std::endl;
  std::cout << "footSize = " << ocpparams.footSize << std::endl;
  std::cout << "refStateWeight = " << ocpparams.refStateWeight << std::endl;
  std::cout << "refTorqueWeight = " << ocpparams.refTorqueWeight << std::endl;
  std::cout << "comWeight = " << ocpparams.comWeight << std::endl;
  std::cout << "vcomWeight = " << ocpparams.vcomWeight << std::endl;
  std::cout << "copWeight = " << ocpparams.copWeight << std::endl;
  std::cout << "conePenaltyWeight = " << ocpparams.conePenaltyWeight
            << std::endl;
  std::cout << "coneAxisWeight = " << ocpparams.coneAxisWeight << std::endl;
  std::cout << "refForceWeight = " << ocpparams.refForceWeight << std::endl;
  std::cout << "impactAltitudeWeight = " << ocpparams.impactAltitudeWeight
            << std::endl;
  std::cout << "impactVelocityWeight = " << ocpparams.impactVelocityWeight
            << std::endl;
  std::cout << "impactRotationWeight = " << ocpparams.impactRotationWeight
            << std::endl;
  std::cout << "refMainJointsAtImpactWeight = "
            << ocpparams.refMainJointsAtImpactWeight << std::endl;
  std::cout << "verticalFootVelWeight = " << ocpparams.verticalFootVelWeight
            << std::endl;
  std::cout << "flyHighSlope = " << ocpparams.flyHighSlope << std::endl;
  std::cout << "flyHighWeight = " << ocpparams.flyHighWeight << std::endl;
  std::cout << "groundColWeight = " << ocpparams.groundColWeight << std::endl;
  std::cout << "footMinimalDistance = " << ocpparams.footMinimalDistance
            << std::endl;
  std::cout << "feetCollisionWeight = " << ocpparams.feetCollisionWeight
            << std::endl;
  std::cout << "kktDamping = " << ocpparams.kktDamping << std::endl;
  std::cout << "stateTerminalWeight = " << ocpparams.stateTerminalWeight
            << std::endl;
  std::cout << "solver_th_stop = " << ocpparams.solver_th_stop << std::endl;
  std::cout << "transitionDuration = " << ocpparams.transitionDuration
            << std::endl;
  std::cout << "minimalNormalForce = " << ocpparams.minimalNormalForce
            << std::endl;
  std::cout << "withNormalForceBoundOnly = "
            << ocpparams.withNormalForceBoundOnly << std::endl;
}
BOOST_AUTO_TEST_CASE(test_mpc_walk_params_real) {
  sobec::MPCWalkParams mpcparams;
  std::string filename(PROJECT_SOURCE_DIR "/tests/config-walk.yaml");
  mpcparams.readParamsFromYamlFile(filename);
  std::cout << " == MPC Params " << std::endl;
  std::cout << "vcomRef = " << mpcparams.vcomRef.transpose() << std::endl;
  // std::cout << "x0 = " << mpcparams.x0.transpose() << std::endl;
  std::cout << "Tmpc = " << mpcparams.Tmpc << std::endl;
  std::cout << "Tstart = " << mpcparams.Tstart << std::endl;
  std::cout << "Tsingle = " << mpcparams.Tsingle << std::endl;
  std::cout << "Tdouble = " << mpcparams.Tdouble << std::endl;
  std::cout << "Tend = " << mpcparams.Tend << std::endl;
  std::cout << "DT = " << mpcparams.DT << std::endl;
  std::cout << "solver_th_stop = " << mpcparams.solver_th_stop << std::endl;
  std::cout << "solver_reg_min = " << mpcparams.solver_reg_min << std::endl;
  std::cout << "solver_maxiter = " << mpcparams.solver_maxiter << std::endl;
}

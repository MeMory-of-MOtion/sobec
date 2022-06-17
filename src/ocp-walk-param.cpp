///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////
#include "yaml-cpp/yaml.h"

#include "sobec/ocp-walk.hpp"
namespace sobec {

  
  void OCPWalkParams::readParams
  (
   std::string &StringToParse
   )
  {
    YAML::Node root = YAML::Load(StringToParse);
    YAML::Node config = root["walk"];

    if(!config)
      {
	std::cerr << "No walk section." << std::endl;
	return;
      }

    // Local lambda function to read vectorX
    auto read_double =
      [&config] (double& aref_d,
		 std::string fieldname) {
	YAML::Node yn_ad = config[fieldname];
	if (yn_ad) {
	  aref_d = yn_ad.as<double>();
	} else { std::cout << "No " << fieldname << std::endl; } 
      };

    read_double(DT,"DT");
    
    YAML::Node yn_mainJointIds = config["mainJointIds"];
    if (yn_mainJointIds)  {
      for(std::size_t id=0; id<yn_mainJointIds.size();id++) {
	mainJointIds[id] = yn_mainJointIds[id].as<std::string>();
      }
    } else { std::cout << "No mainJointIds" << std::endl; }


    YAML::Node yn_baumgartGains = config["baumgartGains"];
    if (yn_baumgartGains) {
      for(std::size_t id=0; id<yn_baumgartGains.size();id++) {
	baumgartGains[(Eigen::Index)id] = yn_baumgartGains[id].as<double>();
      }
    } else { std::cout << "No baumgartGains" << std::endl; }

    // Local lambda function to read vectorX
    auto read_vxd =
      [&config] (Eigen::VectorXd & aref_vxd,
		 std::string fieldname) {
	YAML::Node yn_avxd = config[fieldname];
	if (yn_avxd) {
	  aref_vxd.resize(yn_avxd.size());
	  for(std::size_t id=0; id<yn_avxd.size();id++) {
	    aref_vxd[(Eigen::Index)id] = yn_avxd[id].as<double>();
	  }
	} else { std::cout << "No " << fieldname << std::endl; } 
      };
    
    read_vxd(stateImportance,"stateImportance");
    read_vxd(stateTerminalImportance,"stateTerminalImportance");
    read_vxd(controlImportance,"controlImportance");
    read_vxd(vcomImportance,"vcomImportance");
    read_vxd(forceImportance,"forceImportance");  

    YAML::Node yn_vcomRef = config["vcomRef"];
    if (yn_vcomRef) {
      for(std::size_t id=0; id<yn_vcomRef.size();id++) {
	vcomRef[(Eigen::Index)id] = yn_vcomRef[id].as<double>();
      }
    } else { std::cout << "No vcomRef" << std::endl; }

    read_double(footSize,"footSize");
    read_double(refStateWeight,"refStateWeight");
    read_double(refTorqueWeight,"refTorqueWeight");
    read_double(comWeight,"comWeight");
    read_double(vcomWeight,"vcomWeight");
    read_double(copWeight,"copWeight");
    read_double(conePenaltyWeight,"conePenaltyWeight");
    read_double(coneAxisWeight,"coneAxisWeight");
    read_double(refForceWeight,"refForceWeight");
    read_double(impactAltitudeWeight,"impactAltitudeWeight");
    read_double(impactVelocityWeight,"impactVelocityWeight");
    read_double(impactRotationWeight,"impactRotationWeight");
    read_double(refMainJointsAtImpactWeight,"refMainJointsAtImpactWeight");
    read_double(verticalFootVelWeight,"verticalFootVelWeight");
    read_double(flyHighSlope,"flyHighSlope");
    read_double(flyHighWeight,"flyHighWeight");
    read_double(groundColWeight,"groundColWeight");
    read_double(footMinimalDistance,"footMinimalDistance");
    read_double(feetCollisionWeight,"feetCollisionWeight");
    read_double(kktDamping,"kktDamping");
    read_double(stateTerminalWeight,"stateTerminalWeight");
    read_double(solver_th_stop,"solver_th_stop");
    
    if (config["transitionDuration"])
      transitionDuration = config["transitionDuration"].as<int>();
    else { std::cout << "No transitionDuration" << std::endl; }
  }
  
  void OCPWalkParams::readParamsFile(std::string &Filename) {
    std::ifstream t(Filename);
    std::stringstream buffer;
    buffer << t.rdbuf();
    std::string StringToParse = buffer.str();
    readParams(StringToParse);
  }
  
}

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////
#include "sobec/mpc-walk.hpp"
#include "yaml-cpp/yaml.h"
namespace sobec {

void MPCWalkParams::readParamsFromYamlString(std::string &StringToParse) {
  YAML::Node root = YAML::Load(StringToParse);
  YAML::Node config = root["walk"];

  if (!config) {
    std::cerr << "No walk section." << std::endl;
    return;
  }

  // Local lambda function to read double
  auto read_double = [&config](double &aref_d, std::string fieldname) {
    YAML::Node yn_ad = config[fieldname];
    if (yn_ad) {
      aref_d = yn_ad.as<double>();
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  // Local lambda function to read int
  auto read_int = [&config](int &aref_d, std::string fieldname) {
    YAML::Node yn_ad = config[fieldname];
    if (yn_ad) {
      aref_d = yn_ad.as<int>();
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  // Local lambda function to read vectorX
  auto read_vxd = [&config](Eigen::VectorXd &aref_vxd, std::string fieldname) {
    YAML::Node yn_avxd = config[fieldname];
    if (yn_avxd) {
      aref_vxd.resize(yn_avxd.size());
      for (std::size_t id = 0; id < yn_avxd.size(); id++) {
        aref_vxd[(Eigen::Index)id] = yn_avxd[id].as<double>();
      }
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  // Local lambda function to read vector3
  auto read_v3d = [&config](Eigen::Vector3d &aref_v3d, std::string fieldname) {
    YAML::Node yn_av3d = config[fieldname];
    if (yn_av3d) {
      aref_v3d.resize(3);
      for (std::size_t id = 0; id < yn_av3d.size(); id++) {
        aref_v3d[(Eigen::Index)id] = yn_av3d[id].as<double>();
      }
    } else {
      std::cout << "No " << fieldname << std::endl;
    }
  };

  read_v3d(vcomRef, "vcomRef");
  read_vxd(x0, "x0");
  read_int(Tmpc, "Tmpc");
  read_int(Tstart, "Tstart");
  read_int(Tsingle, "Tsingle");
  read_int(Tdouble, "Tdouble");
  read_int(Tend, "Tend");
  read_double(DT, "DT");
  read_double(solver_th_stop, "solver_th_stop");
  read_double(solver_reg_min, "solver_reg_min");
  read_int(solver_maxiter, "solver_maxiter");
}

void MPCWalkParams::readParamsFromYamlFile(const std::string &Filename) {
  std::ifstream t(Filename);
  std::stringstream buffer;
  buffer << t.rdbuf();
  std::string StringToParse = buffer.str();
  readParamsFromYamlString(StringToParse);
}

}  // namespace sobec

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_STATESOFT_FACTORY_HPP_
#define SOBEC_STATESOFT_FACTORY_HPP_

#include <crocoddyl/core/numdiff/state.hpp>
#include <crocoddyl/core/state-base.hpp>
#include <crocoddyl/core/utils/exception.hpp>

#include "pinocchio_model.hpp"
#include "sobec/crocomplements/softcontact/state.hpp"
#include "state.hpp"

namespace sobec {
namespace unittest {

struct StateSoftContactModelTypes {
  enum Type {
    StateSoftContact_TalosArm,
    StateSoftContact_HyQ,
    StateSoftContact_Talos,
    StateSoftContact_RandomHumanoid,
    NbStateSoftContactModelTypes
  };
  static std::vector<Type> init_all() {
    std::vector<Type> v;
    v.clear();
    for (int i = 0; i < NbStateSoftContactModelTypes; ++i) {
      v.push_back((Type)i);
    }
    return v;
  }
  static const std::vector<Type> all;
};

const std::map<StateSoftContactModelTypes::Type, StateModelTypes::Type>
    mapStateSoftContactToStateMultibody{
        {StateSoftContactModelTypes::StateSoftContact_TalosArm,
         StateModelTypes::StateMultibody_TalosArm},
        {StateSoftContactModelTypes::StateSoftContact_HyQ, StateModelTypes::StateMultibody_HyQ},
        {StateSoftContactModelTypes::StateSoftContact_Talos,
         StateModelTypes::StateMultibody_Talos},
        {StateSoftContactModelTypes::StateSoftContact_RandomHumanoid,
         StateModelTypes::StateMultibody_RandomHumanoid}};

std::ostream& operator<<(std::ostream& os, StateSoftContactModelTypes::Type type);

class StateSoftContactModelFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit StateSoftContactModelFactory();
  ~StateSoftContactModelFactory();

  boost::shared_ptr<sobec::StateSoftContact> create(
      StateSoftContactModelTypes::Type state_type, std::size_t nc = 3) const;
};

}  // namespace unittest
}  // namespace sobec

#endif  // SOBEC_STATESOFT_FACTORY_HPP_

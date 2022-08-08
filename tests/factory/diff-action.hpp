///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, University of Edinburgh, CTU, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_DIFF_ACTION_FACTORY_HPP_
#define SOBEC_DIFF_ACTION_FACTORY_HPP_

#include <crocoddyl/core/diff-action-base.hpp>
#include <crocoddyl/core/numdiff/diff-action.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>

#include "actuation.hpp"
#include "contact1d.hpp"
#include "contact3d.hpp"
#include "sobec/crocomplements/contact/contact-fwddyn.hpp"
#include "sobec/crocomplements/softcontact/soft-contact3d-fwddyn.hpp"
#include "state.hpp"

namespace sobec {
namespace unittest {

struct DifferentialActionModelTypes {
  enum Type {
    DifferentialActionModelFreeFwdDynamics_TalosArm,
    DifferentialActionModelFreeFwdDynamics_TalosArm_Squashed,
    DifferentialActionModelContact1DFwdDynamics_TalosArm,
    DifferentialActionModelContact3DFwdDynamics_TalosArm,
    DifferentialActionModelSoftContact3DFwdDynamics_TalosArm,
    DifferentialActionModelSoftContact3DFwdDynamics_HyQ,
    DifferentialActionModelContact1DFwdDynamics_HyQ,
    DifferentialActionModelContact3DFwdDynamics_HyQ,
    NbDifferentialActionModelTypes
  };
  static std::vector<Type> init_all() {
    std::vector<Type> v;
    v.clear();
    for (int i = 0; i < NbDifferentialActionModelTypes; ++i) {
      v.push_back((Type)i);
    }
    return v;
  }
  static const std::vector<Type> all;
};

std::ostream& operator<<(std::ostream& os,
                         DifferentialActionModelTypes::Type type);

class DifferentialActionModelFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit DifferentialActionModelFactory();
  ~DifferentialActionModelFactory();

  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> create(
      DifferentialActionModelTypes::Type type,
      PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
      ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::X) const;

  // Free forward dynamics
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>
  create_freeFwdDynamics(StateModelTypes::Type state_type,
                         ActuationModelTypes::Type actuation_type) const;
  
  // Rigid contact 3D dynamics
  boost::shared_ptr<newcontacts::DifferentialActionModelContactFwdDynamics>
  create_contact3DFwdDynamics(StateModelTypes::Type state_type,
                              ActuationModelTypes::Type actuation_type,
                              PinocchioReferenceTypes::Type ref_type) const;

  // Rigid contact 1D dynamics
  boost::shared_ptr<newcontacts::DifferentialActionModelContactFwdDynamics>
  create_contact1DFwdDynamics(StateModelTypes::Type state_type,
                              ActuationModelTypes::Type actuation_type,
                              PinocchioReferenceTypes::Type ref_type,
                              ContactModelMaskTypes::Type mask_type) const;

  // Soft contact 3D dynamics
  boost::shared_ptr<sobec::DifferentialActionModelSoftContact3DFwdDynamics>
  create_softContact3DFwdDynamics(StateModelTypes::Type state_type,
                                  ActuationModelTypes::Type actuation_type,
                                  PinocchioReferenceTypes::Type ref_type) const;
};

}  // namespace unittest
}  // namespace sobec

#endif  // SOBEC_DIFF_ACTION_FACTORY_HPP_

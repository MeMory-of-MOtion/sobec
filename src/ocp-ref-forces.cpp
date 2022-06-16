///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/fwd.hpp"
#include "sobec/ocp-walk.hpp"

namespace sobec {

// contact_pattern(k,t)

Eigen::MatrixXd computeWeightShareSmoothProfile(
    const Eigen::Ref<const Eigen::MatrixXd> contact_pattern, int duration) {
  double robotGravityForce = 1;              // todo
  Eigen::Index T = contact_pattern.cols();   // time horizon length
  Eigen::Index nc = contact_pattern.rows();  // number of contact
  //    std::cout << T << " x " << nc << std::endl;
  Eigen::MatrixXd contactImportance(nc, T);

  // Contact importance is set at a fair discontinuous share between all the
  // active contacts.
  for (Eigen::Index t = 0; t < T; ++t) {
    double nbactiv = contact_pattern.col(t).sum();
    contactImportance.col(t) =
        contact_pattern.col(t) * (robotGravityForce / nbactiv);
  }

  for (Eigen::Index t = 1; t < T; ++t) {
    bool landing = false;
    for (Eigen::Index k = 0; k < nc; ++k) {
      if ((!contact_pattern(k, t - 1)) && contact_pattern(k, t)) landing = true;
    }
    if (landing)
      for (Eigen::Index s = duration - 1; s >= 0; --s) {
        double ratio = (double(s + 1) / double(duration + 1));
        for (Eigen::Index k = 0; k < nc; ++k) {
          std::cout << k << " " << s << " " << contactImportance(k, t - 1)
                    << ratio << std::endl;
          contactImportance(k, t + s) =
              contactImportance(k, t - 1) * (1 - ratio) +
              contactImportance(k, t) * ratio;
        }
      }
  }
  for (Eigen::Index t = T - 1; t >= 1; --t) {
    bool takingoff = false;
    for (Eigen::Index k = 0; k < nc; ++k) {
      if ((!contact_pattern(k, t)) && contact_pattern(k, t - 1))
        takingoff = true;
    }
    if (takingoff)
      for (Eigen::Index s = duration - 1; s >= 0; --s) {
        double ratio = (double(s + 1) / double(duration + 1));
        for (Eigen::Index k = 0; k < nc; ++k) {
          contactImportance(k, t - s - 1) =
              contactImportance(k, t) * (1 - ratio) +
              contactImportance(k, t - 1) * ratio;
        }
      }
  }

  return contactImportance;
}

void OCPWalk::computeReferenceForces() {
  double duration = params->transitionDuration;
  double robotGravityForce = robot->robotGravityForce;

  Eigen::MatrixXd contactImportance =
      computeWeightShareSmoothProfile(contact_pattern, duration);

  Eigen::Index T = contact_pattern.cols();   // time horizon length
  Eigen::Index nc = contact_pattern.rows();  // number of contact

  referenceForces.resize(T);

  pinocchio::Force grav = pinocchio::Force::Zero();
  grav.linear()[2] = robotGravityForce;

  // contact importance = 1/nb_active_contact
  for (Eigen::Index t = 0; t < T; ++t)
    for (Eigen::Index k = 0; k < nc; ++k) {
      referenceForces[t].push_back(grav * contactImportance(k, t));
    }
}

}  // namespace sobec

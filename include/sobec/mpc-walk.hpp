///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_MPC_WALK_HPP_
#define SOBEC_MPC_WALK_HPP_

#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;
/**
 * @brief MPC manager, calling iterative subpart of a larger OCP.
 */
  
class MPCWalk {
  typedef typename MathBaseTpl<double>::VectorXs VectorXs;

  
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  explicit MPCWalk(boost::shared_ptr<ShootingProblem> problem);
  
  virtual ~MPCWalk();

  void calc(const Eigen::Ref<const VectorXs>& x);


 private:

};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */

#include "sobec/mpc-walk.hxx"

#endif  // SOBEC_MPC_WALK_HPP_

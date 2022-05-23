///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_FWD_HPP_
#define SOBEC_FWD_HPP_

namespace sobec {

// Cost COM-vel
template <typename Scalar>
class ResidualModelCoMVelocityTpl;
template <typename Scalar>
struct ResidualDataCoMVelocityTpl;
typedef ResidualModelCoMVelocityTpl<double> ResidualModelCoMVelocity;
typedef ResidualDataCoMVelocityTpl<double> ResidualDataCoMVelocity;

// Activation quad-ref
template <typename Scalar>
class ActivationModelQuadRefTpl;
typedef ActivationModelQuadRefTpl<double> ActivationModelQuadRef;

// State LPF
template <typename Scalar>
class StateLPFTpl;
typedef StateLPFTpl<double> StateLPF;

}  // namespace sobec

#endif  // SOBEC_FWD_HPP_

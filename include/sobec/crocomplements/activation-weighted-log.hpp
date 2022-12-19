///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_CORE_ACTIVATIONS_WEIGHTED_LOG_HPP_
#define CROCODDYL_CORE_ACTIVATIONS_WEIGHTED_LOG_HPP_

#include <crocoddyl/core/activation-base.hpp>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <stdexcept>

#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;

template <typename _Scalar>
class ActivationModelWeightedLogTpl : public ActivationModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ActivationModelAbstractTpl<Scalar> Base;
  typedef ActivationDataAbstractTpl<Scalar> ActivationDataAbstract;
  typedef ActivationDataWeightedLogTpl<Scalar> Data;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  explicit ActivationModelWeightedLogTpl(const VectorXs &weights, const Scalar &alpha = Scalar(1.))
      : Base(weights.size()), weights_(weights), alpha_(alpha) {
    if (alpha < Scalar(0.)) {
      throw_pretty("Invalid argument: "
                   << "alpha should be a positive value");
    }
  };
  virtual ~ActivationModelWeightedLogTpl(){};

  virtual void calc(const boost::shared_ptr<ActivationDataAbstract> &data, const Eigen::Ref<const VectorXs> &r) {
    if (static_cast<std::size_t>(r.size()) != nr_) {
      throw_pretty("Invalid argument: "
                   << "r has wrong dimension (it should be " + std::to_string(nr_) + ")");
    }
    boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);
    d->Wr = weights_.cwiseProduct(r);
    d->a0 = r.dot(d->Wr) / alpha_;
    data->a_value = log(Scalar(1.0) + d->a0);
  };

  virtual void calcDiff(const boost::shared_ptr<ActivationDataAbstract> &data, const Eigen::Ref<const VectorXs> &r) {
    if (static_cast<std::size_t>(r.size()) != nr_) {
      throw_pretty("Invalid argument: "
                   << "r has wrong dimension (it should be " + std::to_string(nr_) + ")");
    }
    boost::shared_ptr<Data> d = boost::static_pointer_cast<Data>(data);

    d->a1 = Scalar(2.0) / (alpha_ + alpha_ * d->a0);
    data->Ar = d->a1 * d->Wr;
    data->Arr.diagonal() = -d->a1 * d->a1 * d->Wr.array().square();
    data->Arr.diagonal() += d->a1 * weights_;
  };

  virtual boost::shared_ptr<ActivationDataAbstract> createData() {
    boost::shared_ptr<Data> data = boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
    return data;
  };

  Scalar get_alpha() const { return alpha_; };
  void set_alpha(const Scalar alpha) { alpha_ = alpha; };
  const VectorXs &get_weights() const { return weights_; };
  void set_weights(const VectorXs &weights) {
    if (weights.size() != weights_.size()) {
      throw_pretty("Invalid argument: "
                   << "weight vector has wrong dimension (it should be " + std::to_string(weights_.size()) + ")");
    }

    weights_ = weights;
  };

  virtual void print(std::ostream &os) const {
    os << "ActivationModelWeightedLog {nr=" << nr_ << ", a=" << alpha_ << "}";
  }

 protected:
  using Base::nr_;  //!< Dimension of the residual vector

 private:
  VectorXs weights_;  //!< weights used for the activation
  Scalar alpha_;      //!< Width of quadratic basin
};

template <typename _Scalar>
struct ActivationDataWeightedLogTpl : public ActivationDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef typename MathBase::VectorXs VectorXs;
  typedef ActivationDataAbstractTpl<Scalar> Base;

  template <typename Activation>
  explicit ActivationDataWeightedLogTpl(Activation *const activation)
      : Base(activation), a0(0), a1(0), Wr(VectorXs::Zero(activation->get_nr())) {}

  Scalar a0;
  Scalar a1;
  VectorXs Wr;
};

}  // namespace sobec

#endif  // CROCODDYL_CORE_ACTIVATIONS_WEIGHTED_LOG_HPP_

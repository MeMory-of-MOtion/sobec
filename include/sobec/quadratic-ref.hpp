///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_CORE_ACTIVATIONS_QUADRATIC_REF_HPP_
#define CROCODDYL_CORE_ACTIVATIONS_QUADRATIC_REF_HPP_

#include <stdexcept>
#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/activation-base.hpp"
#include "crocoddyl/core/utils/exception.hpp"

namespace crocoddyl {

template <typename _Scalar>
class ActivationModelQuadRefTpl : public ActivationModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ActivationModelAbstractTpl<Scalar> Base;
  typedef ActivationDataAbstractTpl<Scalar> ActivationDataAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  explicit ActivationModelQuadRefTpl(const VectorXs& reference) : Base(reference.size()), ref(reference) {};
  virtual ~ActivationModelQuadRefTpl(){};

  virtual void calc(const boost::shared_ptr<ActivationDataAbstract>& data, const Eigen::Ref<const VectorXs>& r) {
    if (static_cast<std::size_t>(r.size()) != nr_) {
      throw_pretty("Invalid argument: "
                   << "r has wrong dimension (it should be " + std::to_string(nr_) + ")");
    }
    data->a_value = (Scalar(0.5) * (r - ref).transpose() * (r - ref))[0];
  };

  virtual void calcDiff(const boost::shared_ptr<ActivationDataAbstract>& data, const Eigen::Ref<const VectorXs>& r) {
    if (static_cast<std::size_t>(r.size()) != nr_) {
      throw_pretty("Invalid argument: "
                   << "r has wrong dimension (it should be " + std::to_string(nr_) + ")");
    }

    data->Ar = r - ref;
    // The Hessian has constant values which were set in createData.
    assert_pretty(MatrixXs(data->Arr.diagonal().asDiagonal()).isApprox(MatrixXs::Identity(nr_, nr_)),
                  "Arr has wrong value");
  };

  virtual boost::shared_ptr<ActivationDataAbstract> createData() {
    boost::shared_ptr<ActivationDataAbstract> data =
        boost::allocate_shared<ActivationDataAbstract>(Eigen::aligned_allocator<ActivationDataAbstract>(), this);
    data->Arr.diagonal().fill((Scalar)1.);
    return data;
  };
 
  const VectorXs& get_reference() const { return ref; };
  void set_reference(const VectorXs& reference) {
    if (ref.size() != reference.size()) {
      throw_pretty("Invalid argument: "
                   << "weight vector has wrong dimension (it should be " + std::to_string(ref.size()) + ")");
    }

    ref = reference;
  };
 
 protected:
  using Base::nr_;
  
 private:
  VectorXs ref;
};

}  // namespace crocoddyl

#endif  // CROCODDYL_CORE_ACTIVATIONS_QUADRATIC_REF_HPP_

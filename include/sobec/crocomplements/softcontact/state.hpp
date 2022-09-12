///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_STATE_SOFT_CONTACT_AUGMENTED_HPP_
#define SOBEC_STATE_SOFT_CONTACT_AUGMENTED_HPP_
#include <pinocchio/multibody/model.hpp>

#include "crocoddyl/core/state-base.hpp"
#include "sobec/fwd.hpp"
namespace sobec {
using namespace crocoddyl;

template <typename _Scalar>
class StateSoftContactTpl : public StateAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef StateAbstractTpl<Scalar> Base;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  enum JointType { FreeFlyer = 0, Spherical, Simple };

  explicit StateSoftContactTpl(
      boost::shared_ptr<pinocchio::ModelTpl<Scalar> > model, std::size_t nc);
  virtual ~StateSoftContactTpl();

  virtual VectorXs zero() const;
  virtual VectorXs rand() const;
  virtual void diff(const Eigen::Ref<const VectorXs>& y0,
                    const Eigen::Ref<const VectorXs>& y1,
                    Eigen::Ref<VectorXs> dyout) const;
  virtual void integrate(const Eigen::Ref<const VectorXs>& y,
                         const Eigen::Ref<const VectorXs>& dy,
                         Eigen::Ref<VectorXs> yout) const;
  virtual void Jdiff(const Eigen::Ref<const VectorXs>&,
                     const Eigen::Ref<const VectorXs>&,
                     Eigen::Ref<MatrixXs> Jfirst, Eigen::Ref<MatrixXs> Jsecond,
                     const Jcomponent firstsecond = both) const;

  virtual void Jintegrate(const Eigen::Ref<const VectorXs>& y,
                          const Eigen::Ref<const VectorXs>& dy,
                          Eigen::Ref<MatrixXs> Jfirst,
                          Eigen::Ref<MatrixXs> Jsecond,
                          const Jcomponent firstsecond = both,
                          const AssignmentOp = setto) const;
  virtual void JintegrateTransport(const Eigen::Ref<const VectorXs>& y,
                                   const Eigen::Ref<const VectorXs>& dy,
                                   Eigen::Ref<MatrixXs> Jin,
                                   const Jcomponent firstsecond) const;

  const boost::shared_ptr<pinocchio::ModelTpl<Scalar> >& get_pinocchio() const;
  const std::size_t& get_nc() const;
  const std::size_t& get_ny() const;
  const std::size_t& get_ndy() const;

 protected:
  using Base::has_limits_;
  using Base::lb_;
  using Base::ndx_;
  using Base::nq_;
  using Base::nv_;
  using Base::nx_;
  using Base::ub_;
  boost::shared_ptr<pinocchio::ModelTpl<Scalar> > pinocchio_;
  std::size_t nc_;
  std::size_t ny_;
  std::size_t ndy_;

 private:
  // boost::shared_ptr<pinocchio::ModelTpl<Scalar> > pinocchio_;
  VectorXs y0_;
  JointType joint_type_;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/crocomplements/softcontact/state-soft-contact.hxx"

#endif  // SOBEC_STATE_SOFT_CONTACT_AUGMENTED_HPP_

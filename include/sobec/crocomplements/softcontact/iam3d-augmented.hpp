///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_IAM3D_AUGMENTED_HPP_
#define SOBEC_IAM3D_AUGMENTED_HPP_

#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/diff-action-base.hpp>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/multibody/model.hpp>

#include "state.hpp"
#include "dam3d-augmented.hpp"

namespace sobec {
using namespace crocoddyl;

template <typename _Scalar>
class IAMSoftContact3DAugmentedTpl : public ActionModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ActionModelAbstractTpl<Scalar> Base;
  typedef IADSoftContact3DAugmentedTpl<Scalar> Data;
  typedef ActionDataAbstractTpl<Scalar> ActionDataAbstract;
  typedef DifferentialActionModelAbstractTpl<Scalar>
      DifferentialActionModelAbstract;
  typedef DAMSoftContact3DAugmentedFwdDynamicsTpl<Scalar>
      DAMSoftContact3DAugmentedFwdDynamics;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef StateSoftContactTpl<Scalar> StateSoftContact;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef pinocchio::ModelTpl<Scalar> PinocchioModel;

  IAMSoftContact3DAugmentedTpl(
      boost::shared_ptr<DAMSoftContact3DAugmentedFwdDynamics> model,
      const Scalar& time_step = Scalar(1e-3),
      const bool& with_cost_residual = true);
  virtual ~IAMSoftContact3DAugmentedTpl();

  virtual void calc(const boost::shared_ptr<ActionDataAbstract>& data,
                    const Eigen::Ref<const VectorXs>& y,
                    const Eigen::Ref<const VectorXs>& w);

  virtual void calc(const boost::shared_ptr<ActionDataAbstract>& data,
                    const Eigen::Ref<const VectorXs>& y);

  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& y,
                        const Eigen::Ref<const VectorXs>& w);

  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& y);

  virtual boost::shared_ptr<ActionDataAbstract> createData();

  virtual bool checkData(const boost::shared_ptr<ActionDataAbstract>& data);

//   virtual void quasiStatic(const boost::shared_ptr<ActionDataAbstract>& data,
//                            Eigen::Ref<VectorXs> u,
//                            const Eigen::Ref<const VectorXs>& x,
//                            const std::size_t& maxiter = 100,
//                            const Scalar& tol = Scalar(1e-9));

  const boost::shared_ptr<DAMSoftContact3DAugmentedFwdDynamics>& get_differential()
      const;
  const Scalar& get_dt() const;

  const std::size_t& get_nc() const { return nc_; };
  const std::size_t& get_ny() const { return ny_; };

  void set_dt(const Scalar& dt);
  void set_differential(boost::shared_ptr<DAMSoftContact3DAugmentedFwdDynamics> model);

 protected:
  using Base::has_control_limits_;  //!< Indicates whether any of the control
                                    //!< limits are active
  using Base::nr_;                  //!< Dimension of the cost residual
  using Base::nu_;                  //!< Control dimension
  using Base::u_lb_;                //!< Lower control limits
  using Base::u_ub_;                //!< Upper control limits
  using Base::unone_;               //!< Neutral state
  std::size_t nc_;                  //!< Contact model dimension
  std::size_t ny_;                  //!< Augmented state dimension : nq+nv+ntau
  using Base::state_;               //!< Model of the state

 private:
  boost::shared_ptr<DAMSoftContact3DAugmentedFwdDynamics> differential_;
  Scalar time_step_;
  Scalar time_step2_;
  bool with_cost_residual_;
  boost::shared_ptr<PinocchioModel> pin_model_;  //!< for reg cost
  bool is_terminal_;  //!< is it a terminal model or not ? (deactivate cost on w
                      //!< if true)
};

template <typename _Scalar>
struct IADSoftContact3DAugmentedTpl : public ActionDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ActionDataAbstractTpl<Scalar> Base;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef pinocchio::DataTpl<Scalar> PinocchioData;
  typedef DifferentialActionDataAbstractTpl<Scalar>
      DifferentialActionDataAbstract;
  typedef DADSoftContact3DAugmentedFwdDynamicsTpl<Scalar>
      DADSoftContact3DAugmentedFwdDynamics;

  template <template <typename Scalar> class Model>
  explicit IADSoftContact3DAugmentedTpl(Model<Scalar>* const model)
      : Base(model), tau_tmp(model->get_nu()) {
    tau_tmp.setZero();
    differential = model->get_differential()->createData();
    const std::size_t& ndy = model->get_state()->get_ndx();
    dy = VectorXs::Zero(ndy);
  }
  virtual ~IADSoftContact3DAugmentedTpl() {}

  boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> > differential;
  VectorXs dy;

  using Base::cost;
  using Base::r;
  VectorXs tau_tmp;
  // use refs to "alias" base class member names
  VectorXs& ynext = Base::xnext;
  MatrixXs& Fy = Base::Fx;
//   MatrixXs& Fw = Base::Fu;
  VectorXs& Ly = Base::Lx;
//   VectorXs& Lw = Base::Lu;
  MatrixXs& Lyy = Base::Lxx;
  MatrixXs& Lyu = Base::Lxu;
//   MatrixXs& Lww = Base::Luu;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/crocomplements/softcontact/iam3d-augmented.hxx"

#endif  // SOBEC_IAM3D_AUGMENTED_HPP_

///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_RESIDUAL_2D_SURFACE_HPP_
#define SOBEC_RESIDUAL_2D_SURFACE_HPP_

#include <crocoddyl/core/residual-base.hpp>
#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;
/**
 * @brief Cost penalizing the position of one effector with respect to the other
 *
 * The cost is r(q,v) = A * (pos_2 - pos_1) - c
 * with pos_1, pos_2 (x,y) translations of feet, and A, C
 * matrix and vector transcribing the restricted 2D surface through inequalities
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModel2DSurfaceTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualData2DSurfaceTpl<Scalar> Data;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::Vector2s Vector2s;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::Matrix3s Matrix3s;

  /**
   * @brief Initialize the residual model
   *
   * @param[in] state  State of the multibody system
   * @param[in] frame_id ID of the frame that should be considered for altitude
   * and velocity
   * @param[in] slope  Slope value, ie altitude multiplier.
   * @param[in] nu     Dimension of the control vector
   */
  ResidualModel2DSurfaceTpl(boost::shared_ptr<StateMultibody> state,
                          const pinocchio::FrameIndex frame_id,
                          const Vector2s support_translation,
                          const Scalar separation,
                          const Scalar orientation,
                          const std::size_t nu);

  /**
   * @brief Initialize the residual model
   *
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state  State of the multibody system
   * @param[in] frame_id ID of the frame that should be considered for altitude
   * and velocity
   * @param[in] slope  Slope value, ie altitude multiplier.
   */
  ResidualModel2DSurfaceTpl(boost::shared_ptr<StateMultibody> state,
                          const pinocchio::FrameIndex frame_id,
                          const Vector2s support_translation,
                          const Scalar separation,
                          const Scalar orientation);
  virtual ~ResidualModel2DSurfaceTpl();

  /**
   * @brief Compute the residual
   *
   * @param[in] data  CoM velocity residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                    const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the CoM velocity residual
   *
   * @param[in] data  CoM velocity residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);
  virtual boost::shared_ptr<ResidualDataAbstract> createData(
      DataCollectorAbstract* const data);

  /**
   * @brief Return the frame index.
   */
  const pinocchio::FrameIndex& get_frame_id() const;

  /**
   * @brief Modify the frame index.
   */
  void set_frame_id(const pinocchio::FrameIndex& fid);
  
  /**
   * @brief Modify the matrix A and vector b
   */
  
  void set_Ab(
    const Vector2s support_translation,
    const Scalar orientation);
 
  const MatrixXs& get_A() const {return A_;}
  const Scalar& get_b() const {return b_;}
  
  void set_A(const MatrixXs& A) { A_ = A;}
  void set_b(const Scalar& b) { b_ = b;}
 
 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  pinocchio::FrameIndex frame_id;
  boost::shared_ptr<typename StateMultibody::PinocchioModel> pin_model_;  //!< Pinocchio model used for internal computations
  Vector2s support_translation_; // Coordinates of the support foot
  Scalar separation_; // Separation between flying foot and support foot
  Scalar orientation_; // Angle of support foot
  Vector2s pointA_;
  Vector2s pointB_;
  MatrixXs A_;
  Scalar b_;
};

template <typename _Scalar>
struct ResidualData2DSurfaceTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Matrix6xs Matrix6xs;
  typedef typename MathBase::Matrix3xs Matrix3xs;
  typedef typename MathBase::VectorXs VectorXs;

  template <template <typename Scalar> class Model>
  ResidualData2DSurfaceTpl(Model<Scalar>* const model,
                         DataCollectorAbstract* const data)
      : Base(model, data), fJf(6, model->get_state()->get_nv()) {
	fJf.setZero();
    //  Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d =
        dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty(
          "Invalid argument: the shared data should be derived from "
          "DataCollectorMultibody");
    }

    // Avoids data casting at runtime
    pinocchio = d->pinocchio;
  }

  pinocchio::DataTpl<Scalar>* pinocchio;  //!< Pinocchio data
  Matrix6xs fJf;                          //!< Local Jacobian of the frame

  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */

#include "sobec/crocomplements/residual-2D-surface.hxx"

#endif  // SOBEC_RESIDUAL_2D_surface_HPP_

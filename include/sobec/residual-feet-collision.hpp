///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_RESIDUAL_FEET_COLLISION_HPP_
#define SOBEC_RESIDUAL_FEET_COLLISION_HPP_

#include <crocoddyl/core/residual-base.hpp>
#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;
/**
 * @brief Cost penalizing distance between two frames r=||f1.translation-f2.translation||
 *
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModelFeetCollisionTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataFeetCollisionTpl<Scalar> Data;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::VectorXs VectorXs;

  /**
   * @brief Initialize the residual model
   *
   * @param[in] state  State of the multibody system
   * @param[in] frame_id1 ID of the first frame in the collision pair
   * @param[in] frame_id2 ID of the second frame in the collision pair
   * and velocity
   * @param[in] nu     Dimension of the control vector
   */
  ResidualModelFeetCollisionTpl(boost::shared_ptr<StateMultibody> state,
                                const pinocchio::FrameIndex frame_id1,
                                const pinocchio::FrameIndex frame_id2,
                                const std::size_t nu);

  /**
   * @brief Initialize the residual model
   *
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state  State of the multibody system
   * @param[in] frame_id1 ID of the first frame in the collision pair
   * @param[in] frame_id2 ID of the second frame in the collision pair
   */
  ResidualModelFeetCollisionTpl(boost::shared_ptr<StateMultibody> state,
                                const pinocchio::FrameIndex frame_id1,
                                const pinocchio::FrameIndex frame_id2);
  virtual ~ResidualModelFeetCollisionTpl();

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

  /** @brief Return the first frame index in the collision pair. */
  const pinocchio::FrameIndex& get_frame_id1() const;
  /** @brief Return the second frame index in the collision pair. */
  const pinocchio::FrameIndex& get_frame_id2() const;

  /** @brief Set the first frame index in the collision pair. */
  void set_frame_id1(const pinocchio::FrameIndex& fid1);
  /** @brief Set the second frame index in the collision pair. */
  void set_frame_id2(const pinocchio::FrameIndex& fid2);

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  pinocchio::FrameIndex frame_id1;
  pinocchio::FrameIndex frame_id2;
  typename StateMultibody::PinocchioModel
      pin_model_;  //!< Pinocchio model used for internal computations
};

template <typename _Scalar>
struct ResidualDataFeetCollisionTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Vector3s Vector3s;
  typedef Eigen::Matrix<Scalar, 2, Eigen::Dynamic> Matrix2xs;
  typedef typename MathBase::Matrix6xs Matrix6xs;
  typedef typename MathBase::VectorXs VectorXs;

  template <template <typename Scalar> class Model>
  ResidualDataFeetCollisionTpl(Model<Scalar>* const model,
                         DataCollectorAbstract* const data)
      : Base(model, data),
        J1(6, model->get_state()->get_nv()),
        J2(6, model->get_state()->get_nv()),
        dJ(2, model->get_state()->get_nv()),
        p1p2(3)
  {
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

    J1.fill(0);
    J2.fill(0);
    dJ.fill(0);
    p1p2.fill(0);
  }

  pinocchio::DataTpl<Scalar>* pinocchio;  //!< Pinocchio data
  Matrix6xs J1,J2;
  Matrix2xs dJ;
  Vector3s p1p2;
  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */

#include "sobec/residual-feet-collision.hxx"

#endif  // SOBEC_RESIDUAL_FEET_COLLISION_HPP_

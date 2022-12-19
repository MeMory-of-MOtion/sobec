///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_RESIDUAL_FLY_HIGH_HPP_
#define SOBEC_RESIDUAL_FLY_HIGH_HPP_

#include <crocoddyl/core/residual-base.hpp>
#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
using namespace crocoddyl;
/**
 * @brief Cost penalizing high horizontal velocity near zero altitude.
 *
 * The cost is r(q,v) = v_foot[:2] / exp(slope*z_foot)
 * with v_foot = J_foot(q) vq the local-world-aligned linear velocity of the
 * considered frame velocity and z_foot(q) the altitude
 * oMfoot[frameId].translation[2] of the considered frame wrt world.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModelFlyHighTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataFlyHighTpl<Scalar> Data;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Vector3s Vector3s;
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
  ResidualModelFlyHighTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex frame_id,
                          const Scalar slope, const std::size_t nu);

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
  ResidualModelFlyHighTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex frame_id,
                          const Scalar slope);
  virtual ~ResidualModelFlyHighTpl();

  /**
   * @brief Compute the residual
   *
   * @param[in] data  CoM velocity residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the CoM velocity residual
   *
   * @param[in] data  CoM velocity residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);
  virtual boost::shared_ptr<ResidualDataAbstract> createData(DataCollectorAbstract* const data);

  /**
   * @brief Return the frame index.
   */
  const pinocchio::FrameIndex& get_frame_id() const;

  /**
   * @brief Modify the frame index.
   */
  void set_frame_id(const pinocchio::FrameIndex& fid);

  const Scalar getSlope() const { return slope; }
  void setSlope(const Scalar s) { slope = s; }

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  pinocchio::FrameIndex frame_id;
  Scalar slope;                                        // multiplication in front of the altitude in the cost
  typename StateMultibody::PinocchioModel pin_model_;  //!< Pinocchio model used for internal computations
};

template <typename _Scalar>
struct ResidualDataFlyHighTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Matrix6xs Matrix6xs;
  typedef typename MathBase::Matrix3xs Matrix3xs;
  typedef typename MathBase::VectorXs VectorXs;

  template <template <typename Scalar> class Model>
  ResidualDataFlyHighTpl(Model<Scalar>* const model, DataCollectorAbstract* const data)
      : Base(model, data),
        d_dq(6, model->get_state()->get_nv()),
        d_dv(6, model->get_state()->get_nv()),
        l_dnu_dq(6, model->get_state()->get_nv()),
        l_dnu_dv(6, model->get_state()->get_nv()),
        o_dv_dq(3, model->get_state()->get_nv()),
        o_dv_dv(3, model->get_state()->get_nv()),
        o_Jw(3, model->get_state()->get_nv()),
        vxJ(3, model->get_state()->get_nv()) {
    // dvcom_dq.setZero();
    //  Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d = dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty(
          "Invalid argument: the shared data should be derived from "
          "DataCollectorMultibody");
    }

    // Avoids data casting at runtime
    pinocchio = d->pinocchio;
    // Clean buffer as pinocchio not necessarily initialize the memory.
    d_dq.fill(0);
    d_dv.fill(0);
    l_dnu_dq.fill(0);
    l_dnu_dv.fill(0);
    o_dv_dq.fill(0);
    o_dv_dv.fill(0);
    o_Jw.fill(0);
    vxJ.fill(0);
  }

  pinocchio::DataTpl<Scalar>* pinocchio;  //!< Pinocchio data
  Matrix6xs d_dq, d_dv;
  Matrix6xs l_dnu_dq, l_dnu_dv;
  Matrix3xs o_dv_dq, o_dv_dv, o_Jw, vxJ;

  Scalar ez;
  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */

#include "sobec/crocomplements/residual-fly-high.hxx"

#endif  // SOBEC_RESIDUAL_FLY_HIGH_HPP_

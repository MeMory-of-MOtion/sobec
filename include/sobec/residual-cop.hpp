///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_RESIDUAL_COP_HPP_
#define SOBEC_RESIDUAL_COP_HPP_

#include <crocoddyl/core/residual-base.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/data/multibody.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"
#include "crocoddyl/multibody/data/contacts.hpp"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/motion.hpp>

#include "sobec/fwd.hpp"

namespace sobec {

using namespace crocoddyl;

/**
 * @brief COP residual  
 *
 * residual = [ tau_y/f_z, -tau_x/fx ]
 *
 * As described in `ResidualModelAbstractTpl()`, the residual value and its
 * Jacobians are calculated by `calc` and `calcDiff`, respectively.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModelCenterOfPressureTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataCenterOfPressureTpl<Scalar> Data;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;

  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename pinocchio::ForceTpl<Scalar> Force;

  /**
   * @brief Initialize the residual model
   *
   * @param[in] state       State of the multibody system
   * @param[in] nu          Dimension of the control vector
   * @param[in] geom_model  Pinocchio geometry model containing the collision
   * pair
   * @param[in] pair_id     Index of the collision pair in the geometry model
   * @param[in] joint_id    Index of the nearest joint on which the collision
   * link is attached
   */
  ResidualModelCenterOfPressureTpl(boost::shared_ptr<StateMultibody> state,
                                   const pinocchio::FrameIndex contact_id,
                                   const std::size_t nu);

  virtual ~ResidualModelCenterOfPressureTpl();

  /**
   * @brief Compute the cop.
   *
   * @param[in] data  residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract> &data,
                    const Eigen::Ref<const VectorXs> &x,
                    const Eigen::Ref<const VectorXs> &u);

  /**
   * @brief Compute the derivatives of residual
   *
   * @param[in] data  residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract> &data,
                        const Eigen::Ref<const VectorXs> &x,
                        const Eigen::Ref<const VectorXs> &u);

  virtual boost::shared_ptr<ResidualDataAbstract> createData(
      DataCollectorAbstract *const data);

  /**
   * @brief Return the reference contact id
   */
  pinocchio::PairIndex get_contact_id() const { return contact_id_; }
  /** @brief Set the reference contact id. */
  void set_contact_id(const pinocchio::PairIndex id) { contact_id_ = id; }

  
 protected:
  using Base::nu_;
  using Base::state_;
  using Base::unone_;

 private:
  pinocchio::FrameIndex contact_id_;
};

template <typename _Scalar>
struct ResidualDataCenterOfPressureTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef ContactModelMultipleTpl<Scalar> ContactModelMultiple;

  typedef typename MathBase::Matrix6xs Matrix6xs;

  template <template <typename Scalar> class Model>
  ResidualDataCenterOfPressureTpl(Model<Scalar> *const model,
                                  DataCollectorAbstract *const data)
    : Base(model, data) {
    // Check that proper shared data has been passed
    DataCollectorContactTpl<Scalar>* d = dynamic_cast<DataCollectorContactTpl<Scalar>*>(this->shared);
    if (d == NULL) {
      throw_pretty(
          "Invalid argument: the shared data should be derived from DataCollectorContact");
    }
    const pinocchio::FrameIndex id = model->get_contact_id();
    const boost::shared_ptr<StateMultibody>& state = boost::static_pointer_cast<StateMultibody>(model->get_state());
    std::string frame_name = state->get_pinocchio()->frames[id].name;
    bool found_contact = false;
    for (typename ContactModelMultiple::ContactDataContainer::iterator it = d->contacts->contacts.begin();
         it != d->contacts->contacts.end(); ++it) {
      if (it->second->frame == id) {
        ContactData6DTpl<Scalar>* d6d = dynamic_cast<ContactData6DTpl<Scalar>*>(it->second.get());
        if (d6d != NULL) {
          found_contact = true;
          this->contact = it->second;
          break;
        }
        throw_pretty("Domain error: there isn't defined at least a 6d contact for " + frame_name);
      }
    }
    if (!found_contact) {
      throw_pretty("Domain error: there isn't defined contact data for " + frame_name);
    }
  }
  boost::shared_ptr<ForceDataAbstractTpl<Scalar> > contact;
  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/residual-cop.hxx"

#endif  // SOBEC_RESIDUAL_COP_HPP_

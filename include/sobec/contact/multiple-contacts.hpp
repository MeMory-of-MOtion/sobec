///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2022, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_MULTIPLE_CONTACTS_HPP_
#define SOBEC_MULTIPLE_CONTACTS_HPP_

#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/contact-base.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <map>
#include <set>
#include <string>
#include <utility>

#include "sobec/contact/contact1d.hpp"
#include "sobec/contact/contact3d.hpp"
#include "sobec/fwd.hpp"

namespace sobec {
namespace newcontacts{

/**
 * @brief Define a stack of contact models
 *
 * The contact models can be defined with active and inactive status. The idea
 * behind this design choice is to be able to create a mechanism that allocates
 * the entire data needed for the computations. Then, there are designed
 * routines that update the only active contacts.
 */
template <typename _Scalar>
class ContactModelMultipleTpl
    : public crocoddyl::ContactModelMultipleTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef crocoddyl::ContactModelMultipleTpl<Scalar> Base;
  typedef crocoddyl::MathBaseTpl<Scalar> MathBase;
  typedef crocoddyl::StateMultibodyTpl<Scalar> StateMultibody;
  typedef crocoddyl::ContactDataAbstractTpl<Scalar> ContactDataAbstract;
  typedef crocoddyl::ContactDataMultipleTpl<Scalar> ContactDataMultiple;
  typedef crocoddyl::ContactModelAbstractTpl<Scalar> ContactModelAbstract;

  typedef crocoddyl::ContactItemTpl<Scalar> ContactItem;

  typedef typename MathBase::Vector2s Vector2s;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::Matrix3s Matrix3s;

  typedef std::map<std::string, boost::shared_ptr<ContactItem> >
      ContactModelContainer;
  typedef std::map<std::string, boost::shared_ptr<ContactDataAbstract> >
      ContactDataContainer;
  typedef typename pinocchio::container::aligned_vector<
      pinocchio::ForceTpl<Scalar> >::iterator ForceIterator;

  /**
   * @brief Initialize the multi-contact model
   *
   * @param[in] state  Multibody state
   * @param[in] nu     Dimension of control vector
   */
  ContactModelMultipleTpl(boost::shared_ptr<StateMultibody> state,
                          const std::size_t nu);

  /**
   * @brief Initialize the multi-contact model
   *
   * @param[in] state  Multibody state
   */
  ContactModelMultipleTpl(boost::shared_ptr<StateMultibody> state);
  ~ContactModelMultipleTpl();

  /**
   * @brief Update the Jacobian of the spatial force defined in frame coordinate
   *
   * @param[in] data   Multi-contact data
   * @param[in] df_dx  Jacobian of the spatial impulse defined in frame
   * coordinate
   * \f$\frac{\partial{}^o\underline{\boldsymbol{\lambda}}_c}{\partial\mathbf{x}}\in\mathbb{R}^{nc\times{ndx}}\f$
   * @param[in] df_du  Jacobian of the spatial impulse defined in frame
   * coordinate
   * \f$\frac{\partial{}^o\underline{\boldsymbol{\lambda}}_c}{\partial\mathbf{u}}\in\mathbb{R}^{nc\times{nu}}\f$
   */
  void updateForceDiff(const boost::shared_ptr<ContactDataMultiple>& data,
                       const boost::shared_ptr<MatrixXs> df_dx,
                       const boost::shared_ptr<MatrixXs> df_du) const;

  /**
   * @brief Update the RNEA derivatives dtau_dq by adding the skew term
   * (necessary for contacts expressed in LOCAL_WORLD_ALIGNED)
   * @brief as explained in this document :
   * https://www.overleaf.com/read/tzvrrxxtntwk
   *
   * @param[in] data   Multi-contact data
   * @param[in] pinocchio   Pinocchio data
   */
  void updateRneaDerivatives(const boost::shared_ptr<ContactDataMultiple>& data,
                             pinocchio::DataTpl<Scalar>& pinocchio) const;

  //   MatrixXs rotateJacobians(const boost::shared_ptr<MatrixXs> Jin);
};

}  // namespace newcontacts
}  // namespace sobec

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "sobec/contact/multiple-contacts.hxx"

#endif  // SOBEC_MULTIPLE_CONTACTS_HPP_

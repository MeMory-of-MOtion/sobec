///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, New York University, Max Planck
// Gesellschaft,
//                          University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include "common.hpp"
#include "factory/actuation.hpp"
#include "factory/state.hpp"
#include "factory/statesoft.hpp"

using namespace boost::unit_test;
using namespace sobec::unittest;

//----------------------------------------------------------------------------//

void test_state_dimension(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  BOOST_CHECK(static_cast<std::size_t>(state->zero().size()) == state->get_ny());
  BOOST_CHECK(static_cast<std::size_t>(state->rand().size()) == state->get_ny());
  BOOST_CHECK(state->get_ny() == (state->get_nq() + state->get_nv() + state->get_nc()));
  BOOST_CHECK(state->get_ndy() == (2 * state->get_nv() + state->get_nc()));
  BOOST_CHECK(static_cast<std::size_t>(state->get_lb().size()) == state->get_ny());
  BOOST_CHECK(static_cast<std::size_t>(state->get_ub().size()) == state->get_ny());
}

// Check that dimensions matches multibody dimensions when nc=0 (no contact force)
void test_state_dimension_0(StateSoftContactModelTypes::Type state_type) {  
  StateSoftContactModelFactory factory;
  StateModelFactory factoryMultibody; 
  const boost::shared_ptr<sobec::StateSoftContact>& state0 = factory.create(state_type, 0);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody = factoryMultibody.create(mapStateSoftContactToStateMultibody.at(state_type));
  BOOST_CHECK(state0->get_ny() == stateMultibody->get_nx());
  BOOST_CHECK(state0->get_ndy() == stateMultibody->get_ndx());
  BOOST_CHECK(state0->get_nq() == stateMultibody->get_nq());
  BOOST_CHECK(state0->get_nv() == stateMultibody->get_nv());
  BOOST_CHECK(state0->get_nv() == stateMultibody->get_nv());
}



void test_integrate_against_difference(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random states
  const Eigen::VectorXd& x1 = state->rand();
  const Eigen::VectorXd& x2 = state->rand();
  // Computing x2 by integrating its difference
  Eigen::VectorXd dx(state->get_ndy());
  state->diff(x1, x2, dx);
  Eigen::VectorXd x2i(state->get_ny());
  state->integrate(x1, dx, x2i);
  Eigen::VectorXd dxi(state->get_ndy());
  state->diff(x2i, x2, dxi);
  // Checking that both states agree
  BOOST_CHECK(dxi.isZero(1e-9));
}

// Check diff against state multibody diff when nc=0
void test_integrate_against_difference_0(StateSoftContactModelTypes::Type state_type) {
  StateSoftContactModelFactory factory;
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateSoftContact>& state0 = factory.create(state_type, 0);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateSoftContactToStateMultibody.at(state_type));
  // Generating random states
  const Eigen::VectorXd& x1 = state0->rand();
  const Eigen::VectorXd& x2 = state0->rand();
  // Difference SoftContact
  Eigen::VectorXd dxSoft(state0->get_ndy());
  state0->diff(x1, x2, dxSoft);
  // Difference multibody
  Eigen::VectorXd dxMultibody(stateMultibody->get_ndx());
  stateMultibody->diff(x1, x2, dxMultibody);
  BOOST_CHECK((dxSoft - dxMultibody).isZero(1e-9));
}



void test_difference_against_integrate(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state =
      factory.create(state_type, nc);
  // Generating random states
  const Eigen::VectorXd& x = state->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(state->get_ndy());
  // Computing dx by differentiation of its integrate
  Eigen::VectorXd xidx(state->get_ny());
  state->integrate(x, dx, xidx);
  Eigen::VectorXd dxd(state->get_ndy());
  state->diff(x, xidx, dxd);
  // Checking that both states agree
  BOOST_CHECK((dxd - dx).isZero(1e-9));
}

// Check integrate against state multibody integrate when nc=0
void test_difference_against_integrate_0(StateSoftContactModelTypes::Type state_type) {
  StateSoftContactModelFactory factory;
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateSoftContact>& state0 = factory.create(state_type, 0);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateSoftContactToStateMultibody.at(state_type));
  // Generating random states
  const Eigen::VectorXd& x1 = state0->rand();
  const Eigen::VectorXd& dx1 = Eigen::VectorXd::Random(state0->get_ndy());
  // Integrate SoftContact
  Eigen::VectorXd xSoft(state0->get_ny());
  state0->integrate(x1, dx1, xSoft);
  // Integrate multibody
  Eigen::VectorXd xMultibody(state0->get_nx());
  stateMultibody->integrate(x1, dx1, xMultibody);
  BOOST_CHECK((xSoft - xMultibody).isZero(1e-9));
}



void test_Jdiff_firstsecond(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x1 = state->rand();
  const Eigen::VectorXd& x2 = state->rand();
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_tmp(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdiff_first(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdiff_second(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jdiff(x1, x2, Jdiff_first, Jdiff_tmp, crocoddyl::first);
  state->Jdiff(x1, x2, Jdiff_tmp, Jdiff_second, crocoddyl::second);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_both_first(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdiff_both_second(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jdiff(x1, x2, Jdiff_both_first, Jdiff_both_second);
  BOOST_CHECK((Jdiff_first - Jdiff_both_first).isZero(1e-9));
  BOOST_CHECK((Jdiff_second - Jdiff_both_second).isZero(1e-9));
}


// Check Jdiff against state multibody when nc=0
void test_Jdiff_firstsecond_0(StateSoftContactModelTypes::Type state_type) {
  StateSoftContactModelFactory factory;
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateSoftContact>& state0 = factory.create(state_type, 0);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateSoftContactToStateMultibody.at(state_type));
  const Eigen::VectorXd& x3 = state0->rand();
  const Eigen::VectorXd& x4 = state0->rand();
  // Jdiff SoftContact
  Eigen::MatrixXd Jdiff_tmpSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  Eigen::MatrixXd Jdiff_firstSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  Eigen::MatrixXd Jdiff_secondSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  state0->Jdiff(x3, x4, Jdiff_firstSoftContact, Jdiff_tmpSoftContact, crocoddyl::first);
  state0->Jdiff(x3, x4, Jdiff_tmpSoftContact, Jdiff_secondSoftContact, crocoddyl::second);
  Eigen::MatrixXd Jdiff_both_firstSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  Eigen::MatrixXd Jdiff_both_secondSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  state0->Jdiff(x3, x4, Jdiff_both_firstSoftContact, Jdiff_both_secondSoftContact);
  // Jdiff multibody
  Eigen::MatrixXd Jdiff_tmpMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jdiff_firstMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jdiff_secondMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jdiff(x3, x4, Jdiff_firstMultibody, Jdiff_tmpMultibody,
                        crocoddyl::first);
  stateMultibody->Jdiff(x3, x4, Jdiff_tmpMultibody, Jdiff_secondMultibody,
                        crocoddyl::second);
  Eigen::MatrixXd Jdiff_both_firstMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jdiff_both_secondMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jdiff(x3, x4, Jdiff_both_firstMultibody,
                        Jdiff_both_secondMultibody);

  BOOST_CHECK((Jdiff_firstSoftContact - Jdiff_firstMultibody).isZero(1e-9));
  BOOST_CHECK((Jdiff_secondSoftContact - Jdiff_secondMultibody).isZero(1e-9));
  BOOST_CHECK((Jdiff_both_firstSoftContact - Jdiff_both_firstMultibody).isZero(1e-9));
  BOOST_CHECK((Jdiff_both_secondSoftContact - Jdiff_both_secondMultibody).isZero(1e-9));
}



void test_Jint_firstsecond(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x = state->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(state->get_ndy());
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jint_tmp(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jint_first(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jint_second(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jintegrate(x, dx, Jint_first, Jint_tmp, crocoddyl::first);
  state->Jintegrate(x, dx, Jint_tmp, Jint_second, crocoddyl::second);
  // Computing the partial derivatives of the integrate function separately
  Eigen::MatrixXd Jint_both_first(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jint_both_second(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jintegrate(x, dx, Jint_both_first, Jint_both_second);
  BOOST_CHECK((Jint_first - Jint_both_first).isZero(1e-9));
  BOOST_CHECK((Jint_second - Jint_both_second).isZero(1e-9));
}

// Check Jintegrate against state multibody Jintegrate when nc=0
void test_Jint_firstsecond_0(StateSoftContactModelTypes::Type state_type) {
  StateSoftContactModelFactory factory;
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateSoftContact>& state0 = factory.create(state_type, 0);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateSoftContactToStateMultibody.at(state_type));
  const Eigen::VectorXd& x1 = state0->rand();
  const Eigen::VectorXd& dx1 = Eigen::VectorXd::Random(state0->get_ndy());
  // Jint SoftContact
  Eigen::MatrixXd Jint_tmpSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  Eigen::MatrixXd Jint_firstSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  Eigen::MatrixXd Jint_secondSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  state0->Jintegrate(x1, dx1, Jint_firstSoftContact, Jint_tmpSoftContact,
                           crocoddyl::first);
  state0->Jintegrate(x1, dx1, Jint_tmpSoftContact, Jint_secondSoftContact,
                           crocoddyl::second);
  Eigen::MatrixXd Jint_both_firstSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  Eigen::MatrixXd Jint_both_secondSoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  state0->Jintegrate(x1, dx1, Jint_both_firstSoftContact, Jint_both_secondSoftContact);
  // Jint multibody
  Eigen::MatrixXd Jint_tmpMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jint_firstMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jint_secondMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jintegrate(x1, dx1, Jint_firstMultibody, Jint_tmpMultibody,
                             crocoddyl::first);
  stateMultibody->Jintegrate(x1, dx1, Jint_tmpMultibody, Jint_secondMultibody,
                             crocoddyl::second);
  Eigen::MatrixXd Jint_both_firstMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jint_both_secondMultibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jintegrate(x1, dx1, Jint_both_firstMultibody,
                             Jint_both_secondMultibody);

  BOOST_CHECK((Jint_firstSoftContact - Jint_firstMultibody).isZero(1e-9));
  BOOST_CHECK((Jint_secondSoftContact - Jint_secondMultibody).isZero(1e-9));
  BOOST_CHECK((Jint_both_firstSoftContact - Jint_both_firstMultibody).isZero(1e-9));
  BOOST_CHECK((Jint_both_secondSoftContact - Jint_both_secondMultibody).isZero(1e-9));
}



void test_Jdiff_num_diff_firstsecond(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x1 = state->rand();
  const Eigen::VectorXd& x2 = state->rand();
  // Get the num diff state
  crocoddyl::StateNumDiff state_num_diff(state);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_num_diff_tmp(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdiff_num_diff_first(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdiff_num_diff_second(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state_num_diff.Jdiff(x1, x2, Jdiff_num_diff_first, Jdiff_num_diff_tmp,
                       crocoddyl::first);
  state_num_diff.Jdiff(x1, x2, Jdiff_num_diff_tmp, Jdiff_num_diff_second,
                       crocoddyl::second);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_num_diff_both_first(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdiff_num_diff_both_second(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state_num_diff.Jdiff(x1, x2, Jdiff_num_diff_both_first,
                       Jdiff_num_diff_both_second);
  BOOST_CHECK((Jdiff_num_diff_first - Jdiff_num_diff_both_first).isZero(1e-9));
  BOOST_CHECK((Jdiff_num_diff_second - Jdiff_num_diff_both_second).isZero(1e-9));
}



void test_Jint_num_diff_firstsecond(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x = state->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(state->get_ndy());
  // Get the num diff state
  crocoddyl::StateNumDiff state_num_diff(state);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jint_num_diff_tmp(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jint_num_diff_first(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jint_num_diff_second(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state_num_diff.Jintegrate(x, dx, Jint_num_diff_first, Jint_num_diff_tmp,
                            crocoddyl::first);
  state_num_diff.Jintegrate(x, dx, Jint_num_diff_tmp, Jint_num_diff_second,
                            crocoddyl::second);
  // Computing the partial derivatives of the given function separately
  Eigen::MatrixXd Jint_num_diff_both_first(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jint_num_diff_both_second(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state_num_diff.Jintegrate(x, dx, Jint_num_diff_both_first,
                            Jint_num_diff_both_second);
  BOOST_CHECK((Jint_num_diff_first - Jint_num_diff_both_first).isZero(1e-9));
  BOOST_CHECK((Jint_num_diff_second - Jint_num_diff_both_second).isZero(1e-9));
}



void test_Jdiff_against_numdiff(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x1 = state->rand();
  const Eigen::VectorXd& x2 = state->rand();
  // Computing the partial derivatives of the difference function analytically
  Eigen::MatrixXd Jdiff_1(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdiff_2(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jdiff(x1, x2, Jdiff_1, Jdiff_2, crocoddyl::first);
  state->Jdiff(x1, x2, Jdiff_1, Jdiff_2, crocoddyl::second);
  // Computing the partial derivatives of the difference function numerically
  crocoddyl::StateNumDiff state_num_diff(state);
  Eigen::MatrixXd Jdiff_num_1(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdiff_num_2(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state_num_diff.Jdiff(x1, x2, Jdiff_num_1, Jdiff_num_2);
  // Checking the partial derivatives against NumDiff
  // The previous tolerance was 10*disturbance
  double tol = NUMDIFF_MODIFIER * sqrt(state_num_diff.get_disturbance());
  BOOST_CHECK((Jdiff_1 - Jdiff_num_1).isZero(tol));
  BOOST_CHECK((Jdiff_2 - Jdiff_num_2).isZero(tol));
}



void test_Jintegrate_against_numdiff(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random values for the initial state and its rate of change
  const Eigen::VectorXd& x = state->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(state->get_ndy());
  // Computing the partial derivatives of the difference function analytically
  Eigen::MatrixXd Jint_1(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jint_2(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jintegrate(x, dx, Jint_1, Jint_2);
  // Computing the partial derivatives of the difference function numerically
  crocoddyl::StateNumDiff state_num_diff(state);
  Eigen::MatrixXd Jint_num_1(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jint_num_2(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state_num_diff.Jintegrate(x, dx, Jint_num_1, Jint_num_2);
  // Checking the partial derivatives against NumDiff
  // The previous tolerance was 10*disturbance
  double tol = sqrt(state_num_diff.get_disturbance());
  BOOST_CHECK((Jint_1 - Jint_num_1).isZero(tol));
  BOOST_CHECK((Jint_2 - Jint_num_2).isZero(tol));

  // Check Jdiff against state multibody when nc=0
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateSoftContact>& state0 =
      factory.create(state_type, 0);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateSoftContactToStateMultibody.at(state_type));
  Eigen::MatrixXd Jint_1SoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  Eigen::MatrixXd Jint_2SoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  state0->Jintegrate(x, dx, Jint_1SoftContact, Jint_2SoftContact);
  Eigen::MatrixXd Jint_1Multibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jint_2Multibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jintegrate(x, dx, Jint_1Multibody, Jint_2Multibody);
  BOOST_CHECK((Jint_1SoftContact - Jint_1Multibody).isZero(tol));
  BOOST_CHECK((Jint_2SoftContact - Jint_2Multibody).isZero(tol));
}

// Check Jintegrate against state multibody when nc=0
void test_Jintegrate_against_numdiff_0(StateSoftContactModelTypes::Type state_type) {
  StateSoftContactModelFactory factory;
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateSoftContact>& state0 = factory.create(state_type, 0);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateSoftContactToStateMultibody.at(state_type));
  const Eigen::VectorXd& x = state0->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(state0->get_ndy());
  Eigen::MatrixXd Jint_1SoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  Eigen::MatrixXd Jint_2SoftContact(
      Eigen::MatrixXd::Zero(state0->get_ndy(), state0->get_ndy()));
  state0->Jintegrate(x, dx, Jint_1SoftContact, Jint_2SoftContact);
  Eigen::MatrixXd Jint_1Multibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jint_2Multibody(Eigen::MatrixXd::Zero(
      stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jintegrate(x, dx, Jint_1Multibody, Jint_2Multibody);
  double tol = 1e-8;
  BOOST_CHECK((Jint_1SoftContact - Jint_1Multibody).isZero(tol));
  BOOST_CHECK((Jint_2SoftContact - Jint_2Multibody).isZero(tol));
}



void test_JintegrateTransport(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random values for the initial state and its rate of change
  const Eigen::VectorXd& x = state->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(state->get_ndy());
  // Computing the partial derivatives of the difference function analytically
  Eigen::MatrixXd Jint_1(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jint_2(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jintegrate(x, dx, Jint_1, Jint_2);
  Eigen::MatrixXd Jref(Eigen::MatrixXd::Random(state->get_ndy(),
                                               2 * state->get_ndy()));
  const Eigen::MatrixXd Jtest(Jref);
  state->JintegrateTransport(x, dx, Jref, crocoddyl::first);
  BOOST_CHECK((Jref - Jint_1 * Jtest).isZero(1e-10));
  Jref = Jtest;
  state->JintegrateTransport(x, dx, Jref, crocoddyl::second);
  BOOST_CHECK((Jref - Jint_2 * Jtest).isZero(1e-10));
}

// Check JintegrateTransport against state multibody JintegrateTransport when nc=0
void test_JintegrateTransport_0(StateSoftContactModelTypes::Type state_type) {
  StateSoftContactModelFactory factory;
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateSoftContact>& state0 = factory.create(state_type, 0);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateSoftContactToStateMultibody.at(state_type));
  const Eigen::VectorXd& x1 = state0->rand();
  const Eigen::VectorXd& dx1 = Eigen::VectorXd::Random(state0->get_ndy());
  Eigen::MatrixXd JrefSoftContact(Eigen::MatrixXd::Random(state0->get_ndy(),
                                                  2 * state0->get_ndy()));
  Eigen::MatrixXd JrefMultibody(JrefSoftContact);
  const Eigen::MatrixXd Jtest1(JrefSoftContact);
  // test first
  state0->JintegrateTransport(x1, dx1, JrefSoftContact, crocoddyl::first);
  stateMultibody->JintegrateTransport(x1, dx1, JrefMultibody, crocoddyl::first);
  BOOST_CHECK((JrefSoftContact - JrefMultibody).isZero(1e-10));
  // reset
  JrefSoftContact = Jtest1;
  JrefMultibody = Jtest1;
  // test second
  state0->JintegrateTransport(x1, dx1, JrefSoftContact, crocoddyl::second);
  stateMultibody->JintegrateTransport(x1, dx1, JrefMultibody,
                                      crocoddyl::second);
  BOOST_CHECK((JrefSoftContact - JrefMultibody).isZero(1e-10));
}



void test_Jdiff_and_Jintegrate_are_inverses(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random states
  const Eigen::VectorXd& x1 = state->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(state->get_ndy());
  Eigen::VectorXd x2(state->get_ny());
  state->integrate(x1, dx, x2);
  // Computing the partial derivatives of the integrate and difference function
  Eigen::MatrixXd Jx(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdx(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jintegrate(x1, dx, Jx, Jdx);
  Eigen::MatrixXd J1(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd J2(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jdiff(x1, x2, J1, J2);
  // Checking that Jdiff and Jintegrate are inverses
  Eigen::MatrixXd dX_dDX = Jdx;
  Eigen::MatrixXd dDX_dX = J2;
  BOOST_CHECK((dX_dDX - dDX_dX.inverse()).isZero(1e-9));
}



void test_velocity_from_Jintegrate_Jdiff(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  StateSoftContactModelFactory factory;
  const boost::shared_ptr<sobec::StateSoftContact>& state = factory.create(state_type, nc);
  // Generating random states
  const Eigen::VectorXd& x1 = state->rand();
  Eigen::VectorXd dx = Eigen::VectorXd::Random(state->get_ndy());
  Eigen::VectorXd x2(state->get_ny());
  state->integrate(x1, dx, x2);
  Eigen::VectorXd eps = Eigen::VectorXd::Random(state->get_ndy());
  double h = 1e-8;
  // Computing the partial derivatives of the integrate and difference function
  Eigen::MatrixXd Jx(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd Jdx(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jintegrate(x1, dx, Jx, Jdx);
  Eigen::MatrixXd J1(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  Eigen::MatrixXd J2(
      Eigen::MatrixXd::Zero(state->get_ndy(), state->get_ndy()));
  state->Jdiff(x1, x2, J1, J2);
  // Checking that computed velocity from Jintegrate
  const Eigen::MatrixXd& dX_dDX = Jdx;
  Eigen::VectorXd x2eps(state->get_ny());
  state->integrate(x1, dx + eps * h, x2eps);
  Eigen::VectorXd x2_eps(state->get_ndy());
  state->diff(x2, x2eps, x2_eps);
  BOOST_CHECK((dX_dDX * eps - x2_eps / h).isZero(1e-3));
  // Checking the velocity computed from Jdiff
  const Eigen::VectorXd& x = state->rand();
  dx.setZero();
  state->diff(x1, x, dx);
  Eigen::VectorXd x2i(state->get_ny());
  state->integrate(x, eps * h, x2i);
  Eigen::VectorXd dxi(state->get_ndy());
  state->diff(x1, x2i, dxi);
  J1.setZero();
  J2.setZero();
  state->Jdiff(x1, x, J1, J2);
  BOOST_CHECK((J2 * eps - (-dx + dxi) / h).isZero(1e-3));
}

//----------------------------------------------------------------------------//

void register_state_unit_tests(StateSoftContactModelTypes::Type state_type, std::size_t nc) {
  boost::test_tools::output_test_stream test_name;
  test_name << "test_" << state_type << "_" << nc;
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  ts->add(BOOST_TEST_CASE(boost::bind(&test_state_dimension, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_integrate_against_difference, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_difference_against_integrate, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jdiff_firstsecond, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jint_firstsecond, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jdiff_num_diff_firstsecond, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jint_num_diff_firstsecond, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jdiff_against_numdiff, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jintegrate_against_numdiff, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_JintegrateTransport, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jdiff_and_Jintegrate_are_inverses, state_type, nc)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_velocity_from_Jintegrate_Jdiff, state_type, nc)));
  framework::master_test_suite().add(ts);
}

void register_state_unit_tests_0(StateSoftContactModelTypes::Type state_type) {
  boost::test_tools::output_test_stream test_name;
  test_name << "test_" << state_type << "_0";
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  ts->add(BOOST_TEST_CASE(boost::bind(&test_state_dimension_0, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_integrate_against_difference_0, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_difference_against_integrate_0, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jdiff_firstsecond_0, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jint_firstsecond_0, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jintegrate_against_numdiff_0, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_JintegrateTransport_0, state_type)));
  framework::master_test_suite().add(ts);
}

bool init_function() {
  // test state SoftContact functions
  for (size_t i = 0; i < StateSoftContactModelTypes::all.size(); ++i) {
    for (size_t nc = 0; nc<=3; nc++){
        // Test nc=0 matches multibody
        if(nc==0){
            register_state_unit_tests_0(StateSoftContactModelTypes::all[i]);
        }
        // Test nc=1,2,3
        else{
            register_state_unit_tests(StateSoftContactModelTypes::all[i], nc);
        }
    }
  }
  return true;
}

int main(int argc, char** argv) {
  return ::boost::unit_test::unit_test_main(&init_function, argc, argv);
}

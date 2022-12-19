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
#include "factory/statelpf.hpp"

using namespace boost::unit_test;
using namespace sobec::unittest;

//----------------------------------------------------------------------------//

void test_state_dimension(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Checking the dimension of zero and random states
  BOOST_CHECK(static_cast<std::size_t>(stateLPFAll->zero().size()) == stateLPFAll->get_ny());
  BOOST_CHECK(static_cast<std::size_t>(stateLPFAll->rand().size()) == stateLPFAll->get_ny());
  BOOST_CHECK(stateLPFAll->get_ny() == (stateLPFAll->get_nq() + stateLPFAll->get_nv() + stateLPFAll->get_ntau()));
  BOOST_CHECK(stateLPFAll->get_ndy() == (2 * stateLPFAll->get_nv() + stateLPFAll->get_ntau()));
  BOOST_CHECK(static_cast<std::size_t>(stateLPFAll->get_lb().size()) == stateLPFAll->get_ny());
  BOOST_CHECK(static_cast<std::size_t>(stateLPFAll->get_ub().size()) == stateLPFAll->get_ny());

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  BOOST_CHECK(static_cast<std::size_t>(stateLPFRand->zero().size()) == stateLPFRand->get_ny());
  BOOST_CHECK(static_cast<std::size_t>(stateLPFRand->rand().size()) == stateLPFRand->get_ny());
  BOOST_CHECK(stateLPFRand->get_ny() == (stateLPFRand->get_nq() + stateLPFRand->get_nv() + stateLPFRand->get_ntau()));
  BOOST_CHECK(stateLPFRand->get_ndy() == (2 * stateLPFRand->get_nv() + stateLPFRand->get_ntau()));
  BOOST_CHECK(static_cast<std::size_t>(stateLPFRand->get_lb().size()) == stateLPFRand->get_ny());
  BOOST_CHECK(static_cast<std::size_t>(stateLPFRand->get_ub().size()) == stateLPFRand->get_ny());

  // Check that dimensions matches multibody dimensions when nu=0 (no LPF joint)
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFNone = factory.create(state_type, LPFJointMaskType::NONE);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateLPFToStateMultibody.at(state_type));
  BOOST_CHECK(stateLPFNone->get_ny() == stateMultibody->get_nx());
  BOOST_CHECK(stateLPFNone->get_ndy() == stateMultibody->get_ndx());
  BOOST_CHECK(stateLPFNone->get_nq() == stateMultibody->get_nq());
  BOOST_CHECK(stateLPFNone->get_nv() == stateMultibody->get_nv());
  BOOST_CHECK(stateLPFNone->get_nv() == stateMultibody->get_nv());
}

void test_integrate_against_difference(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random states
  const Eigen::VectorXd& x1 = stateLPFAll->rand();
  const Eigen::VectorXd& x2 = stateLPFAll->rand();
  // Computing x2 by integrating its difference
  Eigen::VectorXd dx(stateLPFAll->get_ndy());
  stateLPFAll->diff(x1, x2, dx);
  Eigen::VectorXd x2i(stateLPFAll->get_ny());
  stateLPFAll->integrate(x1, dx, x2i);
  Eigen::VectorXd dxi(stateLPFAll->get_ndy());
  stateLPFAll->diff(x2i, x2, dxi);
  // Checking that both states agree
  BOOST_CHECK(dxi.isZero(1e-9));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random states
  const Eigen::VectorXd& x1rand = stateLPFRand->rand();
  const Eigen::VectorXd& x2rand = stateLPFRand->rand();
  // Computing x2 by integrating its difference
  Eigen::VectorXd dxrand(stateLPFRand->get_ndy());
  stateLPFRand->diff(x1rand, x2rand, dxrand);
  Eigen::VectorXd x2irand(stateLPFRand->get_ny());
  stateLPFRand->integrate(x1rand, dxrand, x2irand);
  Eigen::VectorXd dxirand(stateLPFRand->get_ndy());
  stateLPFRand->diff(x2irand, x2rand, dxirand);
  // Checking that both states agree
  BOOST_CHECK(dxirand.isZero(1e-9));

  // Check diff against state multibody diff when nu=0
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFNone = factory.create(state_type, LPFJointMaskType::NONE);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateLPFToStateMultibody.at(state_type));
  // Generating random states
  const Eigen::VectorXd& x3 = stateLPFNone->rand();
  const Eigen::VectorXd& x4 = stateLPFNone->rand();
  // Difference LPF
  Eigen::VectorXd dxLPF(stateLPFNone->get_ndy());
  stateLPFNone->diff(x3, x4, dxLPF);
  // Difference multibody
  Eigen::VectorXd dxMultibody(stateMultibody->get_ndx());
  stateMultibody->diff(x3, x4, dxMultibody);
  BOOST_CHECK((dxLPF - dxMultibody).isZero(1e-9));
}

void test_difference_against_integrate(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random states
  const Eigen::VectorXd& x = stateLPFAll->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(stateLPFAll->get_ndy());
  // Computing dx by differentiation of its integrate
  Eigen::VectorXd xidx(stateLPFAll->get_ny());
  stateLPFAll->integrate(x, dx, xidx);
  Eigen::VectorXd dxd(stateLPFAll->get_ndy());
  stateLPFAll->diff(x, xidx, dxd);
  // Checking that both states agree
  BOOST_CHECK((dxd - dx).isZero(1e-9));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  const Eigen::VectorXd& xrand = stateLPFRand->rand();
  const Eigen::VectorXd& dxrand = Eigen::VectorXd::Random(stateLPFRand->get_ndy());
  // Computing dx by differentiation of its integrate
  Eigen::VectorXd xidxrand(stateLPFRand->get_ny());
  stateLPFRand->integrate(xrand, dxrand, xidxrand);
  Eigen::VectorXd dxdrand(stateLPFRand->get_ndy());
  stateLPFRand->diff(xrand, xidxrand, dxdrand);
  // Checking that both states agree
  BOOST_CHECK((dxdrand - dxrand).isZero(1e-9));

  // Check integrate against state multibody integrate when nu=0
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFNone = factory.create(state_type, LPFJointMaskType::NONE);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateLPFToStateMultibody.at(state_type));
  // Generating random states
  const Eigen::VectorXd& x1 = stateLPFNone->rand();
  const Eigen::VectorXd& dx1 = Eigen::VectorXd::Random(stateLPFNone->get_ndy());
  // Integrate LPF
  Eigen::VectorXd xLPF(stateLPFNone->get_ny());
  stateLPFNone->integrate(x1, dx1, xLPF);
  // Integrate multibody
  Eigen::VectorXd xMultibody(stateLPFNone->get_nx());
  stateMultibody->integrate(x1, dx1, xMultibody);
  BOOST_CHECK((xLPF - xMultibody).isZero(1e-9));
}

void test_Jdiff_firstsecond(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x1 = stateLPFAll->rand();
  const Eigen::VectorXd& x2 = stateLPFAll->rand();
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_tmp(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdiff_first(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdiff_second(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jdiff(x1, x2, Jdiff_first, Jdiff_tmp, crocoddyl::first);
  stateLPFAll->Jdiff(x1, x2, Jdiff_tmp, Jdiff_second, crocoddyl::second);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_both_first(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdiff_both_second(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jdiff(x1, x2, Jdiff_both_first, Jdiff_both_second);
  BOOST_CHECK((Jdiff_first - Jdiff_both_first).isZero(1e-9));
  BOOST_CHECK((Jdiff_second - Jdiff_both_second).isZero(1e-9));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x1rand = stateLPFRand->rand();
  const Eigen::VectorXd& x2rand = stateLPFRand->rand();
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_tmprand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdiff_firstrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdiff_secondrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  stateLPFRand->Jdiff(x1rand, x2rand, Jdiff_firstrand, Jdiff_tmprand, crocoddyl::first);
  stateLPFRand->Jdiff(x1rand, x2rand, Jdiff_tmprand, Jdiff_secondrand, crocoddyl::second);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_both_firstrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdiff_both_secondrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  stateLPFRand->Jdiff(x1rand, x2rand, Jdiff_both_firstrand, Jdiff_both_secondrand);
  BOOST_CHECK((Jdiff_firstrand - Jdiff_both_firstrand).isZero(1e-9));
  BOOST_CHECK((Jdiff_secondrand - Jdiff_both_secondrand).isZero(1e-9));

  // Check Jdiff against state multibody when nu=0
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFNone = factory.create(state_type, LPFJointMaskType::NONE);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateLPFToStateMultibody.at(state_type));
  const Eigen::VectorXd& x3 = stateLPFNone->rand();
  const Eigen::VectorXd& x4 = stateLPFNone->rand();
  // Jdiff LPF
  Eigen::MatrixXd Jdiff_tmpLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  Eigen::MatrixXd Jdiff_firstLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  Eigen::MatrixXd Jdiff_secondLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  stateLPFNone->Jdiff(x3, x4, Jdiff_firstLPF, Jdiff_tmpLPF, crocoddyl::first);
  stateLPFNone->Jdiff(x3, x4, Jdiff_tmpLPF, Jdiff_secondLPF, crocoddyl::second);
  Eigen::MatrixXd Jdiff_both_firstLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  Eigen::MatrixXd Jdiff_both_secondLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  stateLPFNone->Jdiff(x3, x4, Jdiff_both_firstLPF, Jdiff_both_secondLPF);
  // Jdiff multibody
  Eigen::MatrixXd Jdiff_tmpMultibody(Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jdiff_firstMultibody(Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jdiff_secondMultibody(Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jdiff(x3, x4, Jdiff_firstMultibody, Jdiff_tmpMultibody, crocoddyl::first);
  stateMultibody->Jdiff(x3, x4, Jdiff_tmpMultibody, Jdiff_secondMultibody, crocoddyl::second);
  Eigen::MatrixXd Jdiff_both_firstMultibody(
      Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jdiff_both_secondMultibody(
      Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jdiff(x3, x4, Jdiff_both_firstMultibody, Jdiff_both_secondMultibody);

  BOOST_CHECK((Jdiff_firstLPF - Jdiff_firstMultibody).isZero(1e-9));
  BOOST_CHECK((Jdiff_secondLPF - Jdiff_secondMultibody).isZero(1e-9));
  BOOST_CHECK((Jdiff_both_firstLPF - Jdiff_both_firstMultibody).isZero(1e-9));
  BOOST_CHECK((Jdiff_both_secondLPF - Jdiff_both_secondMultibody).isZero(1e-9));
}

void test_Jint_firstsecond(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x = stateLPFAll->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(stateLPFAll->get_ndy());
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jint_tmp(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_first(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_second(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jintegrate(x, dx, Jint_first, Jint_tmp, crocoddyl::first);
  stateLPFAll->Jintegrate(x, dx, Jint_tmp, Jint_second, crocoddyl::second);
  // Computing the partial derivatives of the integrate function separately
  Eigen::MatrixXd Jint_both_first(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_both_second(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jintegrate(x, dx, Jint_both_first, Jint_both_second);
  BOOST_CHECK((Jint_first - Jint_both_first).isZero(1e-9));
  BOOST_CHECK((Jint_second - Jint_both_second).isZero(1e-9));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& xrand = stateLPFAll->rand();
  const Eigen::VectorXd& dxrand = Eigen::VectorXd::Random(stateLPFAll->get_ndy());
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jint_tmprand(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_firstrand(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_secondrand(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jintegrate(xrand, dxrand, Jint_firstrand, Jint_tmprand, crocoddyl::first);
  stateLPFAll->Jintegrate(xrand, dxrand, Jint_tmprand, Jint_secondrand, crocoddyl::second);
  // Computing the partial derivatives of the integrate function separately
  Eigen::MatrixXd Jint_both_firstrand(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_both_secondrand(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jintegrate(xrand, dxrand, Jint_both_firstrand, Jint_both_secondrand);
  BOOST_CHECK((Jint_firstrand - Jint_both_firstrand).isZero(1e-9));
  BOOST_CHECK((Jint_secondrand - Jint_both_secondrand).isZero(1e-9));

  // Check Jintegrate against state multibody Jintegrate when nu=0
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFNone = factory.create(state_type, LPFJointMaskType::NONE);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateLPFToStateMultibody.at(state_type));
  const Eigen::VectorXd& x1 = stateLPFNone->rand();
  const Eigen::VectorXd& dx1 = Eigen::VectorXd::Random(stateLPFNone->get_ndy());
  // Jint LPF
  Eigen::MatrixXd Jint_tmpLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  Eigen::MatrixXd Jint_firstLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  Eigen::MatrixXd Jint_secondLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  stateLPFNone->Jintegrate(x1, dx1, Jint_firstLPF, Jint_tmpLPF, crocoddyl::first);
  stateLPFNone->Jintegrate(x1, dx1, Jint_tmpLPF, Jint_secondLPF, crocoddyl::second);
  Eigen::MatrixXd Jint_both_firstLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  Eigen::MatrixXd Jint_both_secondLPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  stateLPFNone->Jintegrate(x1, dx1, Jint_both_firstLPF, Jint_both_secondLPF);
  // Jint multibody
  Eigen::MatrixXd Jint_tmpMultibody(Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jint_firstMultibody(Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jint_secondMultibody(Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jintegrate(x1, dx1, Jint_firstMultibody, Jint_tmpMultibody, crocoddyl::first);
  stateMultibody->Jintegrate(x1, dx1, Jint_tmpMultibody, Jint_secondMultibody, crocoddyl::second);
  Eigen::MatrixXd Jint_both_firstMultibody(
      Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jint_both_secondMultibody(
      Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jintegrate(x1, dx1, Jint_both_firstMultibody, Jint_both_secondMultibody);

  BOOST_CHECK((Jint_firstLPF - Jint_firstMultibody).isZero(1e-9));
  BOOST_CHECK((Jint_secondLPF - Jint_secondMultibody).isZero(1e-9));
  BOOST_CHECK((Jint_both_firstLPF - Jint_both_firstMultibody).isZero(1e-9));
  BOOST_CHECK((Jint_both_secondLPF - Jint_both_secondMultibody).isZero(1e-9));
}

void test_Jdiff_num_diff_firstsecond(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x1 = stateLPFAll->rand();
  const Eigen::VectorXd& x2 = stateLPFAll->rand();
  // Get the num diff stateLPFAll
  crocoddyl::StateNumDiff state_num_diff(stateLPFAll);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_num_diff_tmp(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdiff_num_diff_first(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdiff_num_diff_second(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  state_num_diff.Jdiff(x1, x2, Jdiff_num_diff_first, Jdiff_num_diff_tmp, crocoddyl::first);
  state_num_diff.Jdiff(x1, x2, Jdiff_num_diff_tmp, Jdiff_num_diff_second, crocoddyl::second);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_num_diff_both_first(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdiff_num_diff_both_second(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  state_num_diff.Jdiff(x1, x2, Jdiff_num_diff_both_first, Jdiff_num_diff_both_second);
  BOOST_CHECK((Jdiff_num_diff_first - Jdiff_num_diff_both_first).isZero(1e-9));
  BOOST_CHECK((Jdiff_num_diff_second - Jdiff_num_diff_both_second).isZero(1e-9));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x1rand = stateLPFRand->rand();
  const Eigen::VectorXd& x2rand = stateLPFRand->rand();
  // Get the num diff stateLPFRand
  crocoddyl::StateNumDiff state_num_diffrand(stateLPFRand);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_num_diff_tmprand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdiff_num_diff_firstrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdiff_num_diff_secondrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  state_num_diffrand.Jdiff(x1rand, x2rand, Jdiff_num_diff_firstrand, Jdiff_num_diff_tmprand, crocoddyl::first);
  state_num_diffrand.Jdiff(x1rand, x2rand, Jdiff_num_diff_tmprand, Jdiff_num_diff_secondrand, crocoddyl::second);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jdiff_num_diff_both_firstrand(
      Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdiff_num_diff_both_secondrand(
      Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  state_num_diffrand.Jdiff(x1rand, x2rand, Jdiff_num_diff_both_firstrand, Jdiff_num_diff_both_secondrand);
  BOOST_CHECK((Jdiff_num_diff_firstrand - Jdiff_num_diff_both_firstrand).isZero(1e-9));
  BOOST_CHECK((Jdiff_num_diff_secondrand - Jdiff_num_diff_both_secondrand).isZero(1e-9));
}

void test_Jint_num_diff_firstsecond(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x = stateLPFAll->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(stateLPFAll->get_ndy());
  // Get the num diff stateLPFAll
  crocoddyl::StateNumDiff state_num_diff(stateLPFAll);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jint_num_diff_tmp(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_num_diff_first(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_num_diff_second(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  state_num_diff.Jintegrate(x, dx, Jint_num_diff_first, Jint_num_diff_tmp, crocoddyl::first);
  state_num_diff.Jintegrate(x, dx, Jint_num_diff_tmp, Jint_num_diff_second, crocoddyl::second);
  // Computing the partial derivatives of the given function separately
  Eigen::MatrixXd Jint_num_diff_both_first(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_num_diff_both_second(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  state_num_diff.Jintegrate(x, dx, Jint_num_diff_both_first, Jint_num_diff_both_second);
  BOOST_CHECK((Jint_num_diff_first - Jint_num_diff_both_first).isZero(1e-9));
  BOOST_CHECK((Jint_num_diff_second - Jint_num_diff_both_second).isZero(1e-9));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& xrand = stateLPFRand->rand();
  const Eigen::VectorXd& dxrand = Eigen::VectorXd::Random(stateLPFRand->get_ndy());
  // Get the num diff stateLPFRand
  crocoddyl::StateNumDiff state_num_diffrand(stateLPFRand);
  // Computing the partial derivatives of the difference function separately
  Eigen::MatrixXd Jint_num_diff_tmprand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jint_num_diff_firstrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jint_num_diff_secondrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  state_num_diffrand.Jintegrate(xrand, dxrand, Jint_num_diff_firstrand, Jint_num_diff_tmprand, crocoddyl::first);
  state_num_diffrand.Jintegrate(xrand, dxrand, Jint_num_diff_tmprand, Jint_num_diff_secondrand, crocoddyl::second);
  // Computing the partial derivatives of the given function separately
  Eigen::MatrixXd Jint_num_diff_both_firstrand(
      Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jint_num_diff_both_secondrand(
      Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  state_num_diffrand.Jintegrate(xrand, dxrand, Jint_num_diff_both_firstrand, Jint_num_diff_both_secondrand);
  BOOST_CHECK((Jint_num_diff_firstrand - Jint_num_diff_both_firstrand).isZero(1e-9));
  BOOST_CHECK((Jint_num_diff_secondrand - Jint_num_diff_both_secondrand).isZero(1e-9));
}

void test_Jdiff_against_numdiff(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x1 = stateLPFAll->rand();
  const Eigen::VectorXd& x2 = stateLPFAll->rand();
  // Computing the partial derivatives of the difference function analytically
  Eigen::MatrixXd Jdiff_1(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdiff_2(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jdiff(x1, x2, Jdiff_1, Jdiff_2, crocoddyl::first);
  stateLPFAll->Jdiff(x1, x2, Jdiff_1, Jdiff_2, crocoddyl::second);
  // Computing the partial derivatives of the difference function numerically
  crocoddyl::StateNumDiff state_num_diff(stateLPFAll);
  Eigen::MatrixXd Jdiff_num_1(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdiff_num_2(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  state_num_diff.Jdiff(x1, x2, Jdiff_num_1, Jdiff_num_2);
  // Checking the partial derivatives against NumDiff
  // The previous tolerance was 10*disturbance
  double tol = NUMDIFF_MODIFIER * sqrt(state_num_diff.get_disturbance());
  BOOST_CHECK((Jdiff_1 - Jdiff_num_1).isZero(tol));
  BOOST_CHECK((Jdiff_2 - Jdiff_num_2).isZero(tol));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random values for the initial and terminal states
  const Eigen::VectorXd& x1rand = stateLPFRand->rand();
  const Eigen::VectorXd& x2rand = stateLPFRand->rand();
  // Computing the partial derivatives of the difference function analytically
  Eigen::MatrixXd Jdiff_1rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdiff_2rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  stateLPFRand->Jdiff(x1rand, x2rand, Jdiff_1rand, Jdiff_2rand, crocoddyl::first);
  stateLPFRand->Jdiff(x1rand, x2rand, Jdiff_1rand, Jdiff_2rand, crocoddyl::second);
  // Computing the partial derivatives of the difference function numerically
  crocoddyl::StateNumDiff state_num_diffrand(stateLPFRand);
  Eigen::MatrixXd Jdiff_num_1rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdiff_num_2rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  state_num_diffrand.Jdiff(x1rand, x2rand, Jdiff_num_1rand, Jdiff_num_2rand);
  // Checking the partial derivatives against NumDiff
  // The previous tolerance was 10*disturbance
  double tolrand = NUMDIFF_MODIFIER * sqrt(state_num_diffrand.get_disturbance());
  BOOST_CHECK((Jdiff_1rand - Jdiff_num_1rand).isZero(tolrand));
  BOOST_CHECK((Jdiff_2rand - Jdiff_num_2rand).isZero(tolrand));
}

void test_Jintegrate_against_numdiff(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random values for the initial stateLPFAll and its rate of change
  const Eigen::VectorXd& x = stateLPFAll->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(stateLPFAll->get_ndy());
  // Computing the partial derivatives of the difference function analytically
  Eigen::MatrixXd Jint_1(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_2(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jintegrate(x, dx, Jint_1, Jint_2);
  // Computing the partial derivatives of the difference function numerically
  crocoddyl::StateNumDiff state_num_diff(stateLPFAll);
  Eigen::MatrixXd Jint_num_1(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_num_2(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  state_num_diff.Jintegrate(x, dx, Jint_num_1, Jint_num_2);
  // Checking the partial derivatives against NumDiff
  // The previous tolerance was 10*disturbance
  double tol = sqrt(state_num_diff.get_disturbance());
  BOOST_CHECK((Jint_1 - Jint_num_1).isZero(tol));
  BOOST_CHECK((Jint_2 - Jint_num_2).isZero(tol));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random values for the initial stateLPFAll and its rate of change
  const Eigen::VectorXd& xrand = stateLPFRand->rand();
  const Eigen::VectorXd& dxrand = Eigen::VectorXd::Random(stateLPFRand->get_ndy());
  // Computing the partial derivatives of the difference function analytically
  Eigen::MatrixXd Jint_1rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jint_2rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  stateLPFRand->Jintegrate(xrand, dxrand, Jint_1rand, Jint_2rand);
  // Computing the partial derivatives of the difference function numerically
  crocoddyl::StateNumDiff state_num_diffrand(stateLPFRand);
  Eigen::MatrixXd Jint_num_1rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jint_num_2rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  state_num_diffrand.Jintegrate(xrand, dxrand, Jint_num_1rand, Jint_num_2rand);
  // Checking the partial derivatives against NumDiff
  // The previous tolerance was 10*disturbance
  double tolrand = sqrt(state_num_diffrand.get_disturbance());
  BOOST_CHECK((Jint_1rand - Jint_num_1rand).isZero(tolrand));
  BOOST_CHECK((Jint_2rand - Jint_num_2rand).isZero(tolrand));

  // Check Jdiff against state multibody when nu=0
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFNone = factory.create(state_type, LPFJointMaskType::NONE);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateLPFToStateMultibody.at(state_type));
  Eigen::MatrixXd Jint_1LPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  Eigen::MatrixXd Jint_2LPF(Eigen::MatrixXd::Zero(stateLPFNone->get_ndy(), stateLPFNone->get_ndy()));
  stateLPFNone->Jintegrate(x, dx, Jint_1LPF, Jint_2LPF);
  Eigen::MatrixXd Jint_1Multibody(Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  Eigen::MatrixXd Jint_2Multibody(Eigen::MatrixXd::Zero(stateMultibody->get_ndx(), stateMultibody->get_ndx()));
  stateMultibody->Jintegrate(x, dx, Jint_1Multibody, Jint_2Multibody);
  BOOST_CHECK((Jint_1LPF - Jint_1Multibody).isZero(tol));
  BOOST_CHECK((Jint_2LPF - Jint_2Multibody).isZero(tol));
}

void test_JintegrateTransport(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random values for the initial stateLPFAll and its rate of change
  const Eigen::VectorXd& x = stateLPFAll->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(stateLPFAll->get_ndy());
  // Computing the partial derivatives of the difference function analytically
  Eigen::MatrixXd Jint_1(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jint_2(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jintegrate(x, dx, Jint_1, Jint_2);
  Eigen::MatrixXd Jref(Eigen::MatrixXd::Random(stateLPFAll->get_ndy(), 2 * stateLPFAll->get_ndy()));
  const Eigen::MatrixXd Jtest(Jref);
  stateLPFAll->JintegrateTransport(x, dx, Jref, crocoddyl::first);
  BOOST_CHECK((Jref - Jint_1 * Jtest).isZero(1e-10));
  Jref = Jtest;
  stateLPFAll->JintegrateTransport(x, dx, Jref, crocoddyl::second);
  BOOST_CHECK((Jref - Jint_2 * Jtest).isZero(1e-10));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random values for the initial stateLPFAll and its rate of change
  const Eigen::VectorXd& xrand = stateLPFRand->rand();
  const Eigen::VectorXd& dxrand = Eigen::VectorXd::Random(stateLPFRand->get_ndy());
  // Computing the partial derivatives of the difference function analytically
  Eigen::MatrixXd Jint_1rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jint_2rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  stateLPFRand->Jintegrate(xrand, dxrand, Jint_1rand, Jint_2rand);
  Eigen::MatrixXd Jrefrand(Eigen::MatrixXd::Random(stateLPFRand->get_ndy(), 2 * stateLPFRand->get_ndy()));
  const Eigen::MatrixXd Jtestrand(Jrefrand);
  stateLPFRand->JintegrateTransport(xrand, dxrand, Jrefrand, crocoddyl::first);
  BOOST_CHECK((Jrefrand - Jint_1rand * Jtestrand).isZero(1e-10));
  Jrefrand = Jtestrand;
  stateLPFRand->JintegrateTransport(xrand, dxrand, Jrefrand, crocoddyl::second);
  BOOST_CHECK((Jrefrand - Jint_2rand * Jtestrand).isZero(1e-10));

  // Check JintegrateTransport against state multibody JintegrateTransport when
  // nu=0
  StateModelFactory factoryMultibody;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFNone = factory.create(state_type, LPFJointMaskType::NONE);
  const boost::shared_ptr<crocoddyl::StateAbstract>& stateMultibody =
      factoryMultibody.create(mapStateLPFToStateMultibody.at(state_type));
  const Eigen::VectorXd& x1 = stateLPFNone->rand();
  const Eigen::VectorXd& dx1 = Eigen::VectorXd::Random(stateLPFNone->get_ndy());
  Eigen::MatrixXd JrefLPF(Eigen::MatrixXd::Random(stateLPFNone->get_ndy(), 2 * stateLPFNone->get_ndy()));
  Eigen::MatrixXd JrefMultibody(JrefLPF);
  const Eigen::MatrixXd Jtest1(JrefLPF);
  // test first
  stateLPFNone->JintegrateTransport(x1, dx1, JrefLPF, crocoddyl::first);
  stateMultibody->JintegrateTransport(x1, dx1, JrefMultibody, crocoddyl::first);
  BOOST_CHECK((JrefLPF - JrefMultibody).isZero(1e-10));
  // reset
  JrefLPF = Jtest1;
  JrefMultibody = Jtest1;
  // test second
  stateLPFNone->JintegrateTransport(x1, dx1, JrefLPF, crocoddyl::second);
  stateMultibody->JintegrateTransport(x1, dx1, JrefMultibody, crocoddyl::second);
  BOOST_CHECK((JrefLPF - JrefMultibody).isZero(1e-10));
}

void test_Jdiff_and_Jintegrate_are_inverses(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random states
  const Eigen::VectorXd& x1 = stateLPFAll->rand();
  const Eigen::VectorXd& dx = Eigen::VectorXd::Random(stateLPFAll->get_ndy());
  Eigen::VectorXd x2(stateLPFAll->get_ny());
  stateLPFAll->integrate(x1, dx, x2);
  // Computing the partial derivatives of the integrate and difference function
  Eigen::MatrixXd Jx(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdx(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jintegrate(x1, dx, Jx, Jdx);
  Eigen::MatrixXd J1(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd J2(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jdiff(x1, x2, J1, J2);
  // Checking that Jdiff and Jintegrate are inverses
  Eigen::MatrixXd dX_dDX = Jdx;
  Eigen::MatrixXd dDX_dX = J2;
  BOOST_CHECK((dX_dDX - dDX_dX.inverse()).isZero(1e-9));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random states
  const Eigen::VectorXd& x1rand = stateLPFRand->rand();
  const Eigen::VectorXd& dxrand = Eigen::VectorXd::Random(stateLPFRand->get_ndy());
  Eigen::VectorXd x2rand(stateLPFRand->get_ny());
  stateLPFRand->integrate(x1rand, dxrand, x2rand);
  // Computing the partial derivatives of the integrate and difference function
  Eigen::MatrixXd Jxrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdxrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  stateLPFRand->Jintegrate(x1rand, dxrand, Jxrand, Jdxrand);
  Eigen::MatrixXd J1rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd J2rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  stateLPFRand->Jdiff(x1rand, x2rand, J1rand, J2rand);
  // Checking that Jdiff and Jintegrate are inverses
  Eigen::MatrixXd dX_dDXrand = Jdxrand;
  Eigen::MatrixXd dDX_dXrand = J2rand;
  BOOST_CHECK((dX_dDXrand - dDX_dXrand.inverse()).isZero(1e-9));
}

void test_velocity_from_Jintegrate_Jdiff(StateLPFModelTypes::Type state_type) {
  StateLPFModelFactory factory;
  const boost::shared_ptr<sobec::StateLPF>& stateLPFAll = factory.create(state_type, LPFJointMaskType::ALL);
  // Generating random states
  const Eigen::VectorXd& x1 = stateLPFAll->rand();
  Eigen::VectorXd dx = Eigen::VectorXd::Random(stateLPFAll->get_ndy());
  Eigen::VectorXd x2(stateLPFAll->get_ny());
  stateLPFAll->integrate(x1, dx, x2);
  Eigen::VectorXd eps = Eigen::VectorXd::Random(stateLPFAll->get_ndy());
  double h = 1e-8;
  // Computing the partial derivatives of the integrate and difference function
  Eigen::MatrixXd Jx(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd Jdx(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jintegrate(x1, dx, Jx, Jdx);
  Eigen::MatrixXd J1(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  Eigen::MatrixXd J2(Eigen::MatrixXd::Zero(stateLPFAll->get_ndy(), stateLPFAll->get_ndy()));
  stateLPFAll->Jdiff(x1, x2, J1, J2);
  // Checking that computed velocity from Jintegrate
  const Eigen::MatrixXd& dX_dDX = Jdx;
  Eigen::VectorXd x2eps(stateLPFAll->get_ny());
  stateLPFAll->integrate(x1, dx + eps * h, x2eps);
  Eigen::VectorXd x2_eps(stateLPFAll->get_ndy());
  stateLPFAll->diff(x2, x2eps, x2_eps);
  BOOST_CHECK((dX_dDX * eps - x2_eps / h).isZero(1e-3));
  // Checking the velocity computed from Jdiff
  const Eigen::VectorXd& x = stateLPFAll->rand();
  dx.setZero();
  stateLPFAll->diff(x1, x, dx);
  Eigen::VectorXd x2i(stateLPFAll->get_ny());
  stateLPFAll->integrate(x, eps * h, x2i);
  Eigen::VectorXd dxi(stateLPFAll->get_ndy());
  stateLPFAll->diff(x1, x2i, dxi);
  J1.setZero();
  J2.setZero();
  stateLPFAll->Jdiff(x1, x, J1, J2);
  BOOST_CHECK((J2 * eps - (-dx + dxi) / h).isZero(1e-3));

  // Check for random LPF joint in stateLPF
  const boost::shared_ptr<sobec::StateLPF>& stateLPFRand = factory.create(state_type, LPFJointMaskType::RAND);
  // Generating random states
  const Eigen::VectorXd& x1rand = stateLPFRand->rand();
  Eigen::VectorXd dxrand = Eigen::VectorXd::Random(stateLPFRand->get_ndy());
  Eigen::VectorXd x2rand(stateLPFRand->get_ny());
  stateLPFRand->integrate(x1rand, dxrand, x2rand);
  Eigen::VectorXd epsrand = Eigen::VectorXd::Random(stateLPFRand->get_ndy());
  double hrand = 1e-8;
  // Computing the partial derivatives of the integrate and difference function
  Eigen::MatrixXd Jxrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd Jdxrand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  stateLPFRand->Jintegrate(x1rand, dxrand, Jxrand, Jdxrand);
  Eigen::MatrixXd J1rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  Eigen::MatrixXd J2rand(Eigen::MatrixXd::Zero(stateLPFRand->get_ndy(), stateLPFRand->get_ndy()));
  stateLPFRand->Jdiff(x1rand, x2rand, J1rand, J2rand);
  // Checking that computed velocity from Jintegrate
  const Eigen::MatrixXd& dX_dDXrand = Jdxrand;
  Eigen::VectorXd x2epsrand(stateLPFRand->get_ny());
  stateLPFRand->integrate(x1rand, dxrand + epsrand * hrand, x2epsrand);
  Eigen::VectorXd x2_epsrand(stateLPFRand->get_ndy());
  stateLPFRand->diff(x2rand, x2epsrand, x2_epsrand);
  BOOST_CHECK((dX_dDXrand * epsrand - x2_epsrand / hrand).isZero(1e-3));
  // Checking the velocity computed from Jdiff
  const Eigen::VectorXd& xrand = stateLPFRand->rand();
  dxrand.setZero();
  stateLPFRand->diff(x1rand, xrand, dxrand);
  Eigen::VectorXd x2irand(stateLPFRand->get_ny());
  stateLPFRand->integrate(xrand, epsrand * hrand, x2irand);
  Eigen::VectorXd dxirand(stateLPFRand->get_ndy());
  stateLPFRand->diff(x1rand, x2irand, dxirand);
  J1rand.setZero();
  J2rand.setZero();
  stateLPFRand->Jdiff(x1rand, xrand, J1rand, J2rand);
  BOOST_CHECK((J2rand * epsrand - (-dxrand + dxirand) / hrand).isZero(1e-3));
}

//----------------------------------------------------------------------------//

void register_state_unit_tests(StateLPFModelTypes::Type state_type) {
  boost::test_tools::output_test_stream test_name;
  test_name << "test_" << state_type;
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  ts->add(BOOST_TEST_CASE(boost::bind(&test_state_dimension, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_integrate_against_difference, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_difference_against_integrate, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jdiff_firstsecond, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jint_firstsecond, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jdiff_num_diff_firstsecond, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jint_num_diff_firstsecond, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jdiff_against_numdiff, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jintegrate_against_numdiff, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_JintegrateTransport, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_Jdiff_and_Jintegrate_are_inverses, state_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_velocity_from_Jintegrate_Jdiff, state_type)));
  framework::master_test_suite().add(ts);
}

bool init_function() {
  // test state LPF functions
  for (size_t i = 0; i < StateLPFModelTypes::all.size(); ++i) {
    register_state_unit_tests(StateLPFModelTypes::all[i]);
  }

  return true;
}

int main(int argc, char** argv) { return ::boost::unit_test::unit_test_main(&init_function, argc, argv); }

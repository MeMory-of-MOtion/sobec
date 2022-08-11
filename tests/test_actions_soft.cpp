///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, New York University, Max Planck
// Gesellschaft
//                          University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include <crocoddyl/core/integrator/euler.hpp>

#include "common.hpp"
#include "factory/action-soft.hpp"
#include "factory/diff-action-soft.hpp"
// #include "factory/contact1d.hpp"

using namespace boost::unit_test;
using namespace sobec::unittest;

//----------------------------------------------------------------------------//

void test_check_data(
    IAMSoftContactTypes::Type iam_type,
    DAMSoftContactTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  // create the model
  IAMSoftContactFactory factory_iam;
  const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& model =
      factory_iam.create(iam_type, dam_type, ref_type);
  // Run the print function
  std::ostringstream tmp;
  tmp << *model;
  // create the corresponding data object
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
      model->createData();
  BOOST_CHECK(model->checkData(data));
}



void test_calc_returns_state(
    IAMSoftContactTypes::Type iam_type,
    DAMSoftContactTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  // create the model
  IAMSoftContactFactory factory_iam;
  const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& model =
      factory_iam.create(iam_type, dam_type, ref_type);
  // create the corresponding data object
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
      model->createData();
  // Generating random state and control vectors
  const Eigen::VectorXd& x = model->get_state()->rand();
  const Eigen::VectorXd& u = Eigen::VectorXd::Random(model->get_nu());
  // Getting the state dimension from calc() call
  model->calc(data, x, u);
  BOOST_CHECK(static_cast<std::size_t>(data->xnext.size()) ==
              model->get_state()->get_nx());
}

void test_calc_returns_a_cost(
    IAMSoftContactTypes::Type iam_type,
    DAMSoftContactTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  // create the model
  IAMSoftContactFactory factory_iam;
  const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& model =
      factory_iam.create(iam_type, dam_type, ref_type);
  // create the corresponding data object and set the cost to nan
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
      model->createData();
  data->cost = nan("");
  // Getting the cost value computed by calc()
  const Eigen::VectorXd& x = model->get_state()->rand();
  const Eigen::VectorXd& u = Eigen::VectorXd::Random(model->get_nu());
  model->calc(data, x, u);
  // Checking that calc returns a cost value
  BOOST_CHECK(!std::isnan(data->cost));
}




void test_partial_derivatives_against_numdiff(
    const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& model) {
  // create the corresponding data object and set the cost to nan
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
      model->createData();

  crocoddyl::ActionModelNumDiff model_num_diff(model);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data_num_diff =
      model_num_diff.createData();

  // Generating random values for the state and control
  Eigen::VectorXd x = model->get_state()->rand();
  const Eigen::VectorXd& u = Eigen::VectorXd::Random(model->get_nu());

  // Computing the action derivatives
  model->calc(data, x, u);
  model->calcDiff(data, x, u);

  model_num_diff.calc(data_num_diff, x, u);
  model_num_diff.calcDiff(data_num_diff, x, u);

  // Checking the partial derivatives against NumDiff
  double tol = sqrt(model_num_diff.get_disturbance());
  BOOST_CHECK((data->Fx - data_num_diff->Fx).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((data->Fu - data_num_diff->Fu).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((data->Lx - data_num_diff->Lx).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((data->Lu - data_num_diff->Lu).isZero(NUMDIFF_MODIFIER * tol));
  if (model_num_diff.get_with_gauss_approx()) {
    BOOST_CHECK((data->Lxx - data_num_diff->Lxx).isZero(NUMDIFF_MODIFIER * tol));
    BOOST_CHECK((data->Lxu - data_num_diff->Lxu).isZero(NUMDIFF_MODIFIER * tol));
    BOOST_CHECK((data->Luu - data_num_diff->Luu).isZero(NUMDIFF_MODIFIER * tol));
  } else {
    BOOST_CHECK((data_num_diff->Lxx).isZero(tol));
    BOOST_CHECK((data_num_diff->Lxu).isZero(tol));
    BOOST_CHECK((data_num_diff->Luu).isZero(tol));
  }
}

void test_partial_derivatives_action_model(
    IAMSoftContactTypes::Type iam_type,
    DAMSoftContactTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  // create the model
  IAMSoftContactFactory factory;
  const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& model =
      factory.create(iam_type, dam_type, ref_type);
  test_partial_derivatives_against_numdiff(model);
}




void test_partial_derivatives_against_numdiff_terminal(
    const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& model) {
  // create the corresponding data object and set the cost to nan
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
      model->createData();

  crocoddyl::ActionModelNumDiff model_num_diff(model);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data_num_diff =
      model_num_diff.createData();

  // Generating random values for the state and control
  Eigen::VectorXd x = model->get_state()->rand();

  // Computing the action derivatives
  model->calc(data, x);
  model->calcDiff(data, x);
  model_num_diff.calc(data_num_diff, x);
  model_num_diff.calcDiff(data_num_diff, x);

  // Checking the partial derivatives against NumDiff
  double tol = sqrt(model_num_diff.get_disturbance());
  // Checking the partial derivatives against NumDiff
  BOOST_CHECK((data->Fx - data_num_diff->Fx).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((data->Fu - data_num_diff->Fu).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((data->Lx - data_num_diff->Lx).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((data->Lu - data_num_diff->Lu).isZero(NUMDIFF_MODIFIER * tol));
  if (model_num_diff.get_with_gauss_approx()) {
    BOOST_CHECK((data->Lxx - data_num_diff->Lxx).isZero(NUMDIFF_MODIFIER * tol));
    BOOST_CHECK((data->Lxu - data_num_diff->Lxu).isZero(NUMDIFF_MODIFIER * tol));
    BOOST_CHECK((data->Luu - data_num_diff->Luu).isZero(NUMDIFF_MODIFIER * tol));
  } else {
    BOOST_CHECK((data_num_diff->Lxx).isZero(tol));
    BOOST_CHECK((data_num_diff->Lxu).isZero(tol));
    BOOST_CHECK((data_num_diff->Luu).isZero(tol));
  }
}

void test_partial_derivatives_action_model_terminal(
    IAMSoftContactTypes::Type iam_type,
    DAMSoftContactTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  // create the model
  IAMSoftContactFactory factory;
  const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& model =
      factory.create(iam_type, dam_type, ref_type);
  model->set_dt(0.);
  test_partial_derivatives_against_numdiff_terminal(model);
}




void test_calc_equivalent_euler(
    IAMSoftContactTypes::Type iam_type,
    DAMSoftContactTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  // Create IAM soft from DAMSoft
  IAMSoftContactFactory factory_iam;
  const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& modelSoft = factory_iam.create(iam_type, dam_type, ref_type);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataSoft = modelSoft->createData();
  // Set gains to 0
  modelSoft->get_differential()->set_Kp(0.);
  modelSoft->get_differential()->set_Kv(0.);

  // Create IAM Euler from DAMfree (incompatible with DAMSoft)
  boost::shared_ptr<crocoddyl::StateMultibody> statemb = boost::static_pointer_cast<crocoddyl::StateMultibody>(modelSoft->get_differential()->get_state()); 
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> modelfree = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
          statemb, modelSoft->get_differential()->get_actuation(), modelSoft->get_differential()->get_costs());
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> modelEuler = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(modelfree, modelSoft->get_dt(), true);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEuler = modelEuler->createData();

  // Generating random state and control vectors
  std::size_t nx = statemb->get_nx();
  std::size_t nc = modelSoft->get_nc();
  Eigen::VectorXd y = modelSoft->get_state()->rand();
  y.tail(nc) = Eigen::VectorXd::Zero(nc); // set 0 initial force
  Eigen::VectorXd x = y.head(nx);
  Eigen::VectorXd u = Eigen::VectorXd::Random(modelSoft->get_nu());
  // Getting the state dimension from calc() call
  modelSoft->calc(dataSoft, y, u);
  modelEuler->calc(dataEuler, x, u);
  BOOST_CHECK((dataSoft->xnext.head(nx) - dataEuler->xnext).norm() <= 1e-8);
  BOOST_CHECK((dataSoft->xnext.head(nx) - dataEuler->xnext).isZero(1e-6));
}

void test_calcDiff_equivalent_euler(
    IAMSoftContactTypes::Type iam_type,
    DAMSoftContactTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  // Create IAM soft from DAMSoft
  IAMSoftContactFactory factory_iam;
  const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& modelSoft = factory_iam.create(iam_type, dam_type, ref_type);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataSoft = modelSoft->createData();
  // Set gains to 0
  modelSoft->get_differential()->set_Kp(0.);
  modelSoft->get_differential()->set_Kv(0.);

  // Create IAM Euler from DAMfree (incompatible with DAMSoft)
  boost::shared_ptr<crocoddyl::StateMultibody> statemb = boost::static_pointer_cast<crocoddyl::StateMultibody>(modelSoft->get_differential()->get_state()); 
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> modelfree = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
          statemb, modelSoft->get_differential()->get_actuation(), modelSoft->get_differential()->get_costs());
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> modelEuler = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(modelfree, modelSoft->get_dt(), true);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEuler = modelEuler->createData();

  // Generating random state and control vectors
  std::size_t nx = statemb->get_nx();
  std::size_t ndx = statemb->get_ndx();
  std::size_t nc = modelSoft->get_nc();
  std::size_t nu = modelSoft->get_nu();
  Eigen::VectorXd y = modelSoft->get_state()->rand();
  y.tail(nc) = Eigen::VectorXd::Zero(nc); // set 0 initial force
  Eigen::VectorXd x = y.head(nx);
  Eigen::VectorXd u = Eigen::VectorXd::Random(nu);
  modelSoft->calc(dataSoft, y, u);
  modelSoft->calcDiff(dataSoft, y, u);
  modelEuler->calc(dataEuler, x, u);
  modelEuler->calcDiff(dataEuler, x, u);

  double tol = sqrt(1e-8);

  const Eigen::MatrixXd& Fx = dataSoft->Fx.topLeftCorner(ndx, ndx);
  const Eigen::MatrixXd& Lx = dataSoft->Lx.head(ndx);
  const Eigen::MatrixXd& Lu = dataSoft->Lu;
  const Eigen::MatrixXd& Lxx = dataSoft->Lxx.topLeftCorner(ndx, ndx);
  const Eigen::MatrixXd& Lxu = dataSoft->Lxu.topLeftCorner(ndx, nu);
  const Eigen::MatrixXd& Luu = dataSoft->Luu;

  BOOST_CHECK((Fx - dataEuler->Fx).isZero(tol));
  BOOST_CHECK((Lx - dataEuler->Lx).isZero(tol));
  BOOST_CHECK((Lu - dataEuler->Lu).isZero(tol));
  BOOST_CHECK((Lxx - dataEuler->Lxx).isZero(tol));
  BOOST_CHECK((Lxu - dataEuler->Lxu).isZero(tol));
  BOOST_CHECK((Luu - dataEuler->Luu).isZero(tol));

}



//----------------------------------------------------------------------------//

void register_action_model_unit_tests(
    IAMSoftContactTypes::Type iam_type,
    DAMSoftContactTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  boost::test_tools::output_test_stream test_name;
  test_name << "test_" << iam_type << "_" << dam_type << "_" << ref_type;
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
//   ts->add(BOOST_TEST_CASE(boost::bind(&test_check_data, iam_type, dam_type, ref_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_state, iam_type, dam_type, ref_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_a_cost, iam_type, dam_type, ref_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_partial_derivatives_action_model, iam_type, dam_type, ref_type)));
  // Need to test terminal model as well ? Seems to be incompatible with Euler test 
  // ts->add(BOOST_TEST_CASE(boost::bind(&test_partial_derivatives_action_model_terminal, iam_type, dam_type, ref_type)));
  // Equivalence with Euler when Kp, Kv=0
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_equivalent_euler, iam_type, dam_type, ref_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calcDiff_equivalent_euler, iam_type, dam_type, ref_type)));
  framework::master_test_suite().add(ts);
}

bool init_function() {

  // 3D contact
  for (size_t i = 0; i < IAMSoftContactTypes::all.size(); ++i) {
    for (size_t j = 0; j < DAMSoftContactTypes::all.size(); ++j) {
      for (size_t k = 0; k < PinocchioReferenceTypes::all.size(); ++k) {
        register_action_model_unit_tests(IAMSoftContactTypes::all[i],
                                         DAMSoftContactTypes::all[j],
                                         PinocchioReferenceTypes::all[k]);
      }
    }
  }


  return true;
}

int main(int argc, char** argv) {
  return ::boost::unit_test::unit_test_main(&init_function, argc, argv);
}

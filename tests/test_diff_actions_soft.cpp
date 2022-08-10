///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, New York University, Max Planck
// Gesellschaft, INRIA, University of Oxford Copyright note valid unless
// otherwise stated in individual files. All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include "common.hpp"
#include "factory/diff-action-soft.hpp"

using namespace boost::unit_test;
using namespace sobec::unittest;

//----------------------------------------------------------------------------//

void test_check_data(DAMSoftContactTypes::Type action_type,
                     PinocchioReferenceTypes::Type ref_type) {
  // create the model
  DAMSoftContactFactory factory;
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> model =
      factory.create(action_type, ref_type);
  // Run the print function
  std::ostringstream tmp;
  tmp << *model;
  // create the corresponding data object
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data =
      model->createData();
  BOOST_CHECK(model->checkData(data));
}


void test_calc_returns_state(DAMSoftContactTypes::Type action_type,
                             PinocchioReferenceTypes::Type ref_type) {
  // create the model
  DAMSoftContactFactory factory;
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> model = factory.create(action_type, ref_type);
//   boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> modelcast= boost::static_pointer_cast<sobec::DAMSoftContact3DAugmentedFwdDynamics>(model);
  // create the corresponding data object
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = model->createData();
  // Generating random state and control vectors
  const Eigen::VectorXd x = model->get_state()->rand();
  const Eigen::VectorXd f = Eigen::VectorXd::Random(model->get_nc());
  const Eigen::VectorXd u = Eigen::VectorXd::Random(model->get_nu());
  // Getting the state dimension from calc() call
  model->calc(data, x, f, u);
  BOOST_CHECK(static_cast<std::size_t>(data->xout.size()) == model->get_state()->get_nv());
}


// void test_calc_returns_a_cost(DAMSoftContactTypes::Type action_type,
//                               PinocchioReferenceTypes::Type ref_type) {
//   // create the model
//   DAMSoftContactFactory factory;
//   boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> model =
//       factory.create(action_type, ref_type);

//   // create the corresponding data object and set the cost to nan
//   boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data =
//       model->createData();
//   data->cost = nan("");

//   // Getting the cost value computed by calc()
//   const Eigen::VectorXd x = model->get_state()->rand();
//   const Eigen::VectorXd u = Eigen::VectorXd::Random(model->get_nu());
//   model->calc(data, x, u);

//   // Checking that calc returns a cost value
//   BOOST_CHECK(!std::isnan(data->cost));
// }

// void test_quasi_static(DAMSoftContactTypes::Type action_type,
//                        PinocchioReferenceTypes::Type ref_type) {
//   if (action_type ==
//       DAMSoftContactTypes::
//           DifferentialActionModelFreeFwdDynamics_TalosArm_Squashed)
//     return;
//   // create the model
//   DAMSoftContactFactory factory;
//   boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> model =
//       factory.create(action_type, ref_type);

//   // create the corresponding data object and set the cost to nan
//   boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data =
//       model->createData();

//   // Getting the cost value computed by calc()
//   Eigen::VectorXd x = model->get_state()->rand();
//   x.tail(model->get_state()->get_nv()).setZero();
//   Eigen::VectorXd u = Eigen::VectorXd::Zero(model->get_nu());
//   model->quasiStatic(data, u, x);
//   model->calc(data, x, u);

//   BOOST_CHECK(data->xout.norm() <= 1e-8);
// }

// void test_partial_derivatives_against_numdiff(
//     DAMSoftContactTypes::Type action_type,
//     PinocchioReferenceTypes::Type ref_type) {
//   // create the model
//   DAMSoftContactFactory factory;
//   boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> model =
//       factory.create(action_type, ref_type);

//   // create the corresponding data object and set the cost to nan
//   boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data =
//       model->createData();

//   crocoddyl::DifferentialActionModelNumDiff model_num_diff(model);
//   boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data_num_diff =
//       model_num_diff.createData();

//   // Generating random values for the state and control
//   Eigen::VectorXd x = model->get_state()->rand();
//   Eigen::VectorXd u = Eigen::VectorXd::Random(model->get_nu());

//   // Computing the action derivatives
//   model->calc(data, x, u);
//   model->calcDiff(data, x, u);

//   model_num_diff.calc(data_num_diff, x, u);
//   model_num_diff.calcDiff(data_num_diff, x, u);


//   // Checking the partial derivatives against NumDiff
//   double tol = sqrt(model_num_diff.get_disturbance());
//   // if((data->Fx - data_num_diff->Fx).isZero(NUMDIFF_MODIFIER * tol) == false) {
//   //   boost::shared_ptr<sobec::DifferentialActionDataSoftContact3DFwdDynamics> data_cast = boost::static_pointer_cast<sobec::DifferentialActionDataSoftContact3DFwdDynamics>(data); 
//   //   // std::cout << " dv_dq = " << std::endl;
//   //   // std::cout << data_cast->lv_partial_dq << std::endl;
//   //   // std::cout << " dv_dv = " << std::endl;
//   //   // std::cout << data_cast->lv_partial_dv << std::endl;
//   //   std::cout << " df_dx = " << std::endl;
//   //   std::cout << data_cast->df_dx << std::endl;
//   //   std::cout << " aba_dq = " << std::endl;
//   //   std::cout << data_cast->aba_dq << std::endl;
//   //   std::cout << " aba_dv = " << std::endl;
//   //   std::cout << data_cast->aba_dv << std::endl;
//   //   std::cout << " aba_dtau = " << std::endl;
//   //   std::cout << data_cast->aba_dtau << std::endl;
//   //   std::cout << " Minv = " << std::endl;
//   //   std::cout << data_cast->pinocchio.Minv << std::endl;
//   //   std::cout << " lJ = " << std::endl;
//   //   std::cout << data_cast->lJ << std::endl;
//   //   std::cout << " Fx - Fx_ND = " << std::endl;
//   //   std::cout << data->Fx - data_num_diff->Fx << std::endl;
//   // }
//   // if((data->Lx - data_num_diff->Lx).isZero(NUMDIFF_MODIFIER * tol) == false) {
//   //   std::cout << " Lx - Lx_ND = " << std::endl;
//   //   std::cout << data->Lx - data_num_diff->Lx << std::endl;
//   // }
//   BOOST_CHECK((data->Fx - data_num_diff->Fx).isZero(NUMDIFF_MODIFIER * tol));
//   BOOST_CHECK((data->Fu - data_num_diff->Fu).isZero(NUMDIFF_MODIFIER * tol));
//   BOOST_CHECK((data->Lx - data_num_diff->Lx).isZero(NUMDIFF_MODIFIER * tol));
//   BOOST_CHECK((data->Lu - data_num_diff->Lu).isZero(NUMDIFF_MODIFIER * tol));
//   if (model_num_diff.get_with_gauss_approx()) {
//     BOOST_CHECK(
//         (data->Lxx - data_num_diff->Lxx).isZero(NUMDIFF_MODIFIER * tol));
//     BOOST_CHECK(
//         (data->Lxu - data_num_diff->Lxu).isZero(NUMDIFF_MODIFIER * tol));
//     BOOST_CHECK(
//         (data->Luu - data_num_diff->Luu).isZero(NUMDIFF_MODIFIER * tol));
//   } else {
//     BOOST_CHECK((data_num_diff->Lxx).isZero(tol));
//     BOOST_CHECK((data_num_diff->Lxu).isZero(tol));
//     BOOST_CHECK((data_num_diff->Luu).isZero(tol));
//   }

//   // Computing the action derivatives
//   x = model->get_state()->rand();
//   model->calc(data, x);
//   model->calcDiff(data, x);

//   model_num_diff.calc(data_num_diff, x);
//   model_num_diff.calcDiff(data_num_diff, x);

//   // Checking the partial derivatives against NumDiff
//   BOOST_CHECK((data->Lx - data_num_diff->Lx).isZero(NUMDIFF_MODIFIER * tol));
//   if (model_num_diff.get_with_gauss_approx()) {
//     BOOST_CHECK(
//         (data->Lxx - data_num_diff->Lxx).isZero(NUMDIFF_MODIFIER * tol));
//   } else {
//     BOOST_CHECK((data_num_diff->Lxx).isZero(tol));
//   }
// }



// void test_calc_equivalent_free(DAMSoftContactTypes::Type action_type,
//                                PinocchioReferenceTypes::Type ref_type) {
//   // create the model
//   DAMSoftContactFactory factory;
//   boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> model =
//       factory.create(action_type, ref_type);
//   boost::shared_ptr<sobec::DifferentialActionModelSoftContact3DFwdDynamics> modelsoft = boost::static_pointer_cast<sobec::DifferentialActionModelSoftContact3DFwdDynamics>(model); 
//   boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = modelsoft->createData();
  
//   // Create DAM free
//   boost::shared_ptr<crocoddyl::StateMultibody> statemb = boost::static_pointer_cast<crocoddyl::StateMultibody>(modelsoft->get_state()); 
//   boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> modelfree =
//       boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
//           statemb, modelsoft->get_actuation(), modelsoft->get_costs());
//   const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& datafree = modelfree->createData();

//   // Generating random state and control vectors
//   const Eigen::VectorXd x = modelsoft->get_state()->rand();
//   const Eigen::VectorXd u = Eigen::VectorXd::Random(modelsoft->get_nu());
//   // Set 0 stiffness and damping
//   modelsoft->set_Kp(0.);
//   modelsoft->set_Kv(0.);

//   // Getting the state dimension from calc() call
//   modelsoft->calc(data, x, u);
//   modelfree->calc(datafree, x, u);

//   BOOST_CHECK((data->xout - datafree->xout).norm() <= 1e-8);
//   BOOST_CHECK((data->xout - datafree->xout).isZero(1e-6));
// }


// void test_calcDiff_equivalent_free(DAMSoftContactTypes::Type action_type,
//                                PinocchioReferenceTypes::Type ref_type) {
//   // create the model
//   DAMSoftContactFactory factory;
//   boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> model =
//       factory.create(action_type, ref_type);
//   boost::shared_ptr<sobec::DifferentialActionModelSoftContact3DFwdDynamics> modelsoft = boost::static_pointer_cast<sobec::DifferentialActionModelSoftContact3DFwdDynamics>(model); 
//   boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = modelsoft->createData();
  
//   // Create DAM free
//   boost::shared_ptr<crocoddyl::StateMultibody> statemb = boost::static_pointer_cast<crocoddyl::StateMultibody>(modelsoft->get_state()); 
//   boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> modelfree =
//       boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
//           statemb, modelsoft->get_actuation(), modelsoft->get_costs());
//   const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& datafree = modelfree->createData();

//   // Generating random state and control vectors
//   const Eigen::VectorXd x = modelsoft->get_state()->rand();
//   const Eigen::VectorXd u = Eigen::VectorXd::Random(modelsoft->get_nu());
//   // Set 0 stiffness and damping
//   modelsoft->set_Kp(0.);
//   modelsoft->set_Kv(0.);

//   // Getting the state dimension from calc() call
//   modelsoft->calc(data, x, u);
//   modelsoft->calcDiff(data, x, u);
//   modelfree->calc(datafree, x, u);
//   modelfree->calcDiff(datafree, x, u);

//   // Checking the partial derivatives against NumDiff
//   double tol = 1e-6;
//   BOOST_CHECK((data->Fx - datafree->Fx).isZero(tol));
//   BOOST_CHECK((data->Fu - datafree->Fu).isZero(tol));
//   BOOST_CHECK((data->Lx - datafree->Lx).isZero(tol));
//   BOOST_CHECK((data->Lu - datafree->Lu).isZero(tol));
//   BOOST_CHECK((data->Lxx - datafree->Lxx).isZero(tol));
//   BOOST_CHECK((data->Lxu - datafree->Lxu).isZero(tol));
//   BOOST_CHECK((data->Luu - datafree->Luu).isZero(tol));
// }



//----------------------------------------------------------------------------//

void register_action_model_unit_tests(DAMSoftContactTypes::Type action_type,
                                      PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  boost::test_tools::output_test_stream test_name;
  switch (action_type) {
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_TalosArm:
      test_name << "test_" << action_type << "_" << ref_type;
      break;
    case DAMSoftContactTypes::
        DAMSoftContact3DAugmentedFwdDynamics_HyQ:
      test_name << "test_" << action_type << "_" << ref_type;
      break;
    default:
      throw_pretty(__FILE__ ": Wrong DAMSoftContactTypes::Type given");
      break;
  }
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  ts->add(BOOST_TEST_CASE(boost::bind(&test_check_data, action_type, ref_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_state, action_type, ref_type)));
//   ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_a_cost, action_type, ref_type)));
//   ts->add(BOOST_TEST_CASE(boost::bind(&test_partial_derivatives_against_numdiff, action_type, ref_type)));
  // Test equivalence with Euler for soft contact when Kp, Kv = 0
//   ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_equivalent_free, action_type, ref_type)));
//   ts->add(BOOST_TEST_CASE(boost::bind(&test_calcDiff_equivalent_free, action_type, ref_type)));
  framework::master_test_suite().add(ts);
}

bool init_function() {

  for (size_t i = 0; i < DAMSoftContactTypes::all.size(); ++i) {
    for (size_t j = 0; j < PinocchioReferenceTypes::all.size(); ++j) {
    register_action_model_unit_tests(DAMSoftContactTypes::all[i], PinocchioReferenceTypes::all[j]);
    }
  }

  return true;
}

int main(int argc, char** argv) {
  return ::boost::unit_test::unit_test_main(&init_function, argc, argv);
}

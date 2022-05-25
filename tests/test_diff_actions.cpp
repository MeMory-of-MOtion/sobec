///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, New York University, Max Planck Gesellschaft, INRIA, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include "factory/diff-action.hpp"
#include "common.hpp"

using namespace boost::unit_test;
using namespace sobec::unittest;

//----------------------------------------------------------------------------//

void test_check_data(DifferentialActionModelTypes::Type action_type, PinocchioReferenceTypes::Type ref_type, ContactModelMaskTypes::Type mask_type ) {
  // create the model
  DifferentialActionModelFactory factory;
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> model = factory.create(action_type, ref_type, mask_type);

  // Run the print function
  std::ostringstream tmp;
  tmp << *model;

  // create the corresponding data object
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = model->createData();

  BOOST_CHECK(model->checkData(data));
}

void test_calc_returns_state(DifferentialActionModelTypes::Type action_type, PinocchioReferenceTypes::Type ref_type, ContactModelMaskTypes::Type mask_type ) {
  // create the model
  DifferentialActionModelFactory factory;
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> model = factory.create(action_type, ref_type, mask_type);

  // create the corresponding data object
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = model->createData();

  // Generating random state and control vectors
  const Eigen::VectorXd x = model->get_state()->rand();
  const Eigen::VectorXd u = Eigen::VectorXd::Random(model->get_nu());

  // Getting the state dimension from calc() call
  model->calc(data, x, u);

  BOOST_CHECK(static_cast<std::size_t>(data->xout.size()) == model->get_state()->get_nv());
}

void test_calc_returns_a_cost(DifferentialActionModelTypes::Type action_type, PinocchioReferenceTypes::Type ref_type, ContactModelMaskTypes::Type mask_type ) {
  // create the model
  DifferentialActionModelFactory factory;
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> model = factory.create(action_type, ref_type, mask_type);

  // create the corresponding data object and set the cost to nan
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = model->createData();
  data->cost = nan("");

  // Getting the cost value computed by calc()
  const Eigen::VectorXd x = model->get_state()->rand();
  const Eigen::VectorXd u = Eigen::VectorXd::Random(model->get_nu());
  model->calc(data, x, u);

  // Checking that calc returns a cost value
  BOOST_CHECK(!std::isnan(data->cost));
}

void test_quasi_static(DifferentialActionModelTypes::Type action_type, PinocchioReferenceTypes::Type ref_type, ContactModelMaskTypes::Type mask_type ) {
  if (action_type == DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm_Squashed) return;
  // create the model
  DifferentialActionModelFactory factory;
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> model = factory.create(action_type, ref_type, mask_type);

  // create the corresponding data object and set the cost to nan
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = model->createData();

  // Getting the cost value computed by calc()
  Eigen::VectorXd x = model->get_state()->rand();
  x.tail(model->get_state()->get_nv()).setZero();
  Eigen::VectorXd u = Eigen::VectorXd::Zero(model->get_nu());
  model->quasiStatic(data, u, x);
  model->calc(data, x, u);

  // Checking that the acceleration is zero as supposed to be in a quasi static condition
//   if (action_type == DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_HyQ ||
//       action_type == DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_HyQ){
//       std::cout << "xout = " << std::endl;
//       std::cout << data->xout << std::endl;
//   }
  BOOST_CHECK(data->xout.norm() <= 1e-8);

  // Check for inactive contacts
  if (action_type == DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_HyQ ||
      action_type == DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_HyQ){ // ||
    //   action_type == DifferentialActionModelTypes::DifferentialActionModelContactFwdDynamics_Talos) {
    boost::shared_ptr<sobec::DifferentialActionModelContactFwdDynamics> m =
        boost::static_pointer_cast<sobec::DifferentialActionModelContactFwdDynamics>(model);
    m->get_contacts()->changeContactStatus("lf", false);

    model->quasiStatic(data, u, x);
    model->calc(data, x, u);

    // Checking that the acceleration is zero as supposed to be in a quasi static condition
    BOOST_CHECK(data->xout.norm() <= 1e-8);
  }
}

void test_partial_derivatives_against_numdiff(DifferentialActionModelTypes::Type action_type, PinocchioReferenceTypes::Type ref_type, ContactModelMaskTypes::Type mask_type ) {
  // create the model
  DifferentialActionModelFactory factory;
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> model = factory.create(action_type, ref_type, mask_type);

  // create the corresponding data object and set the cost to nan
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = model->createData();

  crocoddyl::DifferentialActionModelNumDiff model_num_diff(model);
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data_num_diff = model_num_diff.createData();

  // Generating random values for the state and control
  Eigen::VectorXd x = model->get_state()->rand();
  Eigen::VectorXd u = Eigen::VectorXd::Random(model->get_nu());

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

  // Computing the action derivatives
  x = model->get_state()->rand();
  model->calc(data, x);
  model->calcDiff(data, x);

  model_num_diff.calc(data_num_diff, x);
  model_num_diff.calcDiff(data_num_diff, x);

  // Checking the partial derivatives against NumDiff
  BOOST_CHECK((data->Lx - data_num_diff->Lx).isZero(NUMDIFF_MODIFIER * tol));
  if (model_num_diff.get_with_gauss_approx()) {
    BOOST_CHECK((data->Lxx - data_num_diff->Lxx).isZero(NUMDIFF_MODIFIER * tol));
  } else {
    BOOST_CHECK((data_num_diff->Lxx).isZero(tol));
  }
}

//----------------------------------------------------------------------------//

void register_action_model_unit_tests(DifferentialActionModelTypes::Type action_type, 
                                      PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL, 
                                      ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  boost::test_tools::output_test_stream test_name;
  switch(action_type){
      case DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_TalosArm:
        test_name << "test_" << action_type << "_" << ref_type << "_" << mask_type;
        break;
      case DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_HyQ:
        test_name << "test_" << action_type << "_" << ref_type << "_" << mask_type;
        break;
      case DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_TalosArm:
        test_name << "test_" << action_type << "_" << ref_type;
        break;
      case DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_HyQ:
        test_name << "test_" << action_type << "_" << ref_type;
        break;
      case DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm:
        test_name << "test_" << action_type;
        break;
      case DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm_Squashed:
        test_name << "test_" << action_type;
        break;
      default:
        throw_pretty(__FILE__ ": Wrong DifferentialActionModelTypes::Type given");
        break;
  }
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  ts->add(BOOST_TEST_CASE(boost::bind(&test_check_data, action_type, ref_type, mask_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_state, action_type, ref_type, mask_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_a_cost, action_type, ref_type, mask_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_partial_derivatives_against_numdiff, action_type, ref_type, mask_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_quasi_static, action_type, ref_type, mask_type)));
  framework::master_test_suite().add(ts);
}

bool init_function() {

  // free
  for (size_t i = 0; i < DifferentialActionModelTypes::all.size(); ++i) {
    if(DifferentialActionModelTypes::all[i] == DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm ||
       DifferentialActionModelTypes::all[i] == DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm_Squashed){ 
    //   std::cout << "Register " << DifferentialActionModelTypes::all[i] << std::endl;
      register_action_model_unit_tests(DifferentialActionModelTypes::all[i]);
    }
  }
  // 3D contact
  for (size_t i = 0; i < DifferentialActionModelTypes::all.size(); ++i) {
    if(DifferentialActionModelTypes::all[i] == DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_TalosArm ||
       DifferentialActionModelTypes::all[i] == DifferentialActionModelTypes::DifferentialActionModelContact3DFwdDynamics_HyQ){
      for (size_t j = 0; j < PinocchioReferenceTypes::all.size(); ++j){
        // std::cout << "Register " << DifferentialActionModelTypes::all[i] << "_" <<  PinocchioReferenceTypes::all[j] << std::endl;
        register_action_model_unit_tests(DifferentialActionModelTypes::all[i], PinocchioReferenceTypes::all[j]);
      }
    }
  }   

//   // 1D contact
//   for (size_t i = 0; i < DifferentialActionModelTypes::all.size(); ++i) {
//     if(DifferentialActionModelTypes::all[i] == DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_TalosArm ||
//        DifferentialActionModelTypes::all[i] == DifferentialActionModelTypes::DifferentialActionModelContact1DFwdDynamics_HyQ){
//       for (size_t j = 0; j < PinocchioReferenceTypes::all.size(); ++j){
//         for (size_t k = 0; k < ContactModelMaskTypes::all.size(); ++k){
//             // std::cout << "Register " << DifferentialActionModelTypes::all[i] << "_" <<  PinocchioReferenceTypes::all[j] << "_" << ContactModelMaskTypes::all[k] << std::endl;
//           register_action_model_unit_tests(DifferentialActionModelTypes::all[i], PinocchioReferenceTypes::all[j], ContactModelMaskTypes::all[k]);
//         }
//       }
//     }
//   }

//   for (size_t i = 0; i < DifferentialActionModelTypes::all.size(); ++i) {
//     register_action_model_unit_tests(DifferentialActionModelTypes::all[i]);
//   }
  return true;
}

int main(int argc, char** argv) { return ::boost::unit_test::unit_test_main(&init_function, argc, argv); }

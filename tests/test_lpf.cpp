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

#include "common.hpp"
#include "factory/diff-action.hpp"
#include "factory/lpf.hpp"

#include <crocoddyl/core/integrator/euler.hpp>

using namespace boost::unit_test;
using namespace sobec::unittest;

//----------------------------------------------------------------------------//

void test_check_data(
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  // create the model
  ActionModelLPFFactory factory_iam;
  const boost::shared_ptr<sobec::IntegratedActionModelLPF>& model =
      factory_iam.create(iam_type, dam_type, ref_type, mask_type);

  // Run the print function
  std::ostringstream tmp;
  tmp << *model;

  // create the corresponding data object
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
      model->createData();

  BOOST_CHECK(model->checkData(data));
}

void test_calc_returns_state(
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  // create the model
  ActionModelLPFFactory factory_iam;
  const boost::shared_ptr<sobec::IntegratedActionModelLPF>& model =
      factory_iam.create(iam_type, dam_type, ref_type, mask_type);
  // create the corresponding data object
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
      model->createData();

  // Generating random state and control vectors
  const Eigen::VectorXd& x = model->get_state()->rand();
  const Eigen::VectorXd& u = Eigen::VectorXd::Random(model->get_nw());

  // Getting the state dimension from calc() call
  model->calc(data, x, u);

  BOOST_CHECK(static_cast<std::size_t>(data->xnext.size()) ==
              model->get_state()->get_nx());
}

void test_calc_returns_a_cost(
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  // create the model
  ActionModelLPFFactory factory_iam;
  const boost::shared_ptr<sobec::IntegratedActionModelLPF>& model =
      factory_iam.create(iam_type, dam_type, ref_type, mask_type);

  // create the corresponding data object and set the cost to nan
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
      model->createData();

  data->cost = nan("");

  // Getting the cost value computed by calc()
  const Eigen::VectorXd& x = model->get_state()->rand();
  const Eigen::VectorXd& u = Eigen::VectorXd::Random(model->get_nw());
  model->calc(data, x, u);

  // Checking that calc returns a cost value
  BOOST_CHECK(!std::isnan(data->cost));
}

void test_partial_derivatives_against_numdiff(
    const boost::shared_ptr<sobec::IntegratedActionModelLPF>& model) {
  // create the corresponding data object and set the cost to nan
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
      model->createData();

  crocoddyl::ActionModelNumDiff model_num_diff(model);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data_num_diff =
      model_num_diff.createData();

  // Generating random values for the state and control
  Eigen::VectorXd x = model->get_state()->rand();
  const Eigen::VectorXd& u = Eigen::VectorXd::Random(model->get_nw());

  // Computing the action derivatives
  model->calc(data, x, u);
  model->calcDiff(data, x, u);

  model_num_diff.calc(data_num_diff, x, u);
  model_num_diff.calcDiff(data_num_diff, x, u);

  // Checking the partial derivatives against NumDiff
  double tol = sqrt(model_num_diff.get_disturbance());

  // const std::size_t nv = model->get_state()->get_nv();
  // const std::size_t nu = model->get_differential()->get_nu();
  // std::cout << " Fx - Fx_nd [q]: " << std::endl;
  // std::cout << (data->Fx -
  // data_num_diff->Fx).leftCols(nv).lpNorm<Eigen::Infinity>() << std::endl;
  // std::cout << " Fx - Fx_nd [v]: " << std::endl;
  // std::cout << (data->Fx - data_num_diff->Fx).block(0, nv, 2*nv+nu,
  // nv).lpNorm<Eigen::Infinity>() << std::endl; std::cout << " Fx - Fx_nd
  // [tau]: " << std::endl; std::cout << (data->Fx -
  // data_num_diff->Fx).rightCols(nu).lpNorm<Eigen::Infinity>() << std::endl;

  BOOST_CHECK((data->Fx - data_num_diff->Fx).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((data->Fu - data_num_diff->Fu).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((data->Lx - data_num_diff->Lx).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((data->Lu - data_num_diff->Lu).isZero(NUMDIFF_MODIFIER * tol));
  if (model_num_diff.get_with_gauss_approx()) {
    BOOST_CHECK(
        (data->Lxx - data_num_diff->Lxx).isZero(NUMDIFF_MODIFIER * tol));
    BOOST_CHECK(
        (data->Lxu - data_num_diff->Lxu).isZero(NUMDIFF_MODIFIER * tol));
    BOOST_CHECK(
        (data->Luu - data_num_diff->Luu).isZero(NUMDIFF_MODIFIER * tol));
  } else {
    BOOST_CHECK((data_num_diff->Lxx).isZero(tol));
    BOOST_CHECK((data_num_diff->Lxu).isZero(tol));
    BOOST_CHECK((data_num_diff->Luu).isZero(tol));
  }
}

void test_partial_derivatives_action_model(
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  // create the model
  ActionModelLPFFactory factory;
  const boost::shared_ptr<sobec::IntegratedActionModelLPF>& model =
      factory.create(iam_type, dam_type, ref_type, mask_type);
  test_partial_derivatives_against_numdiff(model);
}



void test_calc_equivalent_euler( 
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z){
    
  // Create IAM LPF
  ActionModelLPFFactory factory_iam;
  const boost::shared_ptr<sobec::IntegratedActionModelLPF>& modelLPF =
      factory_iam.create(iam_type, dam_type, ref_type, mask_type);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataLPF = modelLPF->createData();

  // Create IAM Euler from DAM and iamLPF.dt (with cost residual)
  DifferentialActionModelFactory factory_dam;
  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dam = 
      factory_dam.create(dam_type, ref_type, mask_type);
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> modelEuler = 
    boost::make_shared<crocoddyl::IntegratedActionModelEuler>(dam, modelLPF->get_dt(), true);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEuler = modelEuler->createData();

  // Generating random values for the state and control
  Eigen::VectorXd y = modelLPF->get_state()->rand();
  const Eigen::VectorXd& w = Eigen::VectorXd::Random(modelLPF->get_nw());
  std::size_t nx = modelEuler->get_state()->get_nx();
  std::size_t ndx = modelEuler->get_state()->get_ndx();
  std::size_t nv = modelEuler->get_state()->get_nv();
  // Checking the partial derivatives against NumDiff
  double tol = 1e-2;

  // Computing the action 
  modelLPF->calc(dataLPF, y, w);
  if(!(dataLPF->xnext.tail(nv) - w).isZero(tol)){
    std::cout << " lpf " << std::endl;
    std::cout << dataLPF->xnext.tail(nv) << std::endl;
    std::cout << w << std::endl;
    // std::cout << dataEuler->Lxx << std::endl;
  }
  BOOST_CHECK((dataLPF->xnext.tail(nv) - w).isZero(tol));
  modelEuler->calc(dataEuler, y.head(nx), w);
  BOOST_CHECK((dataLPF->xnext.head(nx) - dataEuler->xnext).isZero(tol));
  // Computing the derivatives
  modelLPF->calcDiff(dataLPF, y, w);
  modelEuler->calcDiff(dataEuler, y.head(nx), w);
  BOOST_CHECK((dataLPF->Fx.topLeftCorner(nv, ndx) - dataEuler->Fx).isZero(tol));
  BOOST_CHECK((dataLPF->Fu.topLeftCorner(nv, ndx) - dataEuler->Fu).isZero(tol));
  BOOST_CHECK((dataLPF->Lx.head(nx) - dataEuler->Lx).isZero(tol));
  BOOST_CHECK((dataLPF->Lu.head(nv) - dataEuler->Lu).isZero(tol));
  BOOST_CHECK((dataLPF->Lxx.topLeftCorner(ndx, ndx) - dataEuler->Lxx).isZero(tol));
  // if(!(dataLPF->Lxx.topLeftCorner(ndx, nv) - dataEuler->Lxx).isZero(tol)){
  //   std::cout << " lpf " << std::endl;
  //   std::cout << dataLPF->Lxx.topLeftCorner(ndx, ndx) << std::endl;
  //   std::cout << " euler " << std::endl;
  //   std::cout << dataEuler->Lxx << std::endl;
  // }
  BOOST_CHECK((dataLPF->Lxu.topLeftCorner(ndx, nv) - dataEuler->Lxu).isZero(tol));
  BOOST_CHECK((dataLPF->Luu.topLeftCorner(nv,nv) - dataEuler->Luu).isZero(tol));
}


//----------------------------------------------------------------------------//

void register_action_model_unit_tests(
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  boost::test_tools::output_test_stream test_name;
  if (dam_type == DifferentialActionModelTypes::
                      DifferentialActionModelContact1DFwdDynamics_TalosArm ||
      dam_type == DifferentialActionModelTypes::
                      DifferentialActionModelContact1DFwdDynamics_HyQ) {
    test_name << "test_" << iam_type << "_" << dam_type << "_" << ref_type
              << "_" << mask_type;
  } else {
    test_name << "test_" << iam_type << "_" << dam_type << "_" << ref_type;
  }
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  // ts->add(BOOST_TEST_CASE(
  //     boost::bind(&test_check_data, iam_type, dam_type, ref_type,
  //     mask_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_state, iam_type,
                                      dam_type, ref_type, mask_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_a_cost, iam_type,
                                      dam_type, ref_type, mask_type)));
  ts->add(
      BOOST_TEST_CASE(boost::bind(&test_partial_derivatives_action_model,
                                  iam_type, dam_type, ref_type, mask_type)));
  if(iam_type == ActionModelLPFTypes::Type::IntegratedActionModelLPF_alpha0 ||
     iam_type == ActionModelLPFTypes::Type::IntegratedActionModelLPF_NONE){
    ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_equivalent_euler, iam_type, dam_type, ref_type, mask_type)));
  }
  framework::master_test_suite().add(ts);
}

bool init_function() {
  // free
  // register_action_model_unit_tests(ActionModelLPFTypes::IntegratedActionModelLPF,
  //                                  DifferentialActionModelTypes::DifferentialActionModelFreeFwdDynamics_TalosArm);
  for (size_t i = 0; i < ActionModelLPFTypes::all.size(); ++i) {
    for (size_t j = 0; j < DifferentialActionModelTypes::all.size(); ++j) {
      if (DifferentialActionModelTypes::all[j] ==
              DifferentialActionModelTypes::
                  DifferentialActionModelFreeFwdDynamics_TalosArm ||
          DifferentialActionModelTypes::all[j] ==
              DifferentialActionModelTypes::
                  DifferentialActionModelFreeFwdDynamics_TalosArm_Squashed) {
        register_action_model_unit_tests(ActionModelLPFTypes::all[i],
                                         DifferentialActionModelTypes::all[j]);
      }
    }
  }
  // 3D contact
  for (size_t i = 0; i < ActionModelLPFTypes::all.size(); ++i) {
    for (size_t j = 0; j < DifferentialActionModelTypes::all.size(); ++j) {
      if (DifferentialActionModelTypes::all[j] ==
              DifferentialActionModelTypes::
                  DifferentialActionModelContact3DFwdDynamics_TalosArm ||
          DifferentialActionModelTypes::all[j] ==
              DifferentialActionModelTypes::
                  DifferentialActionModelContact3DFwdDynamics_HyQ) {
        for (size_t k = 0; k < PinocchioReferenceTypes::all.size(); ++k) {
          register_action_model_unit_tests(ActionModelLPFTypes::all[i],
                                           DifferentialActionModelTypes::all[j],
                                           PinocchioReferenceTypes::all[k]);
        }
      }
    }
  }
  // 1D contact
  for (size_t i = 0; i < ActionModelLPFTypes::all.size(); ++i) {
    for (size_t j = 0; j < DifferentialActionModelTypes::all.size(); ++j) {
      if (DifferentialActionModelTypes::all[j] ==
              DifferentialActionModelTypes::
                  DifferentialActionModelContact1DFwdDynamics_TalosArm ||
          DifferentialActionModelTypes::all[j] ==
              DifferentialActionModelTypes::
                  DifferentialActionModelContact1DFwdDynamics_HyQ) {
        for (size_t k = 0; k < PinocchioReferenceTypes::all.size(); ++k) {
          for (size_t l = 0; l < ContactModelMaskTypes::all.size(); ++l) {
            register_action_model_unit_tests(
                ActionModelLPFTypes::all[i],
                DifferentialActionModelTypes::all[j],
                PinocchioReferenceTypes::all[k], ContactModelMaskTypes::all[l]);
          }
        }
      }
    }
  }

  return true;
}

int main(int argc, char** argv) {
  return ::boost::unit_test::unit_test_main(&init_function, argc, argv);
}

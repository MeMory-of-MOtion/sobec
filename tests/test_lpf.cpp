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
#include "factory/diff-action.hpp"
#include "factory/lpf.hpp"

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



void test_partial_derivatives_against_numdiff_terminal(
    const boost::shared_ptr<sobec::IntegratedActionModelLPF>& model) {
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

void test_partial_derivatives_action_model_terminal(
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  // create the model
  ActionModelLPFFactory factory;
  const boost::shared_ptr<sobec::IntegratedActionModelLPF>& model =
      factory.create(iam_type, dam_type, ref_type, mask_type);
  model->set_dt(0);
  test_partial_derivatives_against_numdiff_terminal(model);
}


void test_calc_alpha0_equivalent_euler(
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  // Create IAM LPF
  ActionModelLPFFactory factory_iam;
  const boost::shared_ptr<sobec::IntegratedActionModelLPF>& modelLPF =
      factory_iam.create(iam_type, dam_type, ref_type, mask_type);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataLPF =
      modelLPF->createData();

  // Create IAM Euler from iamLPF.DAM and iamLPF.dt (with cost residual)
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> modelEuler = 
    boost::make_shared<crocoddyl::IntegratedActionModelEuler>(modelLPF->get_differential(), modelLPF->get_dt(), true);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEuler = modelEuler->createData();

  // Generating random values for the state and control
  std::size_t nx = modelEuler->get_state()->get_nx();
  std::size_t ndx = modelEuler->get_state()->get_ndx();
  std::size_t ntau = boost::static_pointer_cast<sobec::IntegratedActionModelLPF>(modelLPF)->get_ntau();
  std::size_t ntau_state = boost::static_pointer_cast<sobec::StateLPF>(modelLPF->get_state())->get_ntau();
  const Eigen::VectorXd y = modelLPF->get_state()->rand();
  const Eigen::VectorXd& w = Eigen::VectorXd::Random(modelLPF->get_nw());
  const Eigen::VectorXd x = y.head(nx);
  const Eigen::VectorXd tau = y.tail(ntau);

  // Check stuff
  BOOST_CHECK(ntau == modelEuler->get_nu());
  BOOST_CHECK(ntau_state == modelEuler->get_nu());
  const std::vector<int>& lpf_torque_ids = modelLPF->get_lpf_torque_ids();
  const std::vector<int>& non_lpf_torque_ids = modelLPF->get_non_lpf_torque_ids();
  BOOST_CHECK(lpf_torque_ids.size() == modelEuler->get_nu());
  BOOST_CHECK(non_lpf_torque_ids.size() == 0);
  BOOST_CHECK(non_lpf_torque_ids.size() + lpf_torque_ids.size() == modelEuler->get_nu());

  // Checking the partial derivatives against NumDiff
  double tol = 1e-6;

  // Computing the action 
  modelLPF->calc(dataLPF, y, w);
  modelEuler->calc(dataEuler, x, tau);
  // Test perfect actuation and state integration
  BOOST_CHECK((dataLPF->xnext.tail(ntau) - w).isZero(tol));
  BOOST_CHECK((dataLPF->xnext.head(nx) - dataEuler->xnext).isZero(tol));
  BOOST_CHECK((dataLPF->r.head(modelEuler->get_nr()) - dataEuler->r).isZero(tol));
  BOOST_CHECK( (dataLPF->cost - dataEuler->cost) <= tol);

  // Test terminal calc
  modelLPF->set_dt(0.);
  modelEuler->set_dt(0.);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataLPFTerminal = modelLPF->createData();
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEulerTerminal = modelEuler->createData();
  modelLPF->calc(dataLPFTerminal, y);
  modelEuler->calc(dataEulerTerminal, x);
  BOOST_CHECK((dataLPFTerminal->r.head(modelEuler->get_nr()) - dataEulerTerminal->r).isZero(tol));
  BOOST_CHECK((dataLPFTerminal->cost - dataEulerTerminal->cost) <= tol);
}

void test_calc_NONE_equivalent_euler(
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  // Create IAM LPF
  ActionModelLPFFactory factory_iam;
  const boost::shared_ptr<sobec::IntegratedActionModelLPF>& modelLPF =
      factory_iam.create(iam_type, dam_type, ref_type, mask_type);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataLPF =
      modelLPF->createData();

  // Create IAM Euler from DAM and iamLPF.dt (with cost residual)
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> modelEuler = 
    boost::make_shared<crocoddyl::IntegratedActionModelEuler>(modelLPF->get_differential(), modelLPF->get_dt(), true);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEuler = modelEuler->createData();

  // Generating random values for the state and control
  std::size_t nx = modelEuler->get_state()->get_nx();
  std::size_t ndx = modelEuler->get_state()->get_ndx();
  // std::size_t nv = modelEuler->get_state()->get_nv();
  std::size_t ntau =
      boost::static_pointer_cast<sobec::IntegratedActionModelLPF>(modelLPF)
          ->get_ntau();
  std::size_t ntau_state =
      boost::static_pointer_cast<sobec::StateLPF>(modelLPF->get_state())
          ->get_ntau();
  BOOST_CHECK(ntau == 0);
  BOOST_CHECK(ntau_state == 0);
  const std::vector<int>& lpf_torque_ids = modelLPF->get_lpf_torque_ids();
  const std::vector<int>& non_lpf_torque_ids = modelLPF->get_non_lpf_torque_ids();
  BOOST_CHECK(lpf_torque_ids.size() == 0);
  BOOST_CHECK(non_lpf_torque_ids.size() + lpf_torque_ids.size() == modelEuler->get_nu());
  BOOST_CHECK(non_lpf_torque_ids.size() == modelEuler->get_nu() );

  const Eigen::VectorXd y = modelLPF->get_state()->rand();
  BOOST_CHECK(y.size() == nx);
  const Eigen::VectorXd& w = Eigen::VectorXd::Random(modelLPF->get_nw());
  BOOST_CHECK(w.size() == modelEuler->get_nu());

  // Checking the partial derivatives against NumDiff
  double tol = 1e-6;
  // Computing the action
  modelLPF->calc(dataLPF, y, w);
  modelEuler->calc(dataEuler, y, w);
  // Test perfect actuation and state integration
  BOOST_CHECK((dataLPF->xnext - dataEuler->xnext).isZero(tol));
  BOOST_CHECK((dataLPF->r - dataEuler->r).isZero(tol));
  BOOST_CHECK( (dataLPF->cost - dataEuler->cost) <= tol);

  // Test terminal calc
  modelLPF->set_dt(0.);
  modelEuler->set_dt(0.);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataLPFTerminal = modelLPF->createData();
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEulerTerminal = modelEuler->createData();
  modelLPF->calc(dataLPFTerminal, y);
  modelEuler->calc(dataEulerTerminal, y);
  BOOST_CHECK((dataLPFTerminal->r - dataEulerTerminal->r).isZero(tol));
  BOOST_CHECK((dataLPFTerminal->cost - dataEulerTerminal->cost) <= tol);
}


// void test_calcDiff_NONE_equivalent_euler( 
//     ActionModelLPFTypes::Type iam_type,
//     DifferentialActionModelTypes::Type dam_type,
//     PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
//     ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z){
    
//   // Create IAM LPF
//   ActionModelLPFFactory factory_iam;
//   const boost::shared_ptr<sobec::IntegratedActionModelLPF>& modelLPF =
//       factory_iam.create(iam_type, dam_type, ref_type, mask_type);
//   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataLPF = modelLPF->createData();

//   // Create IAM Euler from DAM and iamLPF.dt (with cost residual)
//   boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> modelEuler = 
//     boost::make_shared<crocoddyl::IntegratedActionModelEuler>(modelLPF->get_differential(), modelLPF->get_dt(), true);
//   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEuler = modelEuler->createData();

//   // Generating random values for the state and control
//   std::size_t nx = modelEuler->get_state()->get_nx();
//   std::size_t ndx = modelEuler->get_state()->get_ndx();
//   std::size_t nv = modelEuler->get_state()->get_nv();
//   std::size_t ntau = boost::static_pointer_cast<sobec::IntegratedActionModelLPF>(modelLPF)->get_ntau();
//   std::size_t nu = modelEuler->get_nu();
//   const Eigen::VectorXd y = modelLPF->get_state()->rand();
//   const Eigen::VectorXd& w = Eigen::VectorXd::Random(modelLPF->get_nw());
//   const Eigen::VectorXd x = y.head(nx);
//   Eigen::VectorXd tau = w; 
//   const std::vector<int>& lpf_torque_ids = modelLPF->get_lpf_torque_ids();
//   const std::vector<int>& non_lpf_torque_ids = modelLPF->get_non_lpf_torque_ids();
//   BOOST_CHECK(ntau == lpf_torque_ids.size());
//   for(std::size_t i=0; i<lpf_torque_ids.size();i++){
//     tau(lpf_torque_ids[i]) = y.tail(ntau)[i];
//   }
//   BOOST_CHECK(non_lpf_torque_ids.size() + lpf_torque_ids.size() == nu );

//   // Checking the partial derivatives against NumDiff
//   double tol = 1e-6; 

//   // Computing the action 
//   modelLPF->calc(dataLPF, y, w);
//   modelEuler->calc(dataEuler, x, tau); 
//   // Computing the derivatives
//   modelLPF->calcDiff(dataLPF, y, w);
//   modelEuler->calcDiff(dataEuler, x, tau); 

//   // Case no joint is LPF
//   BOOST_CHECK((dataLPF->Fx - dataEuler->Fx).isZero(tol));
//   BOOST_CHECK((dataLPF->Fu - dataEuler->Fu).isZero(tol));
//   BOOST_CHECK((dataLPF->Lx - dataEuler->Lx).isZero(tol));
//   BOOST_CHECK((dataLPF->Lu - dataEuler->Lu).isZero(tol));
//   BOOST_CHECK((dataLPF->Lxx - dataEuler->Lxx).isZero(tol));
//   BOOST_CHECK((dataLPF->Lxu - dataEuler->Lxu).isZero(tol));
//   BOOST_CHECK((dataLPF->Luu - dataEuler->Luu).isZero(tol));
// }


void test_calcDiff_explicit_equivalent_euler( 
    ActionModelLPFTypes::Type iam_type,
    DifferentialActionModelTypes::Type dam_type,
    PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
    ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
  // Create IAM LPF
  ActionModelLPFFactory factory_iam;
  const boost::shared_ptr<sobec::IntegratedActionModelLPF>& modelLPF =
      factory_iam.create(iam_type, dam_type, ref_type, mask_type);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataLPF =
      modelLPF->createData();

  // Create IAM Euler from DAM and iamLPF.dt (with cost residual)
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> modelEuler = 
    boost::make_shared<crocoddyl::IntegratedActionModelEuler>(modelLPF->get_differential(), modelLPF->get_dt(), true);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEuler = modelEuler->createData();

  // Generating random values for the state and control
  std::size_t nx = modelEuler->get_state()->get_nx();
  std::size_t ndx = modelEuler->get_state()->get_ndx();
  // std::size_t nv = modelEuler->get_state()->get_nv();
  std::size_t ntau =
      boost::static_pointer_cast<sobec::IntegratedActionModelLPF>(modelLPF)
          ->get_ntau();
  std::size_t nu = modelEuler->get_nu();
  const Eigen::VectorXd y = modelLPF->get_state()->rand();
  const Eigen::VectorXd& w = Eigen::VectorXd::Random(modelLPF->get_nw());
  const Eigen::VectorXd x = y.head(nx);
  Eigen::VectorXd tau = w;
  const std::vector<int>& lpf_torque_ids = modelLPF->get_lpf_torque_ids();
  const std::vector<int>& non_lpf_torque_ids = modelLPF->get_non_lpf_torque_ids();
  BOOST_CHECK(ntau == lpf_torque_ids.size());
  for(std::size_t i=0; i<lpf_torque_ids.size();i++){
    tau(lpf_torque_ids[i]) = y.tail(ntau)[i];
  }
  BOOST_CHECK(non_lpf_torque_ids.size() + lpf_torque_ids.size() == nu );

  // Checking the partial derivatives against NumDiff
  double tol = 1e-6; 

  // Computing the action
  modelLPF->calc(dataLPF, y, w);
  modelEuler->calc(dataEuler, x, tau); 
  // Computing the derivatives
  modelLPF->calcDiff(dataLPF, y, w);
  modelEuler->calcDiff(dataEuler, x, tau); 

  // All or some joint are LPF
  // Size varying stuff  
  const Eigen::MatrixXd& Fu_LPF = dataLPF->Fx.topRightCorner(ndx, ntau);
  const Eigen::MatrixXd& Lu_LPF = dataLPF->Lx.tail(ntau);
  const Eigen::MatrixXd& Lxu_LPF = dataLPF->Lxx.topRightCorner(ndx, ntau);
  const Eigen::MatrixXd& Luu_LPF = dataLPF->Lxx.bottomRightCorner(ntau, ntau);
  for (std::size_t i = 0; i < lpf_torque_ids.size(); i++) {
    BOOST_CHECK(
        (Fu_LPF.col(i) - dataEuler->Fu.col(lpf_torque_ids[i])).isZero(tol));
    BOOST_CHECK((Lu_LPF(i) - dataEuler->Lu(lpf_torque_ids[i])) <= tol);
    BOOST_CHECK((Lxu_LPF.col(i) - dataEuler->Lxu.col(lpf_torque_ids[i])).isZero(tol));
    for(std::size_t j=0; j<lpf_torque_ids.size();j++){
      BOOST_CHECK( (Luu_LPF(i,j)- dataEuler->Luu(lpf_torque_ids[i], lpf_torque_ids[j]) ) <= tol );
    }
  }
  // Fixed size stuff
  const Eigen::MatrixXd& Fx_LPF = dataLPF->Fx.topLeftCorner(ndx, ndx);
  const Eigen::MatrixXd& Lx_LPF = dataLPF->Lx.head(ndx);
  const Eigen::MatrixXd& Lxx_LPF = dataLPF->Lxx.topLeftCorner(ndx, ndx);
  // Testing the partials w.r.t. u match blocks in partial w.r.t. augmented state y
  BOOST_CHECK((Fx_LPF - dataEuler->Fx).isZero(tol));
  BOOST_CHECK((Lx_LPF - dataEuler->Lx).isZero(tol));
  BOOST_CHECK((Lxx_LPF - dataEuler->Lxx).isZero(tol));

  // Non LPF dimensions
  // Size varying stuff  
  const Eigen::MatrixXd& Fu_nonLPF = dataLPF->Fu.topRows(ndx);
  const Eigen::MatrixXd& Lu_nonLPF = dataLPF->Lu;
  const Eigen::MatrixXd& Lxu_nonLPF = dataLPF->Lxu;
  const Eigen::MatrixXd& Luu_nonLPF = dataLPF->Luu;
  for(std::size_t i=0; i<non_lpf_torque_ids.size();i++){
    BOOST_CHECK((Fu_nonLPF.col(non_lpf_torque_ids[i]) - dataEuler->Fu.col(non_lpf_torque_ids[i])).isZero(tol));
    BOOST_CHECK((Lu_nonLPF(non_lpf_torque_ids[i]) - dataEuler->Lu(non_lpf_torque_ids[i])) <= tol);
    BOOST_CHECK((Lxu_nonLPF.topRows(ndx).col(non_lpf_torque_ids[i]) - dataEuler->Lxu.col(non_lpf_torque_ids[i])).isZero(tol));
    for(std::size_t j=0; j<non_lpf_torque_ids.size();j++){
      BOOST_CHECK((Luu_nonLPF(non_lpf_torque_ids[i],non_lpf_torque_ids[j]) - dataEuler->Luu(non_lpf_torque_ids[i], non_lpf_torque_ids[j])) <= tol );
    }
    for(std::size_t j=0; j<lpf_torque_ids.size();j++){
      BOOST_CHECK((Lxu_nonLPF.bottomRows(ntau)(j, non_lpf_torque_ids[i]) - dataEuler->Luu(lpf_torque_ids[j], non_lpf_torque_ids[i])) <= tol );
    }
  }


  // Test terminal calcDiff
  // Computing the action
  modelLPF->set_dt(0.);
  modelEuler->set_dt(0.);
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataLPFTerminal = modelLPF->createData();
  const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEulerTerminal = modelEuler->createData();
  modelLPF->calc(dataLPFTerminal, y);
  modelEuler->calc(dataEulerTerminal, x); 
  modelLPF->calcDiff(dataLPFTerminal, y);
  modelEuler->calcDiff(dataEulerTerminal, x); 
  // All or some joint are LPF
  // Size varying stuff  
  const Eigen::MatrixXd& Fu_LPF_term = dataLPFTerminal->Fx.topRightCorner(ndx, ntau);
  const Eigen::MatrixXd& Lu_LPF_term = dataLPFTerminal->Lx.tail(ntau);
  const Eigen::MatrixXd& Lxu_LPF_term = dataLPFTerminal->Lxx.topRightCorner(ndx, ntau);
  const Eigen::MatrixXd& Luu_LPF_term = dataLPFTerminal->Lxx.bottomRightCorner(ntau, ntau);
  for (std::size_t i = 0; i < lpf_torque_ids.size(); i++) {
    BOOST_CHECK(
        (Fu_LPF_term.col(i) - dataEulerTerminal->Fu.col(lpf_torque_ids[i])).isZero(tol));
    BOOST_CHECK((Lu_LPF_term(i) - dataEulerTerminal->Lu(lpf_torque_ids[i])) <= tol);
    BOOST_CHECK((Lxu_LPF_term.col(i) - dataEulerTerminal->Lxu.col(lpf_torque_ids[i])).isZero(tol));
    for(std::size_t j=0; j<lpf_torque_ids.size();j++){
      BOOST_CHECK( (Luu_LPF_term(i,j)- dataEulerTerminal->Luu(lpf_torque_ids[i], lpf_torque_ids[j]) ) <= tol );
    }
  }
  // Fixed size stuff
  const Eigen::MatrixXd& Fx_LPF_term = dataLPFTerminal->Fx.topLeftCorner(ndx, ndx);
  const Eigen::MatrixXd& Lx_LPF_term = dataLPFTerminal->Lx.head(ndx);
  const Eigen::MatrixXd& Lxx_LPF_term = dataLPFTerminal->Lxx.topLeftCorner(ndx, ndx);
  // Testing the partials w.r.t. u match blocks in partial w.r.t. augmented state y
  // if(!(Fx_LPF_term - dataEulerTerminal->Fx).isZero(tol)){
  //   std::cout << " Fx_lpf - Fx_euler terminal = " << std::endl;
  //   std::cout << (Fx_LPF_term - dataEulerTerminal->Fx) << std::endl;
  // }
  BOOST_CHECK((Fx_LPF_term - dataEulerTerminal->Fx).isZero(tol));
  BOOST_CHECK((Lx_LPF_term - dataEulerTerminal->Lx).isZero(tol));
  BOOST_CHECK((Lxx_LPF_term - dataEulerTerminal->Lxx).isZero(tol));

  // Non LPF dimensions
  // Size varying stuff  
  const Eigen::MatrixXd& Fu_nonLPF_term = dataLPFTerminal->Fu.topRows(ndx);
  const Eigen::MatrixXd& Lu_nonLPF_term = dataLPFTerminal->Lu;
  const Eigen::MatrixXd& Lxu_nonLPF_term = dataLPFTerminal->Lxu;
  const Eigen::MatrixXd& Luu_nonLPF_term = dataLPFTerminal->Luu;
  for(std::size_t i=0; i<non_lpf_torque_ids.size();i++){
    BOOST_CHECK((Fu_nonLPF_term.col(non_lpf_torque_ids[i]) - dataEulerTerminal->Fu.col(non_lpf_torque_ids[i])).isZero(tol));
    BOOST_CHECK((Lu_nonLPF_term(non_lpf_torque_ids[i]) - dataEulerTerminal->Lu(non_lpf_torque_ids[i])) <= tol);
    BOOST_CHECK((Lxu_nonLPF_term.topRows(ndx).col(non_lpf_torque_ids[i]) - dataEulerTerminal->Lxu.col(non_lpf_torque_ids[i])).isZero(tol));
    for(std::size_t j=0; j<non_lpf_torque_ids.size();j++){
      BOOST_CHECK((Luu_nonLPF_term(non_lpf_torque_ids[i],non_lpf_torque_ids[j]) - dataEulerTerminal->Luu(non_lpf_torque_ids[i], non_lpf_torque_ids[j])) <= tol );
    }
    for(std::size_t j=0; j<lpf_torque_ids.size();j++){
      BOOST_CHECK((Lxu_nonLPF_term.bottomRows(ntau)(j, non_lpf_torque_ids[i]) - dataEulerTerminal->Luu(lpf_torque_ids[j], non_lpf_torque_ids[i])) <= tol );
    }
  }
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
  // seems incompatible with euler equivalence test
  // ts->add(
  //     BOOST_TEST_CASE(boost::bind(&test_partial_derivatives_action_model_terminal,
  //                                 iam_type, dam_type, ref_type, mask_type))); 
  // Equivalence with Euler when alpha=0 or ntau=0
  if(iam_type == ActionModelLPFTypes::Type::IntegratedActionModelLPF_alpha0){
    ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_alpha0_equivalent_euler, iam_type, dam_type, ref_type, mask_type)));
  }
  if(iam_type == ActionModelLPFTypes::Type::IntegratedActionModelLPF_NONE){
    ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_NONE_equivalent_euler, iam_type, dam_type, ref_type, mask_type)));
    // ts->add(BOOST_TEST_CASE(boost::bind(&test_calcDiff_NONE_equivalent_euler, iam_type, dam_type, ref_type, mask_type)));
  }
  ts->add(
      BOOST_TEST_CASE(boost::bind(&test_calcDiff_explicit_equivalent_euler,
                                  iam_type, dam_type, ref_type, mask_type)));
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

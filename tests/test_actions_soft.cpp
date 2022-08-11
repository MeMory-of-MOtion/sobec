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




// void test_partial_derivatives_against_numdiff_terminal(
//     const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& model) {
//   // create the corresponding data object and set the cost to nan
//   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data =
//       model->createData();

//   crocoddyl::ActionModelNumDiff model_num_diff(model);
//   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data_num_diff =
//       model_num_diff.createData();

//   // Generating random values for the state and control
//   Eigen::VectorXd x = model->get_state()->rand();

//   // Computing the action derivatives
//   model->calc(data, x);
//   model->calcDiff(data, x);
//   model_num_diff.calc(data_num_diff, x);
//   model_num_diff.calcDiff(data_num_diff, x);

//   // Checking the partial derivatives against NumDiff
//   double tol = sqrt(model_num_diff.get_disturbance());
//   // Checking the partial derivatives against NumDiff
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
// }

// void test_partial_derivatives_action_model_terminal(
//     IAMSoftContactTypes::Type iam_type,
//     DAMSoftContactTypes::Type dam_type,
//     PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
//     ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
//   // create the model
//   IAMSoftContactFactory factory;
//   const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& model =
//       factory.create(iam_type, dam_type, ref_type, mask_type);
//   model->set_dt(0);
//   test_partial_derivatives_against_numdiff_terminal(model);
// }




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




// // void test_calcDiff_NONE_equivalent_euler(
// //     IAMSoftContactTypes::Type iam_type,
// //     DAMSoftContactTypes::Type dam_type,
// //     PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
// //     ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z){

// //   // Create IAM LPF
// //   IAMSoftContactFactory factory_iam;
// //   const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& modelSoft =
// //       factory_iam.create(iam_type, dam_type, ref_type, mask_type);
// //   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataSoft =
// //   modelSoft->createData();

// //   // Create IAM Euler from DAM and iamLPF.dt (with cost residual)
// //   boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> modelEuler =
// //     boost::make_shared<crocoddyl::IntegratedActionModelEuler>(modelSoft->get_differential(),
// //     modelSoft->get_dt(), true);
// //   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEuler =
// //   modelEuler->createData();

// //   // Generating random values for the state and control
// //   std::size_t nx = modelEuler->get_state()->get_nx();
// //   std::size_t ndx = modelEuler->get_state()->get_ndx();
// //   std::size_t nv = modelEuler->get_state()->get_nv();
// //   std::size_t ntau =
// //   boost::static_pointer_cast<sobec::IAMSoftContact3DAugmented>(modelSoft)->get_ntau();
// //   std::size_t nu = modelEuler->get_nu();
// //   const Eigen::VectorXd y = modelSoft->get_state()->rand();
// //   const Eigen::VectorXd& w = Eigen::VectorXd::Random(modelSoft->get_nu());
// //   const Eigen::VectorXd x = y.head(nx);
// //   Eigen::VectorXd tau = w;
// //   const std::vector<int>& lpf_torque_ids = modelSoft->get_lpf_torque_ids();
// //   const std::vector<int>& non_lpf_torque_ids =
// //   modelSoft->get_non_lpf_torque_ids(); BOOST_CHECK(ntau ==
// //   lpf_torque_ids.size()); for(std::size_t i=0; i<lpf_torque_ids.size();i++){
// //     tau(lpf_torque_ids[i]) = y.tail(ntau)[i];
// //   }
// //   BOOST_CHECK(non_lpf_torque_ids.size() + lpf_torque_ids.size() == nu );

// //   // Checking the partial derivatives against NumDiff
// //   double tol = 1e-6;

// //   // Computing the action
// //   modelSoft->calc(dataSoft, y, w);
// //   modelEuler->calc(dataEuler, x, tau);
// //   // Computing the derivatives
// //   modelSoft->calcDiff(dataSoft, y, w);
// //   modelEuler->calcDiff(dataEuler, x, tau);

// //   // Case no joint is LPF
// //   BOOST_CHECK((dataSoft->Fx - dataEuler->Fx).isZero(tol));
// //   BOOST_CHECK((dataSoft->Fu - dataEuler->Fu).isZero(tol));
// //   BOOST_CHECK((dataSoft->Lx - dataEuler->Lx).isZero(tol));
// //   BOOST_CHECK((dataSoft->Lu - dataEuler->Lu).isZero(tol));
// //   BOOST_CHECK((dataSoft->Lxx - dataEuler->Lxx).isZero(tol));
// //   BOOST_CHECK((dataSoft->Lxu - dataEuler->Lxu).isZero(tol));
// //   BOOST_CHECK((dataSoft->Luu - dataEuler->Luu).isZero(tol));
// // }

// void test_calcDiff_explicit_equivalent_euler(
//     IAMSoftContactTypes::Type iam_type,
//     DAMSoftContactTypes::Type dam_type,
//     PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL,
//     ContactModelMaskTypes::Type mask_type = ContactModelMaskTypes::Z) {
//   // Create IAM LPF
//   IAMSoftContactFactory factory_iam;
//   const boost::shared_ptr<sobec::IAMSoftContact3DAugmented>& modelSoft =
//       factory_iam.create(iam_type, dam_type, ref_type, mask_type);
//   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataSoft =
//       modelSoft->createData();

//   // Create IAM Euler from DAM and iamLPF.dt (with cost residual)
//   boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> modelEuler =
//       boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
//           modelSoft->get_differential(), modelSoft->get_dt(), true);
//   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEuler =
//       modelEuler->createData();

//   // Generating random values for the state and control
//   std::size_t nx = modelEuler->get_state()->get_nx();
//   std::size_t ndx = modelEuler->get_state()->get_ndx();
//   // std::size_t nv = modelEuler->get_state()->get_nv();
//   std::size_t ntau =
//       boost::static_pointer_cast<sobec::IAMSoftContact3DAugmented>(modelSoft)
//           ->get_ntau();
//   std::size_t nu = modelEuler->get_nu();
//   const Eigen::VectorXd y = modelSoft->get_state()->rand();
//   const Eigen::VectorXd& w = Eigen::VectorXd::Random(modelSoft->get_nu());
//   const Eigen::VectorXd x = y.head(nx);
//   Eigen::VectorXd tau = w;
//   const std::vector<int>& lpf_torque_ids = modelSoft->get_lpf_torque_ids();
//   const std::vector<int>& non_lpf_torque_ids =
//       modelSoft->get_non_lpf_torque_ids();
//   BOOST_CHECK(ntau == lpf_torque_ids.size());
//   for (std::size_t i = 0; i < lpf_torque_ids.size(); i++) {
//     tau(lpf_torque_ids[i]) = y.tail(ntau)[i];
//   }
//   BOOST_CHECK(non_lpf_torque_ids.size() + lpf_torque_ids.size() == nu);

//   // Checking the partial derivatives against NumDiff
//   double tol = 1e-6;

//   // Computing the action
//   modelSoft->calc(dataSoft, y, w);
//   modelEuler->calc(dataEuler, x, tau);
//   // Computing the derivatives
//   modelSoft->calcDiff(dataSoft, y, w);
//   modelEuler->calcDiff(dataEuler, x, tau);

//   // All or some joint are LPF
//   // Size varying stuff
//   const Eigen::MatrixXd& Fu_LPF = dataSoft->Fx.topRightCorner(ndx, ntau);
//   const Eigen::MatrixXd& Lu_LPF = dataSoft->Lx.tail(ntau);
//   const Eigen::MatrixXd& Lxu_LPF = dataSoft->Lxx.topRightCorner(ndx, ntau);
//   const Eigen::MatrixXd& Luu_LPF = dataSoft->Lxx.bottomRightCorner(ntau, ntau);
//   for (std::size_t i = 0; i < lpf_torque_ids.size(); i++) {
//     BOOST_CHECK(
//         (Fu_LPF.col(i) - dataEuler->Fu.col(lpf_torque_ids[i])).isZero(tol));
//     BOOST_CHECK((Lu_LPF(i) - dataEuler->Lu(lpf_torque_ids[i])) <= tol);
//     BOOST_CHECK(
//         (Lxu_LPF.col(i) - dataEuler->Lxu.col(lpf_torque_ids[i])).isZero(tol));
//     for (std::size_t j = 0; j < lpf_torque_ids.size(); j++) {
//       BOOST_CHECK((Luu_LPF(i, j) - dataEuler->Luu(lpf_torque_ids[i],
//                                                   lpf_torque_ids[j])) <= tol);
//     }
//   }
//   // Fixed size stuff
//   const Eigen::MatrixXd& Fx_LPF = dataSoft->Fx.topLeftCorner(ndx, ndx);
//   const Eigen::MatrixXd& Lx_LPF = dataSoft->Lx.head(ndx);
//   const Eigen::MatrixXd& Lxx_LPF = dataSoft->Lxx.topLeftCorner(ndx, ndx);
//   // Testing the partials w.r.t. u match blocks in partial w.r.t. augmented
//   // state y
//   BOOST_CHECK((Fx_LPF - dataEuler->Fx).isZero(tol));
//   BOOST_CHECK((Lx_LPF - dataEuler->Lx).isZero(tol));
//   BOOST_CHECK((Lxx_LPF - dataEuler->Lxx).isZero(tol));

//   // Non LPF dimensions
//   // Size varying stuff
//   const Eigen::MatrixXd& Fu_nonLPF = dataSoft->Fu.topRows(ndx);
//   const Eigen::MatrixXd& Lu_nonLPF = dataSoft->Lu;
//   const Eigen::MatrixXd& Lxu_nonLPF = dataSoft->Lxu;
//   const Eigen::MatrixXd& Luu_nonLPF = dataSoft->Luu;
//   for (std::size_t i = 0; i < non_lpf_torque_ids.size(); i++) {
//     BOOST_CHECK((Fu_nonLPF.col(non_lpf_torque_ids[i]) -
//                  dataEuler->Fu.col(non_lpf_torque_ids[i]))
//                     .isZero(tol));
//     BOOST_CHECK((Lu_nonLPF(non_lpf_torque_ids[i]) -
//                  dataEuler->Lu(non_lpf_torque_ids[i])) <= tol);
//     BOOST_CHECK((Lxu_nonLPF.topRows(ndx).col(non_lpf_torque_ids[i]) -
//                  dataEuler->Lxu.col(non_lpf_torque_ids[i]))
//                     .isZero(tol));
//     for (std::size_t j = 0; j < non_lpf_torque_ids.size(); j++) {
//       BOOST_CHECK((Luu_nonLPF(non_lpf_torque_ids[i], non_lpf_torque_ids[j]) -
//                    dataEuler->Luu(non_lpf_torque_ids[i],
//                                   non_lpf_torque_ids[j])) <= tol);
//     }
//     for (std::size_t j = 0; j < lpf_torque_ids.size(); j++) {
//       BOOST_CHECK((Lxu_nonLPF.bottomRows(ntau)(j, non_lpf_torque_ids[i]) -
//                    dataEuler->Luu(lpf_torque_ids[j], non_lpf_torque_ids[i])) <=
//                   tol);
//     }
//   }

//   // Test terminal calcDiff
//   // Computing the action
//   modelSoft->set_dt(0.);
//   modelEuler->set_dt(0.);
//   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataSoftTerminal =
//       modelSoft->createData();
//   const boost::shared_ptr<crocoddyl::ActionDataAbstract>& dataEulerTerminal =
//       modelEuler->createData();
//   modelSoft->calc(dataSoftTerminal, y);
//   modelEuler->calc(dataEulerTerminal, x);
//   modelSoft->calcDiff(dataSoftTerminal, y);
//   modelEuler->calcDiff(dataEulerTerminal, x);
//   // All or some joint are LPF
//   // Size varying stuff
//   const Eigen::MatrixXd& Fu_LPF_term =
//       dataSoftTerminal->Fx.topRightCorner(ndx, ntau);
//   const Eigen::MatrixXd& Lu_LPF_term = dataSoftTerminal->Lx.tail(ntau);
//   const Eigen::MatrixXd& Lxu_LPF_term =
//       dataSoftTerminal->Lxx.topRightCorner(ndx, ntau);
//   const Eigen::MatrixXd& Luu_LPF_term =
//       dataSoftTerminal->Lxx.bottomRightCorner(ntau, ntau);
//   for (std::size_t i = 0; i < lpf_torque_ids.size(); i++) {
//     BOOST_CHECK(
//         (Fu_LPF_term.col(i) - dataEulerTerminal->Fu.col(lpf_torque_ids[i]))
//             .isZero(tol));
//     BOOST_CHECK((Lu_LPF_term(i) - dataEulerTerminal->Lu(lpf_torque_ids[i])) <=
//                 tol);
//     BOOST_CHECK(
//         (Lxu_LPF_term.col(i) - dataEulerTerminal->Lxu.col(lpf_torque_ids[i]))
//             .isZero(tol));
//     for (std::size_t j = 0; j < lpf_torque_ids.size(); j++) {
//       BOOST_CHECK((Luu_LPF_term(i, j) -
//                    dataEulerTerminal->Luu(lpf_torque_ids[i],
//                                           lpf_torque_ids[j])) <= tol);
//     }
//   }
//   // Fixed size stuff
//   const Eigen::MatrixXd& Fx_LPF_term =
//       dataSoftTerminal->Fx.topLeftCorner(ndx, ndx);
//   const Eigen::MatrixXd& Lx_LPF_term = dataSoftTerminal->Lx.head(ndx);
//   const Eigen::MatrixXd& Lxx_LPF_term =
//       dataSoftTerminal->Lxx.topLeftCorner(ndx, ndx);
//   // Testing the partials w.r.t. u match blocks in partial w.r.t. augmented
//   // state y if(!(Fx_LPF_term - dataEulerTerminal->Fx).isZero(tol)){
//   //   std::cout << " Fx_lpf - Fx_euler terminal = " << std::endl;
//   //   std::cout << (Fx_LPF_term - dataEulerTerminal->Fx) << std::endl;
//   // }
//   BOOST_CHECK((Fx_LPF_term - dataEulerTerminal->Fx).isZero(tol));
//   BOOST_CHECK((Lx_LPF_term - dataEulerTerminal->Lx).isZero(tol));
//   BOOST_CHECK((Lxx_LPF_term - dataEulerTerminal->Lxx).isZero(tol));

//   // Non LPF dimensions
//   // Size varying stuff
//   const Eigen::MatrixXd& Fu_nonLPF_term = dataSoftTerminal->Fu.topRows(ndx);
//   const Eigen::MatrixXd& Lu_nonLPF_term = dataSoftTerminal->Lu;
//   const Eigen::MatrixXd& Lxu_nonLPF_term = dataSoftTerminal->Lxu;
//   const Eigen::MatrixXd& Luu_nonLPF_term = dataSoftTerminal->Luu;
//   for (std::size_t i = 0; i < non_lpf_torque_ids.size(); i++) {
//     BOOST_CHECK((Fu_nonLPF_term.col(non_lpf_torque_ids[i]) -
//                  dataEulerTerminal->Fu.col(non_lpf_torque_ids[i]))
//                     .isZero(tol));
//     BOOST_CHECK((Lu_nonLPF_term(non_lpf_torque_ids[i]) -
//                  dataEulerTerminal->Lu(non_lpf_torque_ids[i])) <= tol);
//     BOOST_CHECK((Lxu_nonLPF_term.topRows(ndx).col(non_lpf_torque_ids[i]) -
//                  dataEulerTerminal->Lxu.col(non_lpf_torque_ids[i]))
//                     .isZero(tol));
//     for (std::size_t j = 0; j < non_lpf_torque_ids.size(); j++) {
//       BOOST_CHECK(
//           (Luu_nonLPF_term(non_lpf_torque_ids[i], non_lpf_torque_ids[j]) -
//            dataEulerTerminal->Luu(non_lpf_torque_ids[i],
//                                   non_lpf_torque_ids[j])) <= tol);
//     }
//     for (std::size_t j = 0; j < lpf_torque_ids.size(); j++) {
//       BOOST_CHECK((Lxu_nonLPF_term.bottomRows(ntau)(j, non_lpf_torque_ids[i]) -
//                    dataEulerTerminal->Luu(lpf_torque_ids[j],
//                                           non_lpf_torque_ids[i])) <= tol);
//     }
//   }
// }

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
  // Seems incompatible with euler equivalence test
  // ts->add(BOOST_TEST_CASE(boost::bind(&test_partial_derivatives_action_model_terminal, iam_type, dam_type, ref_type)));
  // Equivalence with Euler when Kp, Kv=0
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_equivalent_euler, iam_type, dam_type, ref_type)));
  // ts->add(BOOST_TEST_CASE(boost::bind(&test_calcDiff_equivalent_euler, iam_type, dam_type, ref_type)));
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

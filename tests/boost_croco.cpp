#include <boost/test/unit_test.hpp>

#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "example-adder/croco.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_boost_add_se3) {
  using namespace gepetto::example;

  const boost::shared_ptr<crocoddyl::ActionModelAbstract>& model = boost::make_shared<ActionModelUniEx>();
  const std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> > running_models(20, model);
  const boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      boost::make_shared<crocoddyl::ShootingProblem>(model->get_state()->rand(), running_models, model);
  const boost::shared_ptr<crocoddyl::SolverAbstract> solver = boost::make_shared<crocoddyl::SolverDDP>(problem);

  BOOST_CHECK(solver->get_xs().back()[0] == 0);
  BOOST_CHECK(solver->get_xs().back()[1] == 0);
  BOOST_CHECK(solver->get_xs().back()[2] == 0);

  solver->solve();

  BOOST_CHECK(solver->get_xs().back()[0] != 0);
  BOOST_CHECK(solver->get_xs().back()[1] != 0);
  BOOST_CHECK(solver->get_xs().back()[2] != 0);
  BOOST_CHECK(fabs(solver->get_xs().back()[0]) < 1e-7);
  BOOST_CHECK(fabs(solver->get_xs().back()[1]) < 1e-1);
  BOOST_CHECK(fabs(solver->get_xs().back()[2]) < 1e-7);
}

BOOST_AUTO_TEST_SUITE_END()

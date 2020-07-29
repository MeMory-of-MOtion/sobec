#include <boost/test/unit_test.hpp>

#include "example-adder/pin.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_boost_add_se3) {
  typedef pinocchio::SE3 SE3;
  SE3 a = SE3::Random();
  SE3 b = SE3::Identity();

  BOOST_CHECK(gepetto::example::adds(a, a) != a);
  BOOST_CHECK(gepetto::example::adds(b, b) == b);
  BOOST_CHECK(gepetto::example::adds(a, b) == a);
  BOOST_CHECK(gepetto::example::adds(b, a) == a);
  BOOST_CHECK(gepetto::example::subs(a, b) == a.inverse());
}

BOOST_AUTO_TEST_SUITE_END()

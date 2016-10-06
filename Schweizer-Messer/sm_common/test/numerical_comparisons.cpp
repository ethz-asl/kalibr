#include <gtest/gtest.h>
#include <sm/numerical_comparisons.hpp>

TEST(SmCommonTestSuite,testApproximatelyEqual)
{
  ASSERT_TRUE(sm::approximatelyEqual(1.0, 1.0, 1e-10));
  ASSERT_FALSE(sm::approximatelyEqual(-1.0, 1.0));
  ASSERT_TRUE(sm::approximatelyEqual(-1.0, -1.0));
  ASSERT_TRUE(sm::approximatelyEqual(1.0, 2.0/2.0));
  ASSERT_TRUE(sm::approximatelyEqual(1000.0, 999.0, 1e-3));
  ASSERT_FALSE(sm::approximatelyEqual(1000.0, 998.0, 1e-3));
  ASSERT_THROW(sm::approximatelyEqual(1.0, 1.0, 0.0), std::invalid_argument);
  ASSERT_THROW(sm::approximatelyEqual(1.0, 1.0, -0.1), std::invalid_argument);
}

TEST(SmCommonTestSuite,testDefinitelyGreaterThan)
{
  ASSERT_TRUE(sm::definitelyGreaterThan(2.0, 1.0, 1e-10));
  ASSERT_FALSE(sm::definitelyGreaterThan(1.0, 1.0));
  ASSERT_FALSE(sm::definitelyGreaterThan(-2.0, 1.0));
  ASSERT_TRUE(sm::definitelyGreaterThan(-0.5, -1.0));
  ASSERT_FALSE(sm::definitelyGreaterThan(1000.0, 999.0, 1e-3));
  ASSERT_TRUE(sm::definitelyGreaterThan(1000.0, 998.0, 1e-3));
  ASSERT_THROW(sm::definitelyGreaterThan(2.0, 1.0, 0.0), std::invalid_argument);
  ASSERT_THROW(sm::definitelyGreaterThan(2.0, 1.0, -0.1), std::invalid_argument);
}

TEST(SmCommonTestSuite,testDefinitelyLessThan)
{
  ASSERT_TRUE(sm::definitelyLessThan(1.0, 2.0, 1e-10));
  ASSERT_FALSE(sm::definitelyLessThan(1.0, 1.0));
  ASSERT_FALSE(sm::definitelyLessThan(1.0, -2.0));
  ASSERT_TRUE(sm::definitelyLessThan(-1.0, -0.5));
  ASSERT_FALSE(sm::definitelyLessThan(999.0, 1000.0, 1e-3));
  ASSERT_TRUE(sm::definitelyLessThan(998.0, 1000.0, 1e-3));
  ASSERT_THROW(sm::definitelyLessThan(1.0, 2.0, 0.0), std::invalid_argument);
  ASSERT_THROW(sm::definitelyLessThan(1.0, 2.0, -0.1), std::invalid_argument);
}

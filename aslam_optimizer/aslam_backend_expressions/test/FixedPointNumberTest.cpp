#include <gtest/gtest.h>
#include <sm/random.hpp>
#include <aslam/backend/FixedPointNumber.hpp>

using namespace aslam::backend;

template <typename FixedPointNumber_>
struct FixedPointNumberTestSuites : public ::testing::Test  {
  typedef typename FixedPointNumber_::Integer Integer;
  typedef FixedPointNumber_ TestFixedPointNumber;

  constexpr static double eps = std::numeric_limits<TestFixedPointNumber>::epsilon();

  TestFixedPointNumber getRand() {
    return TestFixedPointNumber((sm::random::rand() - 0.5) * 10);
  }
};


typedef ::testing::Types<
    FixedPointNumber<char, 10>,
    FixedPointNumber<short, 1000>,
    FixedPointNumber<int, 1000>,
    FixedPointNumber<long, 1000000>,
    FixedPointNumber<long long, long(1E9)>
> PrimTypes;

TYPED_TEST_CASE(FixedPointNumberTestSuites, PrimTypes);

TYPED_TEST(FixedPointNumberTestSuites, testConstAdd)
{
  TypeParam a = this->getRand();
  ASSERT_DOUBLE_EQ((double) (a + TypeParam(0.0)), (double) a);
}

TYPED_TEST(FixedPointNumberTestSuites, testNegation)
{
  TypeParam a = this->getRand();
  ASSERT_DOUBLE_EQ((double) (-a), -(double)a);
}

TYPED_TEST(FixedPointNumberTestSuites, testPlus)
{
  TypeParam a = this->getRand(), b = this->getRand();
  ASSERT_NEAR((double)(a + b), (double)a + (double)b, this->eps);
}

TYPED_TEST(FixedPointNumberTestSuites, testMinus)
{
  TypeParam a = this->getRand(), b = this->getRand();
  ASSERT_NEAR((double)(a - b), (double)a - (double)b, this->eps);
}

TYPED_TEST(FixedPointNumberTestSuites, testTimes)
{
  TypeParam a = this->getRand(), b = this->getRand();
  ASSERT_NEAR((double)(a * b), (double)a * (double)b, this->eps);
}

TYPED_TEST(FixedPointNumberTestSuites, testDivision)
{
  TypeParam a = this->getRand(), b = this->getRand();
  ASSERT_NEAR((double)(a / b), (double)a / (double)b, this->eps);
}

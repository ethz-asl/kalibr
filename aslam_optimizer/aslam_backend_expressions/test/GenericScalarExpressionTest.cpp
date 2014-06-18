#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <aslam/backend/GenericScalarExpression.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <aslam/backend/GenericScalarExpression.hpp>
#include <aslam/backend/GenericScalar.hpp>
#include <sm/random.hpp>
#include <aslam/backend/test/GenericScalarExpressionTests.hpp>
#include <aslam/backend/test/DesignVariableTests.hpp>
#include <aslam/backend/FixedPointNumber.hpp>

using namespace aslam::backend;

template <typename GenericScalar_>
struct GenericScalarExpressionNodeTestSuites : public ::testing::Test  {
  typedef Eigen::Matrix<double,1,1> Vector1d;

  typedef typename GenericScalar_::Scalar PrimScalar;
  typedef GenericScalarExpression<PrimScalar> TestGenericScalarExpression;
  typedef GenericScalar_ TestGenericScalar;

  static PrimScalar getRandScalar() {
    return PrimScalar(sm::random::rand() * 10.0);
  }
};

typedef ::testing::Types<
    GenericScalar<FixedPointNumber<int, long(1E6)>>,
    GenericScalar<FixedPointNumber<long long, long(1E9)>>,
    GenericScalar<double>
> Types;

TYPED_TEST_CASE(GenericScalarExpressionNodeTestSuites, Types);

TYPED_TEST(GenericScalarExpressionNodeTestSuites, testMinimalDifference)
{
  try {
    for(int i = 0; i < 10; i ++){
      typename TestFixture::TestGenericScalar point1(this->getRandScalar());
      SCOPED_TRACE("");
      test::testDesignVariable(point1);
    }
  }
  catch(std::exception const & e)
  {
    FAIL() << e.what();
  }

}

// Test that the jacobian matches the finite difference jacobian
TYPED_TEST(GenericScalarExpressionNodeTestSuites, testScalarProduct)
{
    try
    {
        SCOPED_TRACE("");
        testExpression(
            typename TestFixture::TestGenericScalar(this->getRandScalar()).toExpression()
            *
            typename TestFixture::TestGenericScalar(this->getRandScalar()).toExpression(),
            2, false, test::ExpressionTraits<typename TestFixture::TestGenericScalarExpression>::defaultTolerance(), 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TYPED_TEST(GenericScalarExpressionNodeTestSuites, testScalarProductWithConst)
{
    try
    {
        typename TestFixture::TestGenericScalar point1(this->getRandScalar());
        auto p2 = this->getRandScalar();

        SCOPED_TRACE("");
        testExpression(point1.toExpression() * p2, 1, false, test::ExpressionTraits<typename TestFixture::TestGenericScalarExpression>::defaultTolerance(), 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TYPED_TEST(GenericScalarExpressionNodeTestSuites, testScalarAddition)
{
    try
    {
        typename TestFixture::TestGenericScalar point1(this->getRandScalar());
        typename TestFixture::TestGenericScalarExpression p1 = point1.toExpression();

        typename TestFixture::TestGenericScalar point2(this->getRandScalar());
        typename TestFixture::TestGenericScalarExpression p2 = point2.toExpression();

        SCOPED_TRACE("");
        testExpression(p1 + p2, 2);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TYPED_TEST(GenericScalarExpressionNodeTestSuites, testScalarSubtraction)
{
    try
    {
        typename TestFixture::TestGenericScalar point1(this->getRandScalar());
        typename TestFixture::TestGenericScalarExpression p1 = point1.toExpression();

        typename TestFixture::TestGenericScalar point2(this->getRandScalar());
        typename TestFixture::TestGenericScalarExpression p2 = point2.toExpression();

        SCOPED_TRACE("");
        testExpression(p1 - p2, 2);

    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}


// Test that the jacobian matches the finite difference jacobian
TYPED_TEST(GenericScalarExpressionNodeTestSuites, testScalarNegation)
{
    try
    {
        SCOPED_TRACE("");
        testExpression(-typename TestFixture::TestGenericScalar(this->getRandScalar()).toExpression(), 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TYPED_TEST(GenericScalarExpressionNodeTestSuites, testConstSubtraction)
{
    try
    {
        typename TestFixture::TestGenericScalar point1(this->getRandScalar());
        typename TestFixture::TestGenericScalarExpression p1 = point1.toExpression();

        auto p2 = this->getRandScalar();

        typename TestFixture::TestGenericScalarExpression p_diff = p1 - p2;

        SCOPED_TRACE("");
        testExpression(p_diff, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}


template <typename Scalar_>
GenericScalar<Scalar_> getRandScalar() {
  return GenericScalarExpressionNodeTestSuites<GenericScalar<Scalar_> >::getRandScalar();
}

// Test that the jacobian matches the finite difference jacobian
TYPED_TEST(GenericScalarExpressionNodeTestSuites, testCast)
{
    try
    {
        SCOPED_TRACE("");
        testExpression(typename TestFixture::TestGenericScalarExpression(getRandScalar<double>().toExpression()), 1);
        SCOPED_TRACE("");
        testExpression(typename TestFixture::TestGenericScalarExpression(getRandScalar<FixedPointNumber<long long, long(1E9)> >().toExpression()), 1, false, test::ExpressionTraits<GenericScalarExpression<long long> >::defaultTolerance(), test::ExpressionTraits<GenericScalarExpression<long long> >::defaulEps());
        SCOPED_TRACE("");
        testExpression(typename TestFixture::TestGenericScalarExpression(getRandScalar<FixedPointNumber<int, long(1E6)>>().toExpression()), 1, false, test::ExpressionTraits<GenericScalarExpression<int> >::defaultTolerance(), test::ExpressionTraits<GenericScalarExpression<int> >::defaulEps());
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TYPED_TEST(GenericScalarExpressionNodeTestSuites, testConstAddition)
{
    try
    {
        typename TestFixture::TestGenericScalar point1(this->getRandScalar());
        typename TestFixture::TestGenericScalarExpression p1 = point1.toExpression();

        auto p2 = this->getRandScalar();

        SCOPED_TRACE("");
        testExpression(p1 + p2, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

TYPED_TEST(GenericScalarExpressionNodeTestSuites, testOpsFailure)
{
    if(std::numeric_limits<typename TestFixture::PrimScalar>::is_integer
        || is_fixed_point_number<typename TestFixture::PrimScalar>::value) return;

    using namespace aslam::backend;
    // In [12]: p
    // Out[12]: 628.233093262
    typename TestFixture::PrimScalar p(628.233093262);

    // In [13]: t
    // Out[13]: 1338888490.598351
    typename TestFixture::PrimScalar t(1338888490.598351);
    // In [14]: cC.eC.lineDelayDv.toScalar() * p + t
    // Out[14]: 1338888490.6249666
    typename TestFixture::PrimScalar ld(4.23659880956014e-05);

    typename TestFixture::TestGenericScalar ldDv(ld);
    // In [15]: lineDelayDvExpression = cC.eC.lineDelayDv.toExpression()
    typename TestFixture::TestGenericScalarExpression ldExp = ldDv.toExpression();
    ASSERT_EQ(ldDv.toScalar(), ld);

    // In [16]: lineDelayDvExpression.toScalar()
    // Out[16]: 4.23659880956014e-05  
    ASSERT_EQ(ld, ldExp.toScalar());

    // In [17]: keypointOffset = lineDelayDvExpression * p
    typename TestFixture::TestGenericScalarExpression keypointOffset = ldExp * p;

    // In [19]: keypointOffset.toScalar()
    // Out[19]: 0.02661571575038882
    ASSERT_EQ(ld * p, keypointOffset.toScalar());

    // In [20]: keypointTime = keypointOffset + t
    typename TestFixture::TestGenericScalarExpression keypointTime = keypointOffset + t;

    // In [21]: keypointTime.toScalar()
    // Out[21]: 1338888448.0266156
    ASSERT_EQ(ld * p + t, keypointTime.toScalar());

    // In [22]: cC.eC.lineDelayDv.toScalar() * p + t
    // Out[22]: 1338888490.6249666
    ASSERT_EQ(ldDv.toScalar() * p + t, keypointTime.toScalar());
}

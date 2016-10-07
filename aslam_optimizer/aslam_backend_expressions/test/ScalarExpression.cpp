#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <sm/random.hpp>

#include <aslam/backend/test/ExpressionTests.hpp>
#include <aslam/backend/test/GenericScalarExpressionTests.hpp>

using namespace aslam::backend;
using namespace sm::kinematics;
typedef Eigen::Matrix<double,1,1> Vector1d;

struct ScalarExpressionNodeFunctor
{
    typedef Vector1d value_t;
    typedef value_t::Scalar scalar_t;
    typedef Eigen::VectorXd input_t;
    typedef Eigen::MatrixXd jacobian_t;

  
    ScalarExpressionNodeFunctor(ScalarExpression dv) :
        _dv(dv) {}

    input_t update(const input_t & x, int c, scalar_t delta) { input_t xnew = x; xnew[c] += delta; return xnew; }

    ScalarExpression _dv;

    Eigen::VectorXd operator()(const Eigen::VectorXd & dr)
        {
    
            Vector1d p;
            p(0,0) = _dv.toScalar();
            JacobianContainerSparse<1> J(1);
            _dv.evaluateJacobians(J);

            int offset = 0;
            for(size_t i = 0; i < J.numDesignVariables(); i++)
            {
                DesignVariable * d = J.designVariable(i);
                d->update(&dr[offset],d->minimalDimensions());
                offset += d->minimalDimensions();
            }

            p(0,0) = _dv.toScalar();
 
            for(size_t i = 0; i < J.numDesignVariables(); i++)
            {
                DesignVariable * d = J.designVariable(i);
                d->revertUpdate();
            }

            //std::cout << "returning " << p << std::endl;
            return p;
   
        }
};


TEST(ScalarExpressionNodeTestSuites,testMinimalDifferenceJacobian)
{
  try {
    using namespace sm::kinematics;
    double initialValue = sm::random::rand();
    Scalar point1(initialValue);
    point1.setActive(true);
    point1.setBlockIndex(1);

    double u = sm::random::rand();

    point1.update(&u, 1);
    // now x is equal to x_bar

    Eigen::MatrixXd xHat = Eigen::MatrixXd(1,1);
    xHat(0,0) = initialValue;
    Eigen::VectorXd diffV1;
    point1.minimalDifference(xHat, diffV1);

    // now update again with eps
    double eps = sm::random::rand();
    point1.update(&eps, 1);
    Eigen::VectorXd diffV2;
    Eigen::MatrixXd J;
    point1.minimalDifferenceAndJacobian(xHat, diffV2, J);

    // delta e is now diffV1 - diffV1
    Eigen::VectorXd delta_e = diffV2 - diffV1;
    Eigen::VectorXd delta_eps = Eigen::VectorXd(1);
    delta_eps(0) = eps;

    Eigen::VectorXd v1 = J*delta_eps;

//    std::cout << "initialValue is: " << initialValue << std::endl;
//    std::cout << "xHat is: " << xHat << std::endl;
//    std::cout << "diffV1 is: " << diffV1 << std::endl;
//    std::cout << "diffV2 is: " << diffV2 << std::endl;
//    std::cout << "J is: " << J << std::endl;
//    std::cout << "u is: " << u << std::endl;
//    std::cout << "eps is: " << eps << std::endl;
//    std::cout << "delta_e is: " << delta_e << std::endl;
//    std::cout << "v1 is: " << v1 << std::endl;


    sm::eigen::assertNear(v1, delta_e, 1e-6, SM_SOURCE_FILE_POS, "Testing the minimal difference jacobian");
  }
  catch(std::exception const & e)
  {
    FAIL() << e.what();
  }

}



// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testScalarProduct)
{
    try
    {
        using namespace sm::kinematics;
        Scalar point1(sm::random::rand());
        ScalarExpression p1 = point1.toExpression();

        Scalar point2(sm::random::rand());
        ScalarExpression p2 = point2.toExpression();

        ScalarExpression p_cross = p1 * p2;

        SCOPED_TRACE("");
        testExpression(p_cross, 2);

    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testScalarProduct2)
{
    try
    {
        using namespace sm::kinematics;
        Scalar point1(sm::random::rand());
        ScalarExpression p1 = point1.toExpression();

        double p2 = sm::random::rand();

        ScalarExpression p_cross = p1 * p2;

        SCOPED_TRACE("");
        testExpression(p_cross, 1);

    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}



// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testScalarNegation)
{
    try
    {
        using namespace sm::kinematics;
        Scalar point1(sm::random::rand());
        ScalarExpression p1 = point1.toExpression();

        ScalarExpression p_neg = -p1;

        ASSERT_EQ(-point1.toScalar(), p_neg.toValue());

        SCOPED_TRACE("");
        testExpression(p_neg, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}


// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testScalarAddition)
{
    try
    {
        using namespace sm::kinematics;
        Scalar point1(sm::random::rand());
        ScalarExpression p1 = point1.toExpression();

        Scalar point2(sm::random::rand());
        ScalarExpression p2 = point2.toExpression();

        ScalarExpression p_add = p1 + p2;

        SCOPED_TRACE("");
        testExpression(p_add, 2);

    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testScalarSubtraction)
{
    try
    {
        using namespace sm::kinematics;
        Scalar point1(sm::random::rand());
        ScalarExpression p1 = point1.toExpression();

        Scalar point2(sm::random::rand());
        ScalarExpression p2 = point2.toExpression();

        ScalarExpression p_diff = p1 - p2;

        SCOPED_TRACE("");
        testExpression(p_diff, 2);

    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testVectorSubtraction)
{
    try
    {
        using namespace sm::kinematics;
        Scalar point1(sm::random::rand());
        ScalarExpression p1 = point1.toExpression();

        double p2 = sm::random::rand();

        ScalarExpression p_diff = p1 - p2;

        SCOPED_TRACE("");
        testExpression(p_diff, 1);

    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testVectorAddition)
{
    try
    {
        using namespace sm::kinematics;
        Scalar point1(sm::random::rand());
        ScalarExpression p1 = point1.toExpression();

        double p2 = sm::random::rand();

        ScalarExpression p_diff = p1 + p2;

        SCOPED_TRACE("");
        testExpression(p_diff, 1);

    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testSqrt)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p(sm::random::rand());
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprSqrt = sqrt(pExpr);

        ASSERT_EQ(sqrt(p.toScalar()), pExprSqrt.toValue());

        SCOPED_TRACE("");
        testExpression(pExprSqrt, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testLog)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p(sm::random::rand());
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprLog = log(pExpr);

        ASSERT_EQ(log(p.toScalar()), pExprLog.toValue());

        SCOPED_TRACE("");
        testExpression(pExprLog, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testExp)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p(sm::random::rand());
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprExp = exp(pExpr);

        ASSERT_EQ(exp(p.toScalar()), pExprExp.toValue());

        SCOPED_TRACE("");
        testExpression(pExprExp, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testAtan)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p(sm::random::rand());
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprAtan = atan(pExpr);

        ASSERT_EQ(atan(p.toScalar()), pExprAtan.toValue());

        SCOPED_TRACE("");
        testExpression(pExprAtan, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testTanh)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p(1e6*sm::random::rand());
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprTanh = tanh(pExpr);

        ASSERT_EQ(tanh(p.toScalar()), pExprTanh.toValue());

        SCOPED_TRACE("");
        testExpression(pExprTanh, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testAcos)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p(sm::random::rand());
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprAcos = acos(pExpr);

        ASSERT_EQ(acos(p.toScalar()), pExprAcos.toValue());

        SCOPED_TRACE("");
        testExpression(pExprAcos, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testAtan2)
{
    try
    {
        using namespace sm::kinematics;
        Scalar x(sm::random::rand());
        Scalar y(sm::random::rand());
        ScalarExpression pExprAtan2 = atan2(y.toExpression(), x.toExpression());

        ASSERT_EQ(atan2(y.toScalar(), x.toScalar()), pExprAtan2.toValue());

        SCOPED_TRACE("");
        testExpression(pExprAtan2, 2);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testAcosSquared)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p(sm::random::rand());
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprAcosSquared = acosSquared(pExpr);

        ASSERT_EQ(acos(p.toScalar())*acos(p.toScalar()), pExprAcosSquared.toValue());

        SCOPED_TRACE("");
        testExpression(pExprAcosSquared, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}


// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testAcosSquaredPole)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p(1.0);
        p.setActive(true);
        p.setBlockIndex(0);
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprAcosSquared = acosSquared(pExpr);

        ASSERT_EQ(acos(p.toScalar())*acos(p.toScalar()), pExprAcosSquared.toValue());

        SCOPED_TRACE("");
        const size_t rows = 1;
        JacobianContainerSparse<rows> Jc(rows);
        pExprAcosSquared.evaluateJacobians(Jc);
        ASSERT_DOUBLE_EQ(-2.0, Jc.asDenseMatrix()(0,0));
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testAcosSquaredLimits)
{
    try
    {
        using namespace sm::kinematics;
        double eps = sqrt(std::numeric_limits<double>::epsilon());
        Scalar p(1.0 - eps);
        p.setActive(true);
        p.setBlockIndex(0);
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprAcosSquared = acosSquared(pExpr);

        ASSERT_EQ(acos(p.toScalar())*acos(p.toScalar()), pExprAcosSquared.toValue());

        SCOPED_TRACE("");
        const size_t rows = 1;
        JacobianContainerSparse<rows> Jc(rows);
        pExprAcosSquared.evaluateJacobians(Jc);

        double pow1 =  p.toScalar() - 1.0;
        double pow2 = pow1 * pow1;
        double pow3 = pow1 * pow2;

        ASSERT_DOUBLE_EQ(-2.0 + 2.0/3.0*pow1 - 4.0/15.0*pow2 + 4.0/35.0*pow3, Jc.asDenseMatrix()(0,0));
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

TEST(ScalarExpressionNodeTestSuites, testSigmoid)
{
    try
    {
        for (std::size_t i=0; i<1000; ++i) {
          const double factor = i < 500 ? 1000. : 1.; // Test some large values for the 1st 500 iterations
          Scalar p(factor * sm::random::randn());
          ScalarExpression pExpr = p.toExpression();
          const double height = factor * sm::random::rand();
          const double scale = factor * sm::random::rand();
          const double shift = sm::random::rand();
          ScalarExpression pExprSigmoid = inverseSigmoid(pExpr, height, scale, shift);
          const double p1 = height * 0.5 * (1. + std::tanh( -scale*(p.toScalar() - shift) * 0.5));

          ASSERT_EQ(p1, pExprSigmoid.toScalar());

          SCOPED_TRACE(testing::Message() << "Testing sigmoid at x = " << p.toScalar());
          testExpression(pExprSigmoid, 1, false);
        }
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testPower)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p(sm::random::rand());
        int k = int(sm::random::rand());
        ScalarExpression pExpr = p.toExpression();
        ScalarExpression pExprPow = powerExpression(pExpr,k);

        ASSERT_EQ(pow(p.toScalar(), k), pExprPow.toValue());

        SCOPED_TRACE("");
        testExpression(pExprPow, 1);
    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}


// Test that the jacobian matches the finite difference jacobian
TEST(ScalarExpressionNodeTestSuites, testPiecewiseExpression)
{
    try
    {
        using namespace sm::kinematics;
        Scalar p1(10);
        Scalar p2(20);
        ScalarExpression p1Expr = p1.toExpression();
        ScalarExpression p2Expr = p2.toExpression();
        bool condition = true;
        ScalarExpression jointExpr = piecewiseExpression(p1Expr, p2Expr, [&condition](){return condition;});
        EXPECT_EQ(p1.toScalar(), jointExpr.toValue());
        SCOPED_TRACE("");
        testExpression(jointExpr, 1);

        condition = false;
        EXPECT_EQ(p2.toScalar(), jointExpr.toValue());
        testExpression(jointExpr, 1);

    }
    catch(std::exception const & e)
    {
        FAIL() << e.what();
    }
}

TEST(ScalarExpressionNodeTestSuites, testVectorOpsFailure)
{
    using namespace aslam::backend;
    // In [12]: p
    // Out[12]: 628.233093262
    double p = 628.233093262;

    // In [13]: t
    // Out[13]: 1338888490.598351
    double t = 1338888490.598351;
    // In [14]: cC.eC.lineDelayDv.toScalar() * p + t
    // Out[14]: 1338888490.6249666
    double ld = 4.23659880956014e-05;

    Scalar ldDv(ld);
    // In [15]: lineDelayDvExpression = cC.eC.lineDelayDv.toExpression()
    ScalarExpression ldExp = ldDv.toExpression();
    ASSERT_EQ(ldDv.toScalar(), ld);

    // In [16]: lineDelayDvExpression.toScalar()
    // Out[16]: 4.23659880956014e-05  
    ASSERT_EQ(ld, ldExp.toScalar());

    // In [17]: keypointOffset = lineDelayDvExpression * p
    ScalarExpression keypointOffset = ldExp * p;

    // In [19]: keypointOffset.toScalar()
    // Out[19]: 0.02661571575038882
    ASSERT_EQ(ld * p, keypointOffset.toScalar());

    // In [20]: keypointTime = keypointOffset + t
    ScalarExpression keypointTime = keypointOffset + t;

    // In [21]: keypointTime.toScalar()
    // Out[21]: 1338888448.0266156
    ASSERT_EQ(ld * p + t, keypointTime.toScalar());

    // In [22]: cC.eC.lineDelayDv.toScalar() * p + t
    // Out[22]: 1338888490.6249666
    ASSERT_EQ(ldDv.toScalar() * p + t, keypointTime.toScalar());


}

TEST(ScalarExpressionNodeTestSuites, testMinimalDifference)
{
  try {
    using namespace sm::kinematics;
    double initialValue = sm::random::rand();
    Scalar point1(initialValue);
    point1.setActive(true);
    point1.setBlockIndex(1);

    double eps = sm::random::rand();

    point1.update(&eps, 1);

    Eigen::MatrixXd xHat = Eigen::MatrixXd(1,1);
    xHat(0,0) = initialValue;
    Eigen::VectorXd diffV;
    point1.minimalDifference(xHat, diffV);

    double diff = diffV(0,0);

    EXPECT_DOUBLE_EQ(eps, diff);

  }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

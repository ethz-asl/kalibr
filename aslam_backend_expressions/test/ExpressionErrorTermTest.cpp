#include <gtest/gtest.h>
#include <aslam/backend/ExpressionErrorTerm.hpp>
#include <aslam/backend/test/ErrorTermTester.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/Scalar.hpp>

TEST(ExpressionErrorTermSuite, testScalarToErrorTerm) {
  try {
    using namespace aslam::backend;

    double expectedValue = 1;
    Scalar s(expectedValue);

    ScalarExpression se(&s);
    EXPECT_EQ(se.toValue(), expectedValue);

    expectedValue = 5;
    s.setParameters(Eigen::MatrixXd::Ones(1,1) * expectedValue);
    EXPECT_EQ(se.toValue(), expectedValue);

    se = se * 2;
    expectedValue *= 2;
    EXPECT_EQ(se.toValue(), expectedValue);

    {
      auto error = toErrorTerm(se);
      EXPECT_EQ(error->evaluateError(), pow(expectedValue, 2));
      SCOPED_TRACE("");
      aslam::backend::testErrorTerm(error, 1e-5);
    }
    {
      auto sNSError = toScalarNonSquaredErrorTerm(se, 1);
      EXPECT_EQ(sNSError->evaluateError(), expectedValue);
      SCOPED_TRACE("");
      aslam::backend::testErrorTerm(sNSError, 1e-5);
    }
    {
      auto sNSError = toScalarNonSquaredErrorTerm(se, 2);
      EXPECT_EQ(sNSError->evaluateError(), expectedValue * 2);
      SCOPED_TRACE("");
      aslam::backend::testErrorTerm(sNSError, 1e-5);
    }
  } catch (const std::exception & e) {
    FAIL()<< e.what();
  }
}

/*
 * TestUtils.cpp
 *
 *  Created on: 24.09.2015
 *      Author: Ulrich Schwesinger
 */

#include <sm/eigen/gtest.hpp>

#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/JacobianContainerSparse.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/GenericMatrixExpression.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/DesignVariableGenericVector.hpp>
#include <aslam/backend/util/ExpressionUtils.hpp>

using namespace aslam::backend;
using namespace std;

TEST(GenericMatrixExpressionNodeTestSuites, testExpressionUtils) {

  try {

    bool isFinite = false;
    Scalar s(1.0);
    s.setBlockIndex(0);
    ScalarExpression se = s.toExpression()*s.toExpression();
    JacobianContainerSparse<1> jc(1);

    // test isFinite
    {
      s.setActive(false);
      se.evaluateJacobians(jc);
      SCOPED_TRACE("");
      ASSERT_NO_THROW(isFinite = utils::isFinite(jc, se));
      EXPECT_TRUE(isFinite);
    }

    // test isFinite
    {
      s.setActive(true);
      se.evaluateJacobians(jc);
      SCOPED_TRACE("");
      ASSERT_NO_THROW(isFinite = utils::isFinite(jc, se));
      EXPECT_TRUE(isFinite);
    }

    {
      s.setParameters((Eigen::MatrixXd(1,1) << std::numeric_limits<double>::signaling_NaN()).finished());
      se.evaluateJacobians(jc);
      SCOPED_TRACE("");
      ASSERT_NO_THROW(isFinite = utils::isFinite(jc, se));
      EXPECT_FALSE(isFinite);
    }

  } catch(std::exception const & e)
  {
    FAIL() << e.what();
  }
}

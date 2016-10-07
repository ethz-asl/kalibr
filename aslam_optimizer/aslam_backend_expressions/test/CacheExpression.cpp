/*
 * CacheExpression.cpp
 *
 *  Created on: 08.03.2016
 *      Author: Ulrich Schwesinger
 */

#include <sm/eigen/gtest.hpp>
#include <sm/random.hpp>

#include <aslam/backend/JacobianContainerSparse.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/GenericMatrixExpression.hpp>
#include <aslam/backend/DesignVariableGenericVector.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/DesignVariableVector.hpp>
#include <aslam/backend/VectorExpressionToGenericMatrixTraits.hpp>
#include <aslam/backend/CacheExpression.hpp>

#include <aslam/backend/test/ExpressionTests.hpp>
#include <aslam/backend/test/GenericScalarExpressionTests.hpp>

using namespace aslam::backend;
typedef Eigen::Matrix<double,1,1> Vector1d;

template <typename Expression>
Eigen::MatrixXd evaluateJacobian(const Expression& expr) {
  JacobianContainerSparse<Expression::Dimension> jc(Expression::Dimension);
  expr.evaluateJacobians(jc);
  return jc.asDenseMatrix();
}

TEST(CacheExpressionTestSuites, testCachedExpression)
{
  try
  {
    // ********************** //
    // ScalarExpression cache //
    // ********************** //

    const double s0 = sm::random::rand();
    Scalar point(s0);
    point.setBlockIndex(0);
    point.setActive(true);
    ScalarExpression expr = point.toExpression();
    ScalarExpression cexpr = toCacheExpression(expr);
    ScalarExpression expr2 = expr*expr;
    ScalarExpression cexpr2 = cexpr*cexpr;

    // Test error evaluation
    EXPECT_DOUBLE_EQ(expr2.evaluate(), cexpr2.evaluate());
    sm::eigen::assertEqual(evaluateJacobian(expr2), evaluateJacobian(cexpr2), SM_SOURCE_FILE_POS);

    const double s1 = sm::random::rand();
    point.update(&s1, 1); // make sure update() method resets cache
    EXPECT_EQ(expr2.evaluate(), cexpr2.evaluate());
    sm::eigen::assertEqual(evaluateJacobian(expr2), evaluateJacobian(cexpr2), SM_SOURCE_FILE_POS);

    point.revertUpdate();
    EXPECT_EQ(expr2.evaluate(), cexpr2.evaluate());// make sure revertUpdate() method resets cache
    sm::eigen::assertEqual(evaluateJacobian(expr2), evaluateJacobian(cexpr2), SM_SOURCE_FILE_POS);

    point.setParameters((Vector1d(1,1) << sm::random::rand()).finished());
    EXPECT_EQ(expr2.evaluate(), cexpr2.evaluate());// make sure setParameter() method resets cache
    sm::eigen::assertEqual(evaluateJacobian(expr2), evaluateJacobian(cexpr2), SM_SOURCE_FILE_POS);

    // ***************************** //
    // GenericMatrixExpression cache //
    // ***************************** //
    const int VEC_ROWS = 2;
    typedef GenericMatrixExpression<VEC_ROWS, 1, double> GME;
    typedef DesignVariableGenericVector<VEC_ROWS> DGvec;
    GME::matrix_t vec = GME::matrix_t::Random();
    DGvec dv(vec);
    dv.setActive(true);
    dv.setBlockIndex(0);
    dv.setColumnBase(0);
    GME matExp(&dv);
    const auto cMatExp = toCacheExpression(matExp);
    const auto matExp2 = matExp.transpose()*matExp;
    const auto cMatExp2 = cMatExp.transpose()*cMatExp;

    // Test error evaluation
    sm::eigen::assertEqual(matExp.evaluate(), cMatExp.evaluate(), SM_SOURCE_FILE_POS);
    sm::eigen::assertEqual(matExp2.evaluate(), cMatExp2.evaluate(), SM_SOURCE_FILE_POS);
    sm::eigen::assertEqual(evaluateJacobian(expr2), evaluateJacobian(cexpr2), SM_SOURCE_FILE_POS);

    auto normal_dist = [] (int) { return sm::random::randn(); };
    const GME::matrix_t dx = GME::matrix_t::NullaryExpr(dv.minimalDimensions(), normal_dist);
    dv.update(dx.data(), dx.size()); // make sure update() method resets cache
    EXPECT_EQ(expr2.evaluate(), cexpr2.evaluate());
    sm::eigen::assertEqual(evaluateJacobian(expr2), evaluateJacobian(cexpr2), SM_SOURCE_FILE_POS);

    dv.revertUpdate();
    sm::eigen::assertEqual(matExp2.evaluate(), cMatExp2.evaluate(), SM_SOURCE_FILE_POS);// make sure revertUpdate() method resets cache
    sm::eigen::assertEqual(evaluateJacobian(matExp2), evaluateJacobian(cMatExp2), SM_SOURCE_FILE_POS);

    dv.setParameters(dx);
    sm::eigen::assertEqual(matExp2.evaluate(), cMatExp2.evaluate(), SM_SOURCE_FILE_POS);// make sure setParameter() method resets cache
    sm::eigen::assertEqual(evaluateJacobian(matExp2), evaluateJacobian(cMatExp2), SM_SOURCE_FILE_POS);
  }
  catch(std::exception const & e)
  {
    FAIL() << e.what();
  }

}

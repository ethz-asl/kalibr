#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/kinematics/rotations.hpp>
#include <Eigen/Geometry>
#include <aslam/backend/DesignVariableGenericVector.hpp>
#include <aslam/backend/GenericMatrixExpression.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/DesignVariableVector.hpp>
#include <aslam/backend/VectorExpressionToGenericMatrixTraits.hpp>
#include <aslam/backend/test/ExpressionTests.hpp>

using namespace aslam::backend;
using namespace std;

TEST(GenericMatrixExpressionNodeTestSuites, testGenericMatrixBasicOperations) {
  try {
    const int VEC_ROWS = 5;

    typedef GenericMatrixExpression<4, 2> GMAT;
    typedef GenericMatrixExpression<2, VEC_ROWS> GMAT2;

    auto identity = Eigen::Matrix<double, VEC_ROWS, VEC_ROWS>::Identity();
    GenericMatrixExpression<VEC_ROWS, VEC_ROWS> identityExp(identity);

    GMAT::matrix_t mat = GMAT::matrix_t::Random();
    GMAT2::matrix_t mat2 = GMAT2::matrix_t::Random();
    GMAT matExp(mat);
    GMAT2 matExp2(mat2);

    sm::eigen::assertNear(matExp.evaluate(), mat, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluation fits initialization.");
    sm::eigen::assertNear(matExp2.evaluate(), mat2, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluation fits initialization.");

    sm::eigen::assertNear(identityExp.evaluate(), identity, 1e-14, SM_SOURCE_FILE_POS, "Testing the transpose method.");
    sm::eigen::assertNear((matExp.transpose()).evaluate(), mat.transpose(), 1e-14, SM_SOURCE_FILE_POS, "Testing the transpose method.");
    sm::eigen::assertNear((matExp * matExp2).evaluate(), mat * mat2, 1e-14, SM_SOURCE_FILE_POS, "Testing the product.");
    sm::eigen::assertNear((matExp + matExp).evaluate(), 2 * mat, 1e-14, SM_SOURCE_FILE_POS, "Testing the sum.");
    sm::eigen::assertNear((matExp - matExp).evaluate(), 0 * mat, 1e-14, SM_SOURCE_FILE_POS, "Testing the difference.");

    // vector design variable
    typedef GenericMatrixExpression<VEC_ROWS, 1, double> GV;
    typedef DesignVariableGenericVector<VEC_ROWS> DGvec;
    GV::matrix_t vec = GV::matrix_t::Random();
    DGvec dv(vec);
    dv.setActive(true);
    dv.setBlockIndex(0);
    GV vecExp(&dv);

    sm::eigen::assertNear((matExp2 * vecExp).evaluate(), mat2 * vec, 1e-14, SM_SOURCE_FILE_POS, "Testing the matrix vector design variable product.");

    {
      SCOPED_TRACE("");
      JacobianContainer jc(vec.rows());
      sm::eigen::assertEqual(dv.value(), vec, SM_SOURCE_FILE_POS,"Testing evaluation fits initialization.");
      sm::eigen::assertEqual(vecExp.evaluate(), vec, SM_SOURCE_FILE_POS, "Testing evaluation fits initialization.");
      vecExp.evaluateJacobians(jc);
      sm::eigen::assertNear(jc.asDenseMatrix(), identity, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

    {
      SCOPED_TRACE("");
      JacobianContainer jc(vec.rows());
      auto exp = identityExp * vecExp;
      exp.evaluateJacobians(jc);
      sm::eigen::assertNear(exp.evaluate(), vec, 1e-14, SM_SOURCE_FILE_POS, "Testing the transposed design variable matrix product.");
      sm::eigen::assertNear(jc.asDenseMatrix(), identity, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

    {
      SCOPED_TRACE("");
      JacobianContainer jc(VEC_ROWS);
      auto exp = -vecExp;
      (exp).evaluateJacobians(jc);
      sm::eigen::assertNear(exp.evaluate(), -(vecExp.evaluate()), 1e-14, SM_SOURCE_FILE_POS, "Testing the negated design variable.");
      sm::eigen::assertNear(jc.asDenseMatrix(), -identity, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }
    {
      SCOPED_TRACE("");
      JacobianContainer jc(mat2.rows());
      auto exp = matExp2 * vecExp;
      (exp).evaluateJacobians(jc);
      sm::eigen::assertNear(exp.evaluate(), matExp2.evaluate() * vecExp.evaluate(), 1e-14, SM_SOURCE_FILE_POS, "Testing the design variable matrix product.");
      sm::eigen::assertNear(jc.asDenseMatrix(), mat2, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }
    {
      SCOPED_TRACE("");
      JacobianContainer jc(mat2.rows());
      auto exp = (vecExp.transpose() * matExp2.transpose()).transpose();
      exp.evaluateJacobians(jc);
      sm::eigen::assertNear(exp.evaluate(), matExp2.evaluate() * vecExp.evaluate(), 1e-14, SM_SOURCE_FILE_POS, "Testing the transposed design variable matrix product.");
      sm::eigen::assertNear(jc.asDenseMatrix(), mat2, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

    {
      SCOPED_TRACE("");
      JacobianContainer jc(1);
      auto exp = (vecExp.transpose() * vecExp);
      exp.evaluateJacobians(jc);
      sm::eigen::assertNear(exp.evaluate(), vecExp.transpose().evaluate() * vecExp.evaluate(), 1e-14, SM_SOURCE_FILE_POS, "Testing the design variable vector square.");
      sm::eigen::assertNear(jc.asDenseMatrix(), 2 * vec.transpose(), 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

    {
      JacobianContainer jc(1);
      auto exp = (vecExp.transpose() * vecExp).inverse();
      Eigen::Matrix<double, 1, 1> val;
      val(0, 0) = 1/vec.dot(vec);
      sm::eigen::assertNear(exp.evaluate(), val, 1e-14, SM_SOURCE_FILE_POS, "Testing the inverse operation on a 1 x 1 matrix.");
      exp.evaluateJacobians(jc);
      sm::eigen::assertNear(jc.asDenseMatrix(), - 2 * vec.transpose() * (1 / (vec.dot(vec) * vec.dot(vec))), 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

    {
      auto inv = vecExp * vecExp.transpose();
      auto invValue = (vec * vec.transpose()).eval();
      sm::eigen::assertNear(inv.evaluate(), invValue, 1e-14, SM_SOURCE_FILE_POS, "Testing the dyadic product.");

      JacobianContainer jc(VEC_ROWS);
      for( int i = 0; i < VEC_ROWS; i++){
        jc.clear();
        GV::matrix_t testVectorValue = GV::matrix_t::Zero();
        testVectorValue[i] = 1;
        GV testVector(testVectorValue);
        auto exp = inv * testVector;
        sm::eigen::assertNear(exp.evaluate(), invValue * testVectorValue, 1e-14, SM_SOURCE_FILE_POS, "Testing the dyadic product.");
        testJacobian(exp);
        exp.evaluateJacobians(jc);
        sm::eigen::assertNear(jc.asDenseMatrix(), identity * vec.dot(testVectorValue) + vec * (testVectorValue).transpose(), 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
      }
    }

    {
      auto inv = (identityExp + vecExp * vecExp.transpose()).inverse();
      auto invValue = (identity + vec * vec.transpose()).inverse().eval();
      sm::eigen::assertNear(inv.evaluate(), invValue, 1e-14, SM_SOURCE_FILE_POS, "Testing the inverse operation.");

      JacobianContainer jc(VEC_ROWS);
      for( int i = 0; i < VEC_ROWS; i++){
        jc.clear();
        GV::matrix_t testVectorValue = GV::matrix_t::Zero();
        testVectorValue[i] = 1;
        GV testVector(testVectorValue);
        auto exp = inv * testVector;
        sm::eigen::assertNear(exp.evaluate(), invValue * testVectorValue, 1e-14, SM_SOURCE_FILE_POS, "Testing the inverse operation.");
        testJacobian(exp);
        exp.evaluateJacobians(jc);
        sm::eigen::assertNear(jc.asDenseMatrix(), - invValue * (identity * vec.dot(invValue * testVectorValue) + vec * (invValue * testVectorValue).transpose()), 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
      }
    }

    {
      SCOPED_TRACE("");
      JacobianContainer jc(VEC_ROWS);
      auto exp = vecExp * 3;
      GV exp2 = exp;
      exp2.evaluateJacobians(jc);
      sm::eigen::assertNear(exp2.evaluate(), vec * 3, 1e-14, SM_SOURCE_FILE_POS, "Testing the design variable times a scalar.");
      sm::eigen::assertNear(jc.asDenseMatrix(), identity * 3, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

    typedef GenericMatrixExpression<VEC_ROWS, VEC_ROWS> GM3;
    GM3::matrix_t mat3 = GM3::matrix_t::Random();
    GM3 matExp3(mat3);
    {
      SCOPED_TRACE("");
      JacobianContainer jc(1);
      (vecExp.transpose() * matExp3 * vecExp).evaluateJacobians(jc);
      sm::eigen::assertNear(jc.asDenseMatrix(), vec.transpose() * (mat3 + mat3.transpose()), 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

      // second vector design variable
    GV::matrix_t vec2 = GV::matrix_t::Random();
    DGvec dv2(vec2);
    dv2.setActive(true);
    dv2.setBlockIndex(1);
    GV vecExp2(&dv2);

    {
      SCOPED_TRACE("");
      JacobianContainer jc(1);
      (vecExp2.transpose() * matExp3 * vecExp).evaluateJacobians(jc);

      Eigen::Matrix<double, 1, VEC_ROWS * 2> result;
      result.head<VEC_ROWS>() = vec2.transpose() * mat3;
      result.tail<VEC_ROWS>() = vec.transpose() * mat3.transpose();

      sm::eigen::assertNear(jc.asDenseMatrix(), result, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

    {
      SCOPED_TRACE("");
      JacobianContainer jc(VEC_ROWS);
      (vecExp + vecExp2).evaluateJacobians(jc);
      Eigen::Matrix<double, VEC_ROWS, 2 * VEC_ROWS> result;
      result.block<VEC_ROWS, VEC_ROWS>(0, 0).setIdentity();
      result.block<VEC_ROWS, VEC_ROWS>(0, VEC_ROWS).setIdentity();

      sm::eigen::assertNear(jc.asDenseMatrix(), result, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

    {
      SCOPED_TRACE("");
      JacobianContainer jc(VEC_ROWS);
      (vecExp - vecExp2).evaluateJacobians(jc);
      Eigen::Matrix<double, VEC_ROWS, 2 * VEC_ROWS> result;
      result.block<VEC_ROWS, VEC_ROWS>(0, 0).setIdentity();
      result.block<VEC_ROWS, VEC_ROWS>(0, VEC_ROWS).setIdentity() *= -1;

      sm::eigen::assertNear(jc.asDenseMatrix(), result, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }
  }
  catch(std::exception const & e)
  {
    FAIL() << e.what();
  }
}

TEST(GenericMatrixExpressionNodeTestSuites, testVectorExpressionToGenericMatrixExpression) {
  try {
    const int VEC_ROWS = 5;

    DesignVariableVector<VEC_ROWS> dvec;
    dvec.setActive(true);
    dvec.setBlockIndex(0);
    Eigen::MatrixXd vec = Eigen::MatrixXd::Random(VEC_ROWS, 1);
    dvec.setParameters(vec);

    VectorExpression<VEC_ROWS> vecExp(&dvec);
    auto gVecExp = convertToGME(vecExp);

    SCOPED_TRACE("");
    sm::eigen::assertNear(gVecExp.evaluate(), vec, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluation fits initialization.");
    sm::eigen::assertNear(gVecExp.transpose().evaluate(), vec.transpose(), 1e-14, SM_SOURCE_FILE_POS, "Testing evaluation fits initialization.");

    typedef GenericMatrixExpression<VEC_ROWS, 1, double> GV;
    GV gVec(vec);
    {
      SCOPED_TRACE("");
      JacobianContainer jc(1);
      auto exp = (gVecExp.transpose() * gVecExp);
      sm::eigen::assertNear(exp.evaluate(), (vec.transpose() * vec).eval(), 1e-14, SM_SOURCE_FILE_POS, "Testing the design variable matrix square.");
      exp.evaluateJacobians(jc);
      sm::eigen::assertNear(jc.asDenseMatrix(), (2 * vec.transpose()).eval(), 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }

    auto identity = Eigen::Matrix<double, VEC_ROWS, VEC_ROWS>::Identity();
    GenericMatrixExpression<VEC_ROWS, VEC_ROWS> identityExp(identity);

    {
      SCOPED_TRACE("");
      JacobianContainer jc(VEC_ROWS);
      auto exp = (identityExp * gVecExp);
      sm::eigen::assertNear(exp.evaluate(), vec.eval(), 1e-14, SM_SOURCE_FILE_POS, "Testing the identity times the converted vector expression.");
      exp.evaluateJacobians(jc);
      sm::eigen::assertNear(jc.asDenseMatrix(), identity, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }
    {
      SCOPED_TRACE("");
      JacobianContainer jc(VEC_ROWS);
      auto exp = (gVecExp.transpose() * identityExp).transpose();
      sm::eigen::assertNear(exp.evaluate(), vec.eval(), 1e-14, SM_SOURCE_FILE_POS, "Testing the identity times the converted vector expression.");
      exp.evaluateJacobians(jc);
      sm::eigen::assertNear(jc.asDenseMatrix(), identity, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }
    {
      SCOPED_TRACE("");
      JacobianContainer jc(VEC_ROWS);
      auto exp = gVecExp.transpose().transpose();
      sm::eigen::assertNear(exp.evaluate(), vec.eval(), 1e-14, SM_SOURCE_FILE_POS, "Testing the identity times the converted vector expression.");
      exp.evaluateJacobians(jc);
      sm::eigen::assertNear(jc.asDenseMatrix(), identity, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }
  }
  catch(std::exception const & e)
  {
    FAIL() << e.what();
  }
}

TEST(GenericMatrixExpressionNodeTestSuites, testCrossProduct) {
  try {
    const int VEC_ROWS = 3;

    typedef Eigen::Matrix<double, VEC_ROWS, 1> vector_t;
    DesignVariableVector<VEC_ROWS> dvec, dvec2;
    dvec.setActive(true);
    dvec.setBlockIndex(0);
    dvec2.setActive(true);
    dvec2.setBlockIndex(1);
    vector_t vec = vector_t::Random();
    vector_t vec2 = vector_t::Random();

    dvec.setParameters(vec);
    VectorExpression<VEC_ROWS> vecExp(&dvec);
    dvec2.setParameters(vec2);
    VectorExpression<VEC_ROWS> vec2Exp(&dvec2);

    auto gVecExp = convertToGME(vecExp);
    auto gVec2Exp = convertToGME(vec2Exp);

    typedef GenericMatrixExpression<VEC_ROWS, 2, double> GMAT;
    typename GMAT::matrix_t mat = GMAT::matrix_t::Random();
    GMAT gMat(mat);
    {
      SCOPED_TRACE("");
      auto exp = (gVecExp.cross(gMat));
      sm::eigen::assertNear(exp.evaluate().col(0), vec.cross(mat.col(0)), 1e-14, SM_SOURCE_FILE_POS, "Testing cross product evaluation.");
      sm::eigen::assertNear(exp.evaluate().col(1), vec.cross(mat.col(1)), 1e-14, SM_SOURCE_FILE_POS, "Testing cross product evaluation.");
    }
    {
      SCOPED_TRACE("");
      auto exp = (gVecExp.cross(gVec2Exp));
      sm::eigen::assertNear(exp.evaluate(), vec.cross(vec2), 1e-14, SM_SOURCE_FILE_POS, "Testing cross product evaluation.");
      JacobianContainer jc(VEC_ROWS);
      exp.evaluateJacobians(jc);
      Eigen::Matrix<double, VEC_ROWS, 2 * VEC_ROWS> result;
      result.block<VEC_ROWS, VEC_ROWS>(0, 0) = sm::kinematics::crossMx(-vec2);
      result.block<VEC_ROWS, VEC_ROWS>(0, VEC_ROWS) = sm::kinematics::crossMx(vec);
      sm::eigen::assertNear(jc.asDenseMatrix(), result, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }
    {
      SCOPED_TRACE("");
      auto exp = (gVecExp.cross(gMat) * GenericMatrixExpression<2, 1>(Eigen::Matrix<double, 2, 1>::Ones()));
      sm::eigen::assertNear(exp.evaluate(), vec.cross(mat.col(0)) + vec.cross(mat.col(1)), 1e-14, SM_SOURCE_FILE_POS, "Testing cross product evaluation.");
      JacobianContainer jc(VEC_ROWS);
      exp.evaluateJacobians(jc);
      Eigen::Matrix<double, VEC_ROWS, VEC_ROWS> result;
      result = sm::kinematics::crossMx(-mat.col(0)) + sm::kinematics::crossMx(-mat.col(1));
      sm::eigen::assertNear(jc.asDenseMatrix(), result, 1e-14, SM_SOURCE_FILE_POS, "Testing evaluationJacobian fits theoretical value.");
    }
  }
  catch(std::exception const & e)
  {
    FAIL() << e.what();
  }
}

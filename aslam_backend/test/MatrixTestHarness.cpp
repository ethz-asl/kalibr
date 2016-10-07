#include "MatrixTestHarness.hpp"
#include <sm/eigen/gtest.hpp>

MatrixTestHarness::MatrixTestHarness(aslam::backend::Matrix* M) : matrix(M)
{
}

MatrixTestHarness::~MatrixTestHarness()
{
}


void MatrixTestHarness::testDense()
{
  Eigen::MatrixXd M(3, 7);
  M.setRandom();
  matrix->fromDense(M);
  ASSERT_DOUBLE_MX_EQ(M, *matrix, 1e-9, "Assign full");
  for (int i = 0; i < 3; ++i)
    M(i, i) = 0.0;
  matrix->fromDense(M);
  ASSERT_DOUBLE_MX_EQ(M, *matrix, 1e-9, "Assign sparse zeros");
  for (int i = 0; i < 3; ++i)
    ASSERT_EQ(0.0, (*matrix)(i, i));
  for (int i = 0; i < 3; ++i)
    M(i, i) = 1e-14;
  matrix->fromDenseTolerance(M, 1e-14);
  for (int i = 0; i < 3; ++i)
    M(i, i) = 0.0;
  ASSERT_DOUBLE_MX_EQ(M, *matrix, 1e-9, "Assign sparse tolerance");
  for (int i = 0; i < 3; ++i)
    ASSERT_EQ(0.0, (*matrix)(i, i));
}


void MatrixTestHarness::testRightMultiply()
{
  Eigen::MatrixXd M(3, 7);
  M.setRandom();
  for (int i = 0; i < 3; ++i)
    M(i, i) = 0.0;
  matrix->fromDense(M);
  Eigen::VectorXd v(M.cols());
  v.setRandom();
  Eigen::VectorXd Mv = M * v;
  Eigen::VectorXd MMv;
  matrix->rightMultiply(v, MMv);
  ASSERT_DOUBLE_MX_EQ(Mv, MMv, 1e-9, "Mv is from the dense test matrix MMv is from the matrix being tested");
}


void MatrixTestHarness::testLeftMultiply()
{
  Eigen::MatrixXd M(3, 7);
  M.setRandom();
  for (int i = 0; i < 3; ++i)
    M(i, i) = 0.0;
  matrix->fromDense(M);
  Eigen::VectorXd v(M.rows());
  v.setRandom();
  Eigen::VectorXd Mv = M.transpose() * v;
  Eigen::VectorXd MMv;
  matrix->leftMultiply(v, MMv);
  ASSERT_DOUBLE_MX_EQ(Mv, MMv, 1e-9, "Mv is from the dense test matrix MMv is from the matrix being tested");
}


void MatrixTestHarness::testAll()
{
  try {
    {
      SCOPED_TRACE("Test dense");
      testDense();
    }
    {
      SCOPED_TRACE("Test left multiply");
      testLeftMultiply();
    }
    {
      SCOPED_TRACE("Test right multiply");
      testRightMultiply();
    }
  } catch (const std::exception& e) {
    FAIL() << e.what();
  }
}


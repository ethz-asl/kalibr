#include <sm/eigen/gtest.hpp>

#include <aslam/backend/SparseBlockMatrixWrapper.hpp>
#include <numeric>
#include "MatrixTestHarness.hpp"

TEST(SparseMatrixTestSuite, testMatInterface)
{
  using namespace aslam::backend;
  SparseBlockMatrixWrapper M;
  // The test harness expects a 3x7 matrix. Let's use a block 3x3 3x4
  Eigen::VectorXi blockRows(1);
  blockRows << 3;
  Eigen::VectorXi blockCols(2);
  blockCols << 3, 7;
  M._M = sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>(blockRows, blockCols);
  ASSERT_EQ(3u, M.rows());
  ASSERT_EQ(7u, M.cols());
  MatrixTestHarness mth(&M);
  SCOPED_TRACE("");
  mth.testAll();
}


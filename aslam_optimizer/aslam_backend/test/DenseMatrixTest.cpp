#include <sm/eigen/gtest.hpp>

#include <aslam/backend/DenseMatrix.hpp>
#include <numeric>
#include "MatrixTestHarness.hpp"

TEST(DenseMatrixTestSuite, testMatInterface)
{
  using namespace aslam::backend;
  DenseMatrix M;
  MatrixTestHarness mth(&M);
  SCOPED_TRACE("");
  mth.testAll();
}


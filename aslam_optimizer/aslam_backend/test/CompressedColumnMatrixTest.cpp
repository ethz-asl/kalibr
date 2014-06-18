#include <sm/eigen/gtest.hpp>

#include <aslam/backend/CompressedColumnMatrix.hpp>
#include <numeric>
#include "DummyDesignVariable.hpp"
#include <aslam/backend/CompressedColumnJacobianTransposeBuilder.hpp>
#include "SampleDvAndError.hpp"
#include "MatrixTestHarness.hpp"

TEST(CompressColumnMatrixTestSuite, testMatInterface)
{
  using namespace aslam::backend;
  CompressedColumnMatrix<int> M;
  MatrixTestHarness mth(&M);
  SCOPED_TRACE("");
  mth.testAll();
}

TEST(CompressColumnMatrixTestSuite, testJcBuild)
{
  const int Jrows = 3;
  using namespace aslam::backend;
  std::vector<int> blocks;
  std::vector<DesignVariable*> dvs;
  std::vector<Eigen::MatrixXd> Js;
  DummyDesignVariable<3> dv0;
  dvs.push_back(&dv0);
  DummyDesignVariable<4> dv1;
  dvs.push_back(&dv1);
  DummyDesignVariable<5> dv2;
  dvs.push_back(&dv2);
  DummyDesignVariable<2> dv3;
  dvs.push_back(&dv3);
  DummyDesignVariable<4> dv4;
  dvs.push_back(&dv4);
  Js.resize(dvs.size());
  int block = 0;
  for (unsigned i = 0; i < dvs.size(); ++i) {
    blocks.push_back(dvs[i]->minimalDimensions());
    dvs[i]->setBlockIndex(i);
    dvs[i]->setActive(true);
    dvs[i]->setColumnBase(block);
    block += dvs[i]->minimalDimensions();
    Js[i].resize(Jrows, dvs[i]->minimalDimensions());
    Js[i].setRandom();
  }
  std::vector<int> cbi = blocks;
  std::partial_sum(cbi.begin(), cbi.end(), cbi.begin());
  CompressedColumnMatrix<> mat(cbi.back(), 0, 0, 0);
  JacobianContainer jc0(Jrows);
  jc0.add(dvs[0], Js[0]);
  jc0.add(dvs[4], Js[4]);
  ASSERT_EQ(dvs[0]->blockIndex(), 0);
  ASSERT_EQ(dvs[4]->blockIndex(), 4);
  ASSERT_EQ(jc0.numDesignVariables(), 2u);
  mat.appendJacobians(jc0);
  ASSERT_EQ((int)mat.rows(), cbi.back());
  ASSERT_EQ((int)mat.cols(), Jrows);
  ASSERT_EQ((int)mat.cols(), Jrows);
  ASSERT_EQ((int)mat.nnz(), Js[0].rows() * Js[0].cols() + Js[4].rows() * Js[4].cols());
  Eigen::MatrixXd sb = jc0.asDenseMatrix(cbi).transpose();
  ASSERT_DOUBLE_MX_EQ(sb, mat, 1e-6, "");
  JacobianContainer jc1(Jrows);
  jc1.add(dvs[1], Js[1]);
  jc1.add(dvs[3], Js[3]);
  jc1.add(dvs[4], Js[4]);
  mat.appendJacobians(jc1);
  Eigen::MatrixXd sb2(sb.rows(), Jrows * 2);
  sb2 << sb, jc1.asDenseMatrix(cbi).transpose();
  ASSERT_DOUBLE_MX_EQ(sb2, mat, 1e-6, "");
  //    std::cout << mat << std::endl;
}


TEST(CompressColumnMatrixTestSuite, testJcBuilder)
{
  using namespace aslam::backend;
  const int D = 4;
  const int E = 9;
  std::vector<DesignVariable*> dvs;
  std::vector<ErrorTerm*> errs;
  try {
    CompressedColumnJacobianTransposeBuilder<int> ccjtb;
    int blockBase = 0;
    std::vector<int> blocks;
    for (int i = 0; i < D; ++i) {
      dvs.push_back(new Point2d(Eigen::Vector2d::Random()));
      dvs.back()->setActive(true);
      dvs.back()->setBlockIndex(i);
      dvs.back()->setColumnBase(blockBase);
      blockBase += dvs.back()->minimalDimensions();
      blocks.push_back(blockBase);
    }
    int cols = 0;
    for (int i = 0; i < E; ++i) {
      int mod = i % 3;
      if (mod == 0) {
        //std::cout << "le1\n";
        errs.push_back(new LinearErr((Point2d*)dvs[i % dvs.size()]));
      } else if (mod == 1) {
        //std::cout << "le2\n";
        errs.push_back(new LinearErr2((Point2d*)dvs[i % dvs.size()], (Point2d*)dvs[(i + 1) % dvs.size()]));
      } else {
        //std::cout << "le3\n";
        errs.push_back(new LinearErr3((Point2d*)dvs[i % dvs.size()], (Point2d*)dvs[(i + 1) % dvs.size()], (Point2d*)dvs[(i + 2) % dvs.size() ]));
      }
      errs.back()->setRowBase(cols);
      cols += errs.back()->dimension();
    }
    ccjtb.initMatrixStructure(dvs, errs);
    Eigen::VectorXd diag = Eigen::VectorXd::Random(blocks.back());
    ccjtb.J_transpose().pushDiagonalBlock(diag);
    ASSERT_EQ(blockBase, (int)ccjtb.J_transpose().rows());
    ASSERT_EQ(cols + diag.size(), (int)ccjtb.J_transpose().cols());
    double e0 = 0;
    for (unsigned i = 0; i < errs.size(); ++i)
      e0 += errs[i]->evaluateError();
    /// Test building the Jacobian with different numbers of threads.
    for (unsigned j = 0; j < 2 * errs.size(); ++j) {
      ccjtb.buildSystem(j, false);
      Eigen::MatrixXd J = ccjtb.J_transpose().toDense().transpose();
      int rowStart = 0;
      for (unsigned i = 0; i < errs.size(); ++i) {
        JacobianContainer jc(errs[i]->dimension());
        Eigen::MatrixXd Jrow = J.block(rowStart, 0, errs[i]->dimension(), J.cols());
        errs[i]->getWeightedJacobians(jc, false);
        Eigen::MatrixXd JrowFromError = jc.asDenseMatrix(blocks);
        ASSERT_DOUBLE_MX_EQ(Jrow, JrowFromError, 1e-6, "these should be similar. Number of threads: " << j << ", block row: " << i);
        rowStart += errs[i]->dimension();
      }
      // Test the diagonal block
      Eigen::MatrixXd denseDiag = diag.asDiagonal();
      Eigen::MatrixXd diagonal = J.bottomRows(J.cols());
      ASSERT_DOUBLE_MX_EQ(diagonal, denseDiag, 1e-6, "these should be similar. Number of threads: " << j);
    }
  } catch (std::exception const& e) {
    for (int i = 0; i < D; ++i)
      delete dvs[i];
    for (int i = 0; i < E; ++i)
      delete errs[i];
    FAIL() << e.what();
  }
  for (unsigned i = 0; i < dvs.size(); ++i)
    delete dvs[i];
  for (unsigned i = 0; i < errs.size(); ++i)
    delete errs[i];
}


TEST(CompressColumnMatrixTestSuite, testAppendDiagonal)
{
  const int rows = 5;
  using namespace aslam::backend;
  CompressedColumnMatrix<> mat(rows, 0, 0, 0);
  mat.pushConstantDiagonalBlock(1.0);
  Eigen::MatrixXd matDense = mat.toDense();
  ASSERT_DOUBLE_MX_EQ(matDense, Eigen::MatrixXd::Identity(rows, rows), 1e-6, "");
  ASSERT_THROW(mat.pushConstantDiagonalBlock(1.0), CompressedColumnMatrix<>::Exception);
  mat.popDiagonalBlock();
  Eigen::VectorXd diag(rows);
  diag.setRandom();
  mat.pushDiagonalBlock(diag);
  matDense = mat.toDense();
  Eigen::MatrixXd diagDense = diag.asDiagonal();
  ASSERT_DOUBLE_MX_EQ(matDense, diagDense, 1e-6, "");
}

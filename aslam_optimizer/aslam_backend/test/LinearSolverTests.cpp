#include <sm/eigen/gtest.hpp>

#include <numeric>

#include "SampleDvAndError.hpp"

#include <aslam/backend/DenseQrLinearSystemSolver.hpp>
#include <aslam/backend/SparseCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <boost/lexical_cast.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/OptimizationProblem.hpp>

using namespace aslam::backend;


template<typename S1_TYPE, typename S2_TYPE>
void compareSolvers(int D, int E, bool useM, bool useDiag, int nThreads)
{
  std::string S1Name = typeid(S1_TYPE).name();
  std::string S2Name = typeid(S2_TYPE).name();
  SCOPED_TRACE(("Comparing S1: " + S1Name + " and S2: " + S2Name).c_str());
  std::vector<DesignVariable*> dvs;
  std::vector<ErrorTerm*> errs;
  try {
    buildSystem(D, E, dvs, errs);
    S1_TYPE S1;
    S1.initMatrixStructure(dvs, errs, useDiag);
    S2_TYPE S2;
    S2.initMatrixStructure(dvs, errs, useDiag);
    ASSERT_EQ(S1.JRows(), S2.JRows());
    ASSERT_EQ(S1.JCols(), S2.JCols());
    Eigen::VectorXd diag(S1.JCols());
    diag.setRandom();
    if (useDiag) {
      S1.setConditioner(diag);
      S2.setConditioner(diag);
    }
    Eigen::VectorXd dxS1, dxS2, eS2, eS1, rhsS2, rhsS1;
    double e2S1, e2S2;
    e2S1 = S1.evaluateError(nThreads, useM);
    e2S2 = S2.evaluateError(nThreads, useM);
    ASSERT_NEAR(e2S1, e2S2, 1e-9);
    eS2 = S2.e();
    eS1 = S1.e();
    ASSERT_DOUBLE_MX_EQ(eS2, eS1, 1e-6, "Checking the error vectors");
    ASSERT_EQ((size_t)S2.e().size(), S2.JRows());
    ASSERT_EQ((size_t)S1.e().size(), S1.JRows());
    S1.buildSystem(nThreads, useM);
    S2.buildSystem(nThreads, useM);
    rhsS1 = S1.rhs();
    rhsS2 = S2.rhs();
    ASSERT_DOUBLE_MX_EQ(rhsS1, rhsS2, 1e-6, "Checking right-hand sides");
    S1.solveSystem(dxS1);
    S2.solveSystem(dxS2);
    ASSERT_DOUBLE_MX_EQ(dxS2, dxS1, 1e-6, "Checking the solutions");
    deleteSystem(dvs, errs);
  } catch (const std::exception& e) {
    deleteSystem(dvs, errs);
    FAIL() << e.what();
  }
}
/*
TEST(LinearSolverTestSuite, testDenseQr)
{
   using namespace aslam::backend;
   std::vector<DesignVariable *> dvs;
   std::vector<ErrorTerm *> errs;
   const int D = 4;
   const int E = 20;
   const bool useM = false;
   bool useDiag = true;

   for(int nThreads = 0; nThreads < 16; ++nThreads)
   {
       {
           useDiag = false;
           SCOPED_TRACE(("No Diagonal and " + boost::lexical_cast<std::string>(nThreads) + " threads").c_str());
           compareSolvers<BlockCholeskyLinearSystemSolver, DenseQrLinearSystemSolver>(D,E,useM, useDiag, nThreads);
       }

       {
           useDiag = true;
           SCOPED_TRACE(("With Diagonal and " + boost::lexical_cast<std::string>(nThreads) + " threads").c_str());
           compareSolvers<BlockCholeskyLinearSystemSolver, DenseQrLinearSystemSolver>(D,E,useM, useDiag, nThreads);
       }
   }


}
*/

TEST(LinearSolverTestSuite, testSparseCholesky)
{
  using namespace aslam::backend;
  std::vector<DesignVariable*> dvs;
  std::vector<ErrorTerm*> errs;
  const int D = 4;
  const int E = 20;
  const bool useM = false;
  bool useDiag = true;
  for (int nThreads = 0; nThreads < 4; ++nThreads) {
    {
      useDiag = false;
      SCOPED_TRACE(("No Diagonal and " + boost::lexical_cast<std::string>(nThreads) + " threads").c_str());
      compareSolvers<SparseCholeskyLinearSystemSolver, BlockCholeskyLinearSystemSolver>(D, E, useM, useDiag, nThreads);
    }
    {
      useDiag = true;
      SCOPED_TRACE(("With Diagonal and " + boost::lexical_cast<std::string>(nThreads) + " threads").c_str());
      compareSolvers<SparseCholeskyLinearSystemSolver, BlockCholeskyLinearSystemSolver>(D, E, useM, useDiag, nThreads);
    }
  }
}

TEST(LinearSolverTestSuite, testSparseQR)
{
  using namespace aslam::backend;
  std::vector<DesignVariable*> dvs;
  std::vector<ErrorTerm*> errs;
  const int D = 4;
  const int E = 20;
  const bool useM = false;
  bool useDiag = false;
  for (int nThreads = 0; nThreads < 4; ++nThreads) {
    {
      useDiag = false;
      SCOPED_TRACE(("No Diagonal and " + boost::lexical_cast<std::string>(nThreads) + " threads").c_str());
      compareSolvers<SparseCholeskyLinearSystemSolver, SparseQrLinearSystemSolver>(D, E, useM, useDiag, nThreads);
    }
    /*
            {
                useDiag = true;
                SCOPED_TRACE(("With Diagonal and " + boost::lexical_cast<std::string>(nThreads) + " threads").c_str());
                compareSolvers<SparseCholeskyLinearSystemSolver, SparseQrLinearSystemSolver>(D,E,useM, useDiag, nThreads);
            }*/
  }
}

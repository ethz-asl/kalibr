// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Helpful functions from libasrl
#include <sm/eigen/gtest.hpp>

#include <sparse_block_matrix/linear_solver_cholmod.h>
#include <sparse_block_matrix/linear_solver_dense.h>
#include <sparse_block_matrix/linear_solver_spqr.h>

template<typename SOLVER_T>
void randomSparseBlockMatrix(sparse_block_matrix::SparseBlockMatrix<typename SOLVER_T::matrix_t> * A,   Eigen::MatrixXd & Adense ) {
	typedef typename SOLVER_T::matrix_t SparseMatrixBlock;

	// Fill in like this:
	  //   0  1   2
	  // 0 a
	  // 1    b   d
	  // 2        c
	  //
	  // where a,b,c are symmetric
	  // d is 3x5

	  SparseMatrixBlock * a = A->block(0,0,true);
	  ASSERT_TRUE(a != NULL);
	  ASSERT_EQ(a->rows(),3);
	  ASSERT_EQ(a->cols(),3);
	  a->setRandom();
	  *a = (*a * a->transpose()) + Eigen::Matrix3d::Identity();
	  Adense.block(0,0,3,3) = *a;

	  SparseMatrixBlock * b = A->block(1,1,true);
	  ASSERT_TRUE(b != NULL);
	  ASSERT_EQ(b->rows(),3);
	  ASSERT_EQ(b->cols(),3);
	  b->setRandom();
	  *b = (*b * b->transpose()) + Eigen::Matrix3d::Identity();
	  Adense.block(3,3,3,3) = *b;

	  SparseMatrixBlock * c = A->block(2,2,true);
	  ASSERT_TRUE(c != NULL);
	  ASSERT_EQ(c->rows(),5);
	  ASSERT_EQ(c->cols(),5);
	  c->setRandom();
	  *c = (*c * c->transpose()).eval() + Eigen::MatrixXd::Identity(5,5);
	  //std::cout << "c:\n" << *c << std::endl;
	  Adense.block(6,6,5,5) = *c;

	  SparseMatrixBlock * d = A->block(1,2,true);
	  ASSERT_TRUE(d != NULL);
	  ASSERT_EQ(d->rows(),3);
	  ASSERT_EQ(d->cols(),5);
	  d->setRandom();
	  Adense.block(3,6,3,5) = *d;

}




template<typename SOLVER_T>
void testSolver(const std::string & solver_name)
{  
  // Build up a sparse matrix
  // 3x3 3x3 3x5
  // 3x3 3x3 3x5
  // 5x3 5x3 5x5
  int rows[] = {3,6,11};
  int cols[] = {3,6,11};
  sparse_block_matrix::SparseBlockMatrix<typename SOLVER_T::matrix_t> A(rows,cols,3,3);

  Eigen::MatrixXd Adense(11,11);
  Adense.setZero();

  ASSERT_EQ(A.rows(),11);
  ASSERT_EQ(A.cols(),11);

  randomSparseBlockMatrix<SOLVER_T>(&A, Adense);



  Eigen::VectorXd xx(A.rows());
  xx.setZero();
  Eigen::VectorXd xx2(A.rows());
  xx2.setZero();


  SOLVER_T solver;
  ASSERT_TRUE(solver.init());
    
    Eigen::VectorXd bb(A.rows());
    bb.setRandom();
    
  // virtual bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) = 0;
 	ASSERT_TRUE( solver.solve(A,&xx[0],&bb[0]));
 	// Solve dense
 	Eigen::VectorXd dx = Adense.selfadjointView<Eigen::Upper>().ldlt().solve(bb);

 	//  always solve twice to make sure the value only copying is working for symbolic factorizations
 	randomSparseBlockMatrix<SOLVER_T>(&A, Adense);
 	ASSERT_TRUE( solver.solve(A,&xx2[0],&bb[0]));

 	Eigen::VectorXd dx2 = Adense.selfadjointView<Eigen::Upper>().ldlt().solve(bb);

 	// Solve dense


 	// Make sure the solutions match.
 	sm::eigen::assertNear(dx,xx,1e-10,SM_SOURCE_FILE_POS, "A: dense solution, B: solution from " + solver_name);
 	sm::eigen::assertNear(dx2,xx2,1e-10,SM_SOURCE_FILE_POS, "A: dense solution, B: solution from " + solver_name);


}

// Check that the setup as a whole is correct
TEST(g2oTestSuite, testCholmod)
{

//
   testSolver< sparse_block_matrix::LinearSolverQr<Eigen::MatrixXd> >("sparseQR");
  
  testSolver< sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd> >("cholmod");

  testSolver< sparse_block_matrix::LinearSolverDense<Eigen::MatrixXd> >("dense");


}

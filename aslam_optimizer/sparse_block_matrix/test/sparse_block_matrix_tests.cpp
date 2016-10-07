// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Helpful functions from schweizer_messer
#include <sm/eigen/gtest.hpp>
#include <sparse_block_matrix/sparse_block_matrix.h>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include "sbm_gtest.hpp"

typedef boost::minstd_rand base_generator_type;
// Returns a float r in the range 0.0 < f < 1.0.
inline double rand1() {
  static base_generator_type generator;
  
  // Define a uniform random number distribution which produces "double"
  // values between 0 and 1 (0 inclusive, 1 exclusive).
  static boost::uniform_real<> uni_dist(0, 1);
  static boost::variate_generator<base_generator_type&, boost::uniform_real<> > uni(generator, uni_dist);
  
  return uni();
}

template<typename M>
sparse_block_matrix::SparseBlockMatrix<M> buildRandomMatrix(Eigen::MatrixXi rows, Eigen::MatrixXi cols, double probabilityOfABlockBeingFilled) {
  typedef M matrix_t;

  sparse_block_matrix::SparseBlockMatrix<matrix_t> Mx(rows, cols);
  EXPECT_EQ((size_t )rows.size(), Mx.rowBlockIndices().size());
  EXPECT_EQ((size_t )cols.size(), Mx.colBlockIndices().size());

  size_t nonzeroBlocks = 0;
  size_t nonzeroElements = 0;
  for (int cb = 0; cb < Mx.bCols(); cb++) {
    for (int rb = 0; rb < Mx.bRows(); rb++) {
      if (rand1() < probabilityOfABlockBeingFilled) {
        // check all versions of the "block" function.
        matrix_t * B = Mx.block(rb, cb);
        EXPECT_EQ((matrix_t *)NULL, B) << "Block at " << rb << "," << cb
                                       << " should be null";

        B = Mx.block(rb, cb, false);
        EXPECT_EQ((matrix_t *)NULL, B) << "Block at " << rb << "," << cb
                                       << " should be null";

        B = Mx.block(rb, cb, true);
        EXPECT_NE((matrix_t *)NULL, B);
        EXPECT_EQ(B->rows(), Mx.rowsOfBlock(rb));
        EXPECT_EQ(B->cols(), Mx.colsOfBlock(cb));
        B->setRandom();
        ++nonzeroBlocks;
        nonzeroElements += B->rows() * B->cols();
        EXPECT_EQ(nonzeroBlocks, Mx.nonZeroBlocks());
        EXPECT_EQ(nonzeroElements, Mx.nonZeros());

      } else {
        // Check both versions of this function
        matrix_t * B = Mx.block(rb, cb);
        EXPECT_EQ((matrix_t *)NULL, B) << "Block at " << rb << "," << cb
                                       << " should be null";

        B = Mx.block(rb, cb, false);
        EXPECT_EQ((matrix_t *)NULL, B) << "Block at " << rb << "," << cb
                                       << " should be null";

      }
    }
  }

  return Mx;
}

TEST(sparse_block_matrixTestSuite, testBlock) {
  using namespace Eigen;
  using namespace sparse_block_matrix;
  VectorXi rows(3);
  rows << 3, 6, 11;
  VectorXi cols(4);
  cols << 4, 9, 12, 18;

  SparseBlockMatrix<MatrixXd> Mx(rows, cols, true);

  // Go through all the possible blocks.
  for (int cb = 0; cb < Mx.bCols(); cb++) {
    for (int rb = 0; rb < Mx.bRows(); rb++) {
      // This shouldn't allocate
      MatrixXd * B = Mx.block(rb, cb);
      ASSERT_EQ((MatrixXd *)NULL, B)<< "Block at " << rb << "," << cb << " should be null";

      // This shouldn't allocate
      B = Mx.block(rb, cb, false);
      ASSERT_EQ((MatrixXd *)NULL, B)<< "Block at " << rb << "," << cb << " should be null";

      // This should allocate.
      B = Mx.block(rb, cb, true);
      ASSERT_NE((MatrixXd *)NULL, B);
      ASSERT_EQ(B->rows(), Mx.rowsOfBlock(rb));
      ASSERT_EQ(B->cols(), Mx.colsOfBlock(cb));
    }
  }

}

TEST(sparse_block_matrixTestSuite, testOperator) {
  using namespace sparse_block_matrix;
  using namespace Eigen;

  VectorXi rows(2);
  rows << 2, 4;
  VectorXi cols(3);
  cols << 2, 5, 7;

  SparseBlockMatrix<MatrixXd> M(rows, cols, true);
  // Block rows and columns.
  ASSERT_EQ(2, M.bRows());
  ASSERT_EQ(3, M.bCols());
  // Actual rows and columns.
  ASSERT_EQ(4, M.rows());
  ASSERT_EQ(7, M.cols());

  for (int r = 0; r < M.rows(); r++) {
    for (int c = 0; c < M.cols(); c++) {
      ASSERT_EQ(0.0,M(r,c))<< "The uninitailzed matrix did not return zero at " << r << "," << c;
    }
  }

  // Now fill a block.
  SparseBlockMatrix<MatrixXd>::Block * B = M.block(1, 0, true);
  ASSERT_NE(B, (SparseBlockMatrix<MatrixXd>::Block *)NULL);
  ASSERT_EQ(B->rows(), 2);
  ASSERT_EQ(B->cols(), 2);
  (*B)(0, 0) = 1.0;
  (*B)(0, 1) = 2.0;
  (*B)(1, 0) = 3.0;
  (*B)(1, 1) = 4.0;

  double expected = 1.0;
  for (int r = 0; r < M.rows(); r++) {
    for (int c = 0; c < M.cols(); c++) {
      if (r >= 2 && r < 4 && c < 2) {
        // This is part of the block.
        EXPECT_NEAR(expected, M(r,c), 1e-16) << "Failure at " << r << "," << c;
        expected += 1.0;
      } else {
        ASSERT_EQ(0.0,M(r,c))<< "The uninitailzed matrix did not return zero at " << r << "," << c;
      }
    }
  }
}

TEST(sparse_block_matrixTestSuite, testClone) {
  using namespace Eigen;
  using namespace sparse_block_matrix;
  VectorXi rows(4);
  rows << 2, 4, 14, 15;
  VectorXi cols(3);
  cols << 3, 5, 7;

  // Create an empty matrix.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> ME = buildRandomMatrix<
      MatrixXd>(rows, cols, -1.0);

  sparse_block_matrix::SparseBlockMatrix<MatrixXd> * ME2 = ME.clone();
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> ME3;
  ME.cloneInto(ME3);

  {
    SCOPED_TRACE("");
    sparse_block_matrix::expectEqual(ME, ME3);
  }

  {
    SCOPED_TRACE("");
    sparse_block_matrix::expectEqual(ME, (*ME2));
  }

  delete ME2;

  // Create an not empty matrix.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M1 = buildRandomMatrix<
      MatrixXd>(rows, cols, 0.5);

  sparse_block_matrix::SparseBlockMatrix<MatrixXd> * M2 = M1.clone();
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M3;
  M1.cloneInto(M3);

  {
    SCOPED_TRACE("");
    sparse_block_matrix::expectEqual(M1, (*M2));
  }

  {
    SCOPED_TRACE("");
    sparse_block_matrix::expectEqual(M1, M3);
  }

  delete M2;
}

TEST(sparse_block_matrixTestSuite, testToDense) {
  using namespace Eigen;
  using namespace sparse_block_matrix;
  VectorXi rows(4);
  rows << 2, 4, 14, 15;
  VectorXi cols(3);
  cols << 3, 5, 7;

  // Create an not empty matrix.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M1 = buildRandomMatrix<
      MatrixXd>(rows, cols, 0.5);

  Eigen::MatrixXd D1 = M1.toDense();
  {
    SCOPED_TRACE("");
    sparse_block_matrix::expectEqual(M1, D1);
  }

}

// //! transposes a block matrix, The transposed type should match the argument false on failure
// template <class MatrixTransposedType>
// bool transpose(SparseBlockMatrix<MatrixTransposedType>*& dest) const;
TEST(sparse_block_matrixTestSuite, testTranspose) {
  using namespace Eigen;
  using namespace sparse_block_matrix;
  VectorXi rows(4);
  rows << 2, 4, 14, 15;
  VectorXi cols(3);
  cols << 3, 5, 7;

  // Create an not empty matrix.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M1 = buildRandomMatrix<
      MatrixXd>(rows, cols, 0.5);

  Eigen::MatrixXd D1t = M1.toDense().transpose();

  sparse_block_matrix::SparseBlockMatrix<MatrixXd> * M1t = NULL;
  // First the NULL version.
  ASSERT_TRUE(M1.transpose(M1t));
  ASSERT_NE((sparse_block_matrix::SparseBlockMatrix<MatrixXd> *)NULL, M1t);

  {
    SCOPED_TRACE("");
    sparse_block_matrix::expectEqual((*M1t), D1t);
  }
  {
    SCOPED_TRACE("");
    sparse_block_matrix::expectEqual(M1.transpose(), D1t);
  }

  // Now the non null version.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M2t(M1t->rowBlockIndices(),
                                                       M1t->colBlockIndices(),
                                                       true);
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> * M2tp = &M2t;
  ASSERT_TRUE(M1.transpose(M2tp));

  {
    SCOPED_TRACE("");
    sparse_block_matrix::expectEqual(M2t, D1t);
  }

  delete M1t;

  // What if there is cruff in the transpose matrix? This fails.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M3t = buildRandomMatrix<
      MatrixXd>(cols, rows, 0.5);
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> * M3tp = &M2t;
  ASSERT_TRUE(M1.transpose(M3tp));

  {
    SCOPED_TRACE("");
    // Fails.
    //sparse_block_matrix::expectEqual( M3t.toDense(), D1t);
  }


  // test with statically sized matrices
  rows << 2, 4, 6, 8;
  cols << 1, 2, 3;
  sparse_block_matrix::SparseBlockMatrix<Matrix<double, 2, 1> > M3 = buildRandomMatrix<Matrix<double, 2, 1> >(rows, cols, 0.5);
  {
    SCOPED_TRACE("");
    sm::eigen::assertEqual(M3.transpose().toDense(), M3.toDense().transpose().eval(), SM_SOURCE_FILE_POS);
  }
}

TEST(sparse_block_matrixTestSuite, testEigenVectorMultiplication) {

  using namespace Eigen;
  using namespace sparse_block_matrix;
  VectorXi rows(4);
  rows << 2, 4, 14, 15;
  VectorXi cols(3);
  cols << 3, 5, 7;

  // Create an not empty matrix.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M1 = buildRandomMatrix<
      MatrixXd>(rows, cols, 0.5);

  // a vector to multiply
  VectorXd src = VectorXd::Random(7);
  VectorXd dstp(15);
  dstp.setZero();
  // multiply
  M1.multiply(&dstp, src);

  // existing impl.
  double* dstp2 = NULL;
  //
  M1.multiply(dstp2, &src[0]);

  // make a dense multiplicaation:
  Eigen::MatrixXd M1dense = M1.toDense();
  Eigen::MatrixXd dstDense = M1dense * src;

  sm::eigen::assertNear(dstp, dstDense, 1e-6, SM_SOURCE_FILE_POS);
  Eigen::Map<Eigen::VectorXd> dstp2Map(dstp2, dstp.rows());
  sm::eigen::assertNear(dstDense, dstp2Map, 1e-6, SM_SOURCE_FILE_POS);

  double src4[15];
  double* dstp4 = new double[15];
  for (int i = 0; i < 15; i++)
    dstp4[i] = 0;
  VectorXd src3 = VectorXd::Random(15);
  VectorXd dstp3(7);
  dstp3.setZero();

  M1.rightMultiply(&dstp3, src3);

  for (int i = 0; i < src3.rows(); i++)
    src4[i] = src3(i);

  // take product:

  M1.rightMultiply(dstp4, &src4[0]);

  Eigen::MatrixXd dstDense3 = src3.transpose() * M1dense;

  Eigen::Map<Eigen::VectorXd> dstp2Map2(dstp4, dstp3.rows());

  sm::eigen::assertNear(dstp3, dstDense3.transpose(), 1e-6, SM_SOURCE_FILE_POS);
  sm::eigen::assertNear(dstDense3.transpose(), dstp2Map2, 1e-6,
                        SM_SOURCE_FILE_POS);

  delete[] dstp2;
  delete[] dstp4;

}

TEST(sparse_block_matrixTestSuite, testScaleMatrix) {

  using namespace Eigen;
  using namespace sparse_block_matrix;
  VectorXi rows(4);
  rows << 2, 4, 14, 15;
  VectorXi cols(3);
  cols << 3, 5, 7;

  // Create an not empty matrix.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M1 = buildRandomMatrix<MatrixXd>(rows, cols, 0.5);

  double scale = std::rand();

  try{
    sm::eigen::assertNear((M1 * scale).toDense(), (M1.toDense() * scale).eval(), 1e-6, SM_SOURCE_FILE_POS);
  }catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}

TEST(sparse_block_matrixTestSuite, testMatrixMatrixMultiplication) {

  using namespace Eigen;
  using namespace sparse_block_matrix;
  VectorXi rows(4);
  rows << 2, 4, 14, 15;
  VectorXi cols(3);
  cols << 3, 5, 7;

  // Create an not empty matrix.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M1 = buildRandomMatrix<MatrixXd>(rows, cols, 0.5);

  // Create an not empty matrix.
  sparse_block_matrix::SparseBlockMatrix<MatrixXd> M2 = buildRandomMatrix<MatrixXd>(cols, rows, 0.5);

  try{
    sm::eigen::assertNear((M1 * M2).toDense(), (M1.toDense() * M2.toDense()).eval(), 1e-6, SM_SOURCE_FILE_POS);
  }catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}
// //! adds the current matrix to the destination
// bool add(SparseBlockMatrix<MatrixType>*& dest) const ;

// //! dest = (*this) *  src
// void multiply(double*& dest, const double* src) const;

// //! dest = M * (*this)
// void rightMultiply(double*& dest, const double* src) const;

// SparseBlockMatrix*  slice(int rmin, int rmax, int cmin, int cmax, bool alloc=true) const;

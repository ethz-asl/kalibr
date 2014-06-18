#include <sm/eigen/gtest.hpp>
#include <aslam/backend/sparse_matrix_functions.hpp>
// std::partial_sum
#include <numeric>


TEST(SparseMatrixFunctionTests, testSchurComplement1)
{
  try {
    using namespace aslam::backend;
    typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> SparseBlockMatrix;
    // Create the sparse Hessian. Two dense blocks. Three sparse.
    int structure[5] = {2, 2, 3, 3, 3};
    std::partial_sum(structure, structure + 5, structure);
    int marginalizedStartingBlock = 2;
    int marginalizedStartingIndex = structure[ marginalizedStartingBlock - 1 ];
    double lambda = 1;
    SparseBlockMatrix H(structure, structure, 5, 5, true);
    Eigen::VectorXd e(H.rows());
    e.setRandom();
    Eigen::VectorXd b(H.rowBaseOfBlock(marginalizedStartingBlock));
    b.setZero();
    boost::shared_ptr<SparseBlockMatrix> A(H.slice(0, marginalizedStartingBlock, 0, marginalizedStartingBlock, true));
    ASSERT_EQ(marginalizedStartingBlock, A->bRows());
    ASSERT_EQ(marginalizedStartingBlock, A->bCols());
    A->clear(false);
    std::vector<Eigen::MatrixXd> invVi;
    invVi.resize(H.bRows() - marginalizedStartingBlock);
    // Fill in H.
    *H.block(0, 0, true) = sm::eigen::randomCovariance<2>() * 100;
    *H.block(1, 1, true) = sm::eigen::randomCovariance<2>() * 100;
    *H.block(2, 2, true) = sm::eigen::randomCovariance<3>() * 100;
    *H.block(3, 3, true) = sm::eigen::randomCovariance<3>() * 100;
    *H.block(4, 4, true) = sm::eigen::randomCovariance<3>() * 100;
    // Start with two off diagonals.
    H.block(0, 4, true)->setRandom();
    H.block(0, 4, true)->array() *= 100;
    H.block(1, 4, true)->setRandom();
    H.block(1, 4, true)->array() *= 100;
    //std::cout << "H:\n" << H << std::endl;
    applySchurComplement(H,
                         e,
                         lambda,
                         marginalizedStartingBlock,
                         true,
                         *A,
                         invVi,
                         b);
    Eigen::MatrixXd Hd = H.toDense();
    Eigen::MatrixXd U = Hd.topLeftCorner(marginalizedStartingIndex, marginalizedStartingIndex);
    Eigen::MatrixXd V = Hd.bottomRightCorner(H.rows() - marginalizedStartingIndex, H.rows() - marginalizedStartingIndex);
    Eigen::MatrixXd W = Hd.topRightCorner(marginalizedStartingIndex, H.rows() - marginalizedStartingIndex);
    V.diagonal().array() += lambda;
    Eigen::MatrixXd AA = U - W * V.inverse() * W.transpose();
    AA.diagonal().array() += lambda;
    Eigen::VectorXd epsSparse = e.tail(e.size() - marginalizedStartingIndex);
    Eigen::VectorXd epsDense = e.head(marginalizedStartingIndex);
    Eigen::VectorXd bb = epsDense - W * V.inverse() * epsSparse;
    {
      SCOPED_TRACE("");
      Eigen::MatrixXd Asa = A->toDense().selfadjointView<Eigen::Upper>();
      sm::eigen::assertNear(Asa, AA, 1e-12, SM_SOURCE_FILE_POS, "Testing the lhs schur complement");
    }
    {
      SCOPED_TRACE("");
      sm::eigen::assertNear(b, bb, 1e-12, SM_SOURCE_FILE_POS, "Testing the rhs schur complement");
    }
    // Let's try it again to make sure stuff gets initialized correctly.
    applySchurComplement(H,
                         e,
                         lambda,
                         marginalizedStartingBlock,
                         true,
                         *A,
                         invVi,
                         b);
    {
      SCOPED_TRACE("");
      Eigen::MatrixXd Asa = A->toDense().selfadjointView<Eigen::Upper>();
      sm::eigen::assertNear(Asa, AA, 1e-12, SM_SOURCE_FILE_POS, "Testing the lhs schur complement");
    }
    {
      SCOPED_TRACE("");
      sm::eigen::assertNear(b, bb, 1e-12, SM_SOURCE_FILE_POS, "Testing the rhs schur complement");
    }
    // Now we check the update function.
    Eigen::VectorXd dx(marginalizedStartingIndex);
    dx.setRandom();
    Eigen::VectorXd denseDs = V.inverse() * (epsSparse - W.transpose() * dx);
    for (int i = 0; i < H.bRows() - marginalizedStartingBlock; ++i) {
      Eigen::VectorXd outDsi;
      buildDsi(i, H, e, marginalizedStartingBlock, invVi[i], dx, outDsi);
      Eigen::VectorXd dsi = denseDs.segment(H.rowBaseOfBlock(i + marginalizedStartingBlock) - marginalizedStartingIndex, H.rowsOfBlock(i + marginalizedStartingBlock));
      sm::eigen::assertNear(outDsi, dsi, 1e-12, SM_SOURCE_FILE_POS, "Checking the update step calculation");
    }
  } catch (const std::exception& e) {
    FAIL() << "Exception: " << e.what();
  }
}


TEST(SparseMatrixFunctionTests, testSchurComplement2)
{
  try {
    using namespace aslam::backend;
    typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> SparseBlockMatrix;
    // Create the sparse Hessian. Two dense blocks. Three sparse.
    int structure[5] = {2, 2, 3, 3, 3};
    std::partial_sum(structure, structure + 5, structure);
    int marginalizedStartingBlock = 2;
    int marginalizedStartingIndex = structure[ marginalizedStartingBlock - 1 ];
    double lambda = 1;
    SparseBlockMatrix H(structure, structure, 5, 5, true);
    Eigen::VectorXd e(H.rows());
    e.setRandom();
    Eigen::VectorXd b(H.rowBaseOfBlock(marginalizedStartingBlock));
    b.setZero();
    boost::shared_ptr<SparseBlockMatrix> A(H.slice(0, marginalizedStartingBlock, 0, marginalizedStartingBlock, true));
    A->clear(false);
    std::vector<Eigen::MatrixXd> invVi;
    invVi.resize(H.bRows() - marginalizedStartingBlock);
    // Fill in H.
    *H.block(0, 0, true) = sm::eigen::randomCovariance<2>() * 100;
    *H.block(1, 1, true) = sm::eigen::randomCovariance<2>() * 100;
    *H.block(2, 2, true) = sm::eigen::randomCovariance<3>() * 100;
    *H.block(3, 3, true) = sm::eigen::randomCovariance<3>() * 100;
    *H.block(4, 4, true) = sm::eigen::randomCovariance<3>() * 100;
    // Start with two off diagonals.
    H.block(0, 1, true)->setRandom();
    H.block(0, 1, true)->array() *= 100;
    H.block(0, 2, true)->setRandom();
    H.block(0, 2, true)->array() *= 100;
    H.block(0, 3, true)->setRandom();
    H.block(0, 3, true)->array() *= 100;
    H.block(1, 4, true)->setRandom();
    H.block(1, 4, true)->array() *= 100;
    H.block(0, 4, true)->setRandom();
    H.block(0, 4, true)->array() *= 100;
    H.block(1, 4, true)->setRandom();
    H.block(1, 4, true)->array() *= 100;
    //std::cout << "H:\n" << H << std::endl;
    applySchurComplement(H,
                         e,
                         lambda,
                         marginalizedStartingBlock,
                         true,
                         *A,
                         invVi,
                         b);
    Eigen::MatrixXd Hd = H.toDense().selfadjointView<Eigen::Upper>();;
    Eigen::MatrixXd U = Hd.topLeftCorner(marginalizedStartingIndex, marginalizedStartingIndex);
    //U.diagonal().array() += lambda;
    Eigen::MatrixXd V = Hd.bottomRightCorner(H.rows() - marginalizedStartingIndex, H.rows() - marginalizedStartingIndex);
    V.diagonal().array() += lambda;
    Eigen::MatrixXd W = Hd.topRightCorner(marginalizedStartingIndex, H.rows() - marginalizedStartingIndex);
    Eigen::MatrixXd AA = U - W * V.inverse() * W.transpose();
    Eigen::VectorXd epsSparse = e.tail(e.size() - marginalizedStartingIndex);
    Eigen::VectorXd epsDense = e.head(marginalizedStartingIndex);
    Eigen::VectorXd bb = epsDense - W * V.inverse() * epsSparse;
    AA.diagonal().array() += lambda;
    SCOPED_TRACE("");
    Eigen::MatrixXd Asa = A->toDense().selfadjointView<Eigen::Upper>();
    sm::eigen::assertNear(Asa, AA, 1e-12, SM_SOURCE_FILE_POS, "Testing the rhs schur complement");
    SCOPED_TRACE("");
    sm::eigen::assertNear(b, bb, 1e-12, SM_SOURCE_FILE_POS, "Testing the rhs schur complement");
    // Now we check the update function.
    Eigen::VectorXd dx(Asa.rows());
    dx.setRandom();
    dx.array() *= 100;
    Eigen::VectorXd denseDs = V.inverse() * (epsSparse - W.transpose() * dx);
    for (int i = 0; i < H.bRows() - marginalizedStartingBlock; ++i) {
      Eigen::VectorXd outDsi;
      buildDsi(i, H, e, marginalizedStartingBlock, invVi[i], dx, outDsi);
      Eigen::VectorXd dsi = denseDs.segment(H.rowBaseOfBlock(i + marginalizedStartingBlock) - marginalizedStartingIndex, H.rowsOfBlock(i + marginalizedStartingBlock));
      sm::eigen::assertNear(outDsi, dsi, 1e-14, SM_SOURCE_FILE_POS, "Checking the update step calculation");
    }
  } catch (const std::exception& e) {
    FAIL() << "Exception: " << e.what();
  }
}

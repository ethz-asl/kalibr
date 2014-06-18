#include <aslam/backend/sparse_matrix_functions.hpp>
#include <aslam/Exceptions.hpp>
#include <sm/eigen/assert_macros.hpp>
namespace aslam {
  namespace backend {


    void applySchurComplement(sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>& H,
                              const Eigen::VectorXd& rhs,
                              double lambda,
                              int marginalizedStartingBlock,
                              bool doLevenbergMarquardt,
                              sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>& A,
                              std::vector<Eigen::MatrixXd>& invVi,
                              Eigen::VectorXd& b)
    {
      typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> SparseBlockMatrix;
      H.sliceInto(0, marginalizedStartingBlock, 0, marginalizedStartingBlock, A);
      SM_ASSERT_EQ_DBG(Exception, marginalizedStartingBlock, A.bRows(), "Is A the right size?");
      SM_ASSERT_EQ_DBG(Exception, marginalizedStartingBlock, A.bCols(), "Is A the right size?");
      b = rhs.head(H.rowBaseOfBlock(marginalizedStartingBlock));
      // Invert the Vs, build the Ys
      // This is the essence of the Schur Complement trick.
      for (unsigned i = 0; i < invVi.size(); i++) {
        const int blockIndex = marginalizedStartingBlock + i;
        const int rowBase_i = H.rowBaseOfBlock(blockIndex);
        const bool allocIfMissing = true;
        Eigen::MatrixXd& Vi = *H.block(blockIndex, blockIndex, allocIfMissing);
        // Offset the diagonal as per LM.
        if (doLevenbergMarquardt) {
          invVi[i] = Vi;
          invVi[i].diagonal().array() += lambda;
          invVi[i] = invVi[i].inverse().eval();
        } else {
          invVi[i] = Vi.inverse();
        }
        // Wi inv(Vi) WiT = [ Wi1 iVi ] [ Wi1T ... WinT ]
        //                  [ ...     ]
        //                  [ Win iVi ]
        //                = [ Wi1 iVi Wi1T , ... , Wi1 iVi WinT ]
        //                  [ ...            ... , ...          ]
        //                  [ Win iVi Wi1T , ... , Win iVi WinT ]
        const SparseBlockMatrix::IntBlockMap& Wij = H.blockCols()[blockIndex];
        SparseBlockMatrix::IntBlockMap::const_iterator j = Wij.begin();
        for (; j != Wij.end() && j->first < marginalizedStartingBlock; j++) {
          Eigen::MatrixXd Yij = (*j->second) * invVi[i];
          b.segment(H.rowBaseOfBlock(j->first), Yij.rows()) -= Yij * rhs.segment(rowBase_i, Yij.cols());
          SparseBlockMatrix::IntBlockMap::const_iterator k = j;
          for (; k != Wij.end() && k->first < marginalizedStartingBlock; k++) {
            (*A.block(j->first, k->first, true)) -= Yij * k->second->transpose();
          }
        }
      }
      if (doLevenbergMarquardt) {
        for (int i = 0; i < A.bRows(); ++i) {
          Eigen::MatrixXd& block = *A.block(i, i, true);
          block.diagonal().array() += lambda;
        }
      }
    }



    void buildDsi(int i,
                  sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>& H,
                  const Eigen::VectorXd& rhs, int marginalizedStartingBlock, const Eigen::MatrixXd& invVi,
                  const Eigen::VectorXd& dx, Eigen::VectorXd& outDsi)
    {
      typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> SparseBlockMatrix;
      const int blockIndex = marginalizedStartingBlock + i;
      const int rowBase_i = H.rowBaseOfBlock(blockIndex);
      const int dbd = H.rowsOfBlock(blockIndex);
      outDsi = rhs.segment(rowBase_i, dbd);
      const SparseBlockMatrix::IntBlockMap& Wij = H.blockCols()[blockIndex];
      SparseBlockMatrix::IntBlockMap::const_iterator j = Wij.begin();
      for (; j != Wij.end() && j->first < marginalizedStartingBlock; j++) {
        int row = H.rowBaseOfBlock(j->first);
        //std::cout << "J at row " << row << "\n" << *(j->second) << std::endl;
        outDsi -= j->second->transpose() * dx.segment(row, j->second->rows());
      }
      // Compute the sparse update.
      outDsi = (invVi * outDsi).eval();
    }

  } // namespace backend
} // namespace aslam

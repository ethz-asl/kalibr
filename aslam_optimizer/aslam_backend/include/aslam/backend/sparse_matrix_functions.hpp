#ifndef ASLAM_BACKEND_SPARSE_MATRIX_FUNCTIONS_HPP
#define ASLAM_BACKEND_SPARSE_MATRIX_FUNCTIONS_HPP

#include <sparse_block_matrix/linear_solver.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace aslam {
  namespace backend {

    void applySchurComplement(sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>& H,
                              const Eigen::VectorXd& e,
                              double lambda,
                              int marginalizedStartingBlock,
                              bool doLevenberg,
                              sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>& A,
                              std::vector<Eigen::MatrixXd>& invVi,
                              Eigen::VectorXd& b);

    void buildDsi(int i,
                  sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd>& H,
                  const Eigen::VectorXd& e, int marginalizedStartingBlock, const Eigen::MatrixXd& invVi,
                  const Eigen::VectorXd& dx, Eigen::VectorXd& outDsi);

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_SPARSE_MATRIX_FUNCTIONS_HPP */

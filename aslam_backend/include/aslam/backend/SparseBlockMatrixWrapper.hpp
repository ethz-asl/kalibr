#ifndef ASLAM_BACKEND_SPARSE_BLOCK_MATRIX_WRAPPER_HPP
#define ASLAM_BACKEND_SPARSE_BLOCK_MATRIX_WRAPPER_HPP

#include "Matrix.hpp"
#include <sparse_block_matrix/sparse_block_matrix.h>

namespace aslam {
  namespace backend {

    class SparseBlockMatrixWrapper : public Matrix {
    public:
      SparseBlockMatrixWrapper();
      virtual ~SparseBlockMatrixWrapper();

      /// \brief Get a value from the matrix at row r and column c
      virtual double operator()(size_t r, size_t c) const;

      /// \brief The number of rows in the matrix.
      virtual size_t rows() const;

      /// \brief The number of columns in the matrix.
      virtual size_t cols() const;

      /// \brief Fill and return a dense matrix.
      Eigen::MatrixXd toDense() const;

      /// \brief Fill the input dense matrix. The default version just
      ///        Goes through the matrix calling operator(). Please override.
      virtual void toDenseInto(Eigen::MatrixXd& outM) const;

      /// \brief Initialize the matrix from a dense matrix
      virtual void fromDense(const Eigen::MatrixXd& M);

      /// \brief Initialize the matrix from a dense matrix.
      ///        Entries with absolute value less than tolerance
      ///        should be considered zeros.
      virtual void fromDenseTolerance(const Eigen::MatrixXd& M, double tolerance);

      /// \brief right multiply the vector y = A x
      virtual void rightMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const;

      /// \brief left multiply the vector y = A^T x
      virtual void leftMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const;

      sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> _M;
    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_SPARSE_BLOCK_MATRIX_WRAPPER_HPP */

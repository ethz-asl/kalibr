#ifndef ASLAM_BACKEND_DENSE_MATRIX_HPP
#define ASLAM_BACKEND_DENSE_MATRIX_HPP

#include "Matrix.hpp"

namespace aslam {
  namespace backend {

    /// \class DenseMatrix
    /// \brief A very simple matrix wrapper.
    class DenseMatrix : public Matrix {
    public:
      DenseMatrix();

      virtual ~DenseMatrix();

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

      Eigen::MatrixXd _M;

    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_DENSE_MATRIX_HPP */

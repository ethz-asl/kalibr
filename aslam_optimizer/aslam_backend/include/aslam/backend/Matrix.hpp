#ifndef ASLAM_BACKEND_MATRIX_HPP
#define ASLAM_BACKEND_MATRIX_HPP

#include <Eigen/Core>
#include <iostream>
#include <sm/assert_macros.hpp>

namespace aslam {
  namespace backend {

    /// \class Matrix
    /// \brief A very simple matrix wrapper.
    class Matrix {
    public:
      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

      Matrix();

      virtual ~Matrix();

      /// \brief Get a value from the matrix at row r and column c
      virtual double operator()(size_t r, size_t c) const = 0;

      /// \brief The number of rows in the matrix.
      virtual size_t rows() const = 0;

      /// \brief The number of columns in the matrix.
      virtual size_t cols() const = 0;

      /// \brief Fill and return a dense matrix.
      Eigen::MatrixXd toDense() const;

      /// \brief Fill the input dense matrix. The default version just
      ///        Goes through the matrix calling operator(). Please override.
      virtual void toDenseInto(Eigen::MatrixXd& outM) const;

      /// \brief Initialize the matrix from a dense matrix
      virtual void fromDense(const Eigen::MatrixXd& M) = 0;

      /// \brief Initialize the matrix from a dense matrix.
      ///        Entries with absolute value less than tolerance
      ///        should be considered zeros.
      virtual void fromDenseTolerance(const Eigen::MatrixXd& M, double tolerance) = 0;

      /// \brief right multiply the vector y = A x
      virtual void rightMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const = 0;

      /// \brief left multiply the vector y = A^T x
      virtual void leftMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const = 0;

      /// \todo initialization from a triplet matrix.

      /// Writes to standard output
      virtual void write(std::ostream& stream) const;

    };

  std::ostream& operator<<(std::ostream& os, const aslam::backend::Matrix& m);

  } // namespace backend
} // namespace aslam



#endif /* ASLAM_BACKEND_MATRIX_HPP */

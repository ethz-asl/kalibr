#include <aslam/backend/DenseMatrix.hpp>

namespace aslam {
  namespace backend {

    DenseMatrix::DenseMatrix()
    {
    }


    DenseMatrix::~DenseMatrix()
    {
    }


    /// \brief Get a value from the matrix at row r and column c
    double DenseMatrix::operator()(size_t r, size_t c) const
    {
      return _M(r, c);
    }


    /// \brief The number of rows in the matrix.
    size_t DenseMatrix::rows() const
    {
      return _M.rows();
    }


    /// \brief The number of columns in the matrix.
    size_t DenseMatrix::cols() const
    {
      return _M.cols();
    }


    /// \brief Fill and return a dense matrix.
    Eigen::MatrixXd DenseMatrix::toDense() const
    {
      return _M;
    }


    /// \brief Fill the input dense matrix. The default version just
    ///        Goes through the matrix callingDenseMatrix::operator(). Please override.
    void DenseMatrix::toDenseInto(Eigen::MatrixXd& outM) const
    {
      outM = _M;
    }


    /// \brief Initialize the matrix from a dense matrix
    void DenseMatrix::fromDense(const Eigen::MatrixXd& M)
    {
      _M = M;
    }


    /// \brief Initialize the matrix from a dense matrix.
    ///        Entries with absolute value less than tolerance
    ///        should be considered zeros.
    void DenseMatrix::fromDenseTolerance(const Eigen::MatrixXd& M, double tolerance)
    {
      _M = M;
      for (int r = 0; r < _M.rows(); ++r) {
        for (int c = 0; c < _M.cols(); ++c) {
          if (fabs(_M(r, c)) <= tolerance)
            _M(r, c) = 0.0;
        }
      }
    }


    /// \brief right multiply the vector y = A x
    void DenseMatrix::rightMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const
    {
      outY = _M * x;
    }


    /// \brief left multiply the vector y = A^T x
    void DenseMatrix::leftMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const
    {
      outY = _M.transpose() * x;
    }



  } // namespace backend
} // namespace aslam

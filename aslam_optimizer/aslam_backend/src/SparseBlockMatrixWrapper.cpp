#include <aslam/backend/SparseBlockMatrixWrapper.hpp>

namespace aslam {
  namespace backend {

    SparseBlockMatrixWrapper::SparseBlockMatrixWrapper()
    {
    }

    SparseBlockMatrixWrapper::~SparseBlockMatrixWrapper()
    {
    }


    /// \brief The number of rows in the matrix.
    size_t SparseBlockMatrixWrapper::rows() const
    {
      return _M.rows();
    }


    /// \brief The number of columns in the matrix.
    size_t SparseBlockMatrixWrapper::cols() const
    {
      return _M.cols();
    }

    /// \brief Get a value from the matrix at row r and column c
    double SparseBlockMatrixWrapper::operator()(size_t r, size_t c) const
    {
      return _M(r, c);
    }



    /// \brief Fill the input dense matrix. The default version just
    ///        Goes through the matrix calling SparseBlockMatrixWrapper::operator(). Please override.
    void SparseBlockMatrixWrapper::toDenseInto(Eigen::MatrixXd& outM) const
    {
      _M.toDenseInto(outM);
    }


    /// \brief Initialize the matrix from a dense matrix
    void SparseBlockMatrixWrapper::fromDense(const Eigen::MatrixXd& M)
    {
      fromDenseTolerance(M, 0.0);
    }


    /// \brief Initialize the matrix from a dense matrix.
    ///        Entries with absolute value less than tolerance
    ///        should be considered zeros.
    void SparseBlockMatrixWrapper::fromDenseTolerance(const Eigen::MatrixXd& inM, double tolerance)
    {
      SM_ASSERT_EQ(Matrix::Exception, (size_t)inM.rows(), (size_t)_M.rows(), "The dense matrix is the wrong size. The SparseBlockMatrix needs an existing block structure");
      SM_ASSERT_EQ(Matrix::Exception, (size_t)inM.cols(), (size_t)_M.cols(), "The dense matrix is the wrong size. The SparseBlockMatrix needs an existing block structure");
      _M.clear(true);
      // For each block.
      for (int blockCol = 0; blockCol < _M.bCols(); ++blockCol) {
        for (int blockRow = 0; blockRow < _M.bRows(); ++blockRow) {
          Eigen::MatrixXd* B = NULL;
          int r = _M.rowBaseOfBlock(blockRow);
          int c = _M.colBaseOfBlock(blockCol);
          for (int dc = 0; dc < _M.colsOfBlock(blockCol); ++dc) {
            for (int dr = 0; dr < _M.rowsOfBlock(blockRow); ++dr) {
              if (fabs(inM(r + dr, c + dc)) > tolerance) {
                if (!B) {
                  B = _M.block(blockRow, blockCol, true);
                }
                SM_ASSERT_TRUE(Matrix::Exception, B != NULL, "Null block");
                (*B)(dr, dc) = inM(r + dr, c + dc);
              }
            }
          }
        }
      }
    }


    /// \brief right multiply the vector y = A x
    void SparseBlockMatrixWrapper::rightMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const
    {
      outY.resize(_M.rows());
      _M.multiply(&outY, x);
    }


    /// \brief left multiply the vector y = A^T x
    void SparseBlockMatrixWrapper::leftMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const
    {
      outY.resize(_M.cols());
      _M.rightMultiply(&outY, x);
    }


  } // namespace backend
} // namespace aslam

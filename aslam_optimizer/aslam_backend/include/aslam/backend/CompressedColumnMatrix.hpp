#ifndef ASLAM_BACKEND_COMPRESSED_COLUMN_MATRIX_HPP
#define ASLAM_BACKEND_COMPRESSED_COLUMN_MATRIX_HPP

#include <cs.h>
#include "Cholmod.hpp"
#include <vector>
#include <Eigen/Core>
//#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
//#include <Eigen/Sparse>
#include <sm/assert_macros.hpp>
#include <sm/string_routines.hpp>
#include <boost/cstdint.hpp>
#include "JacobianContainer.hpp"
#include "ErrorTerm.hpp"
#include <iostream>
#include "Matrix.hpp"

namespace aslam {
  namespace backend {



    struct JacobianColumnPointer {
      JacobianColumnPointer(size_t sc = 0, size_t epc = 0) : startValueIndex(sc), elementsPerColumn(epc) {}
      size_t startValueIndex;
      size_t elementsPerColumn;
    };


    template<typename INDEX_T = int>
    class CompressedColumnMatrix : public Matrix {
    public:


      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

      /// \brief the index type of the matrix
      typedef INDEX_T index_t;

      //typedef Eigen::MappedCompressedColumnMatrix<value_t> eigen_sparse_t;
      CompressedColumnMatrix();

      /// \brief initialize and set the potential number of nonzeros.
      CompressedColumnMatrix(size_t rows, size_t cols, size_t nnz, size_t num_cols);

      virtual ~CompressedColumnMatrix();

      /// \brief Return this matrix as a Cholmod sparse matrix
      void getView(cholmod_sparse* cs);

      virtual void toDenseInto(Eigen::MatrixXd& outM) const;

      /// \brief Clear all values in this matrix
      void clear();

      ///  \brief Initialize the matrix
      void init(size_t rows, size_t cols, size_t nnz, size_t num_cols);

      /// \brief return the number of rows in this matrix
      virtual size_t rows() const;

      /// \brief return the number of columns in this matrix
      virtual size_t cols() const;

      /// \brief get the element at row r and column c
      virtual double operator()(size_t r, size_t c) const;

      /// \brief get the element at row r and column c. r and c are checked for validity
      double value(size_t r, size_t c) const;

      /// \brief Get the number of non zeros
      size_t nnz() const;

      /// \brief Get the underlying values
      const std::vector<double>& values() const;

      /// \brief Get the underlying row indices
      const std::vector<index_t>& row_ind() const;

      /// \brief Get the underlying column pointers
      const std::vector<index_t>& col_ptr() const;

      /**
       * \brief A convenience function that gets the Jacobians from the
       *        error term and calls appendJacobiansSymbolic()
       *
       */
      JacobianColumnPointer appendErrorJacobiansSymbolic(const ErrorTerm& e);

      /**
       * \brief Append \f$\mathbf J^T\f$ to the right of this matrix.
       *
       * This method is amortized O(1) as the block for J^T at the right
       * is contiguous in memory and at the end of the _values array
       *
       * @param Jrows The number of rows in the Jacobian (corresponding to the number of elements in an error term)
       * @param dvs The list of design variables with non-zero Jacobians in this column.
       * @return outColumnPointer A pointer to the value array of the Jacobian matrix.
       */
      JacobianColumnPointer appendJacobiansSymbolic(int Jrows, const std::vector<DesignVariable*>& dvs);

      /// \brief Write the Jacobian values to the matrix using the pointer provided by appendJacobiansSymbolic()
      void writeJacobians(const JacobianContainer& jc, const JacobianColumnPointer& cp);

      /// \brief A convenience function that calls appendJacobiansSymbolic() and then writeJacobians()
      void appendJacobians(const JacobianContainer& jc);

      /// \brief Push a constant diagonal block on to the end of the matrix.
      void pushConstantDiagonalBlock(double constant);

      /// \brief Push a diagonal block on to the end of the matrix.
      ///        The diagonal vector must have the same number of rows as the matrix.
      void pushDiagonalBlock(const Eigen::VectorXd& diagonal);

      /// \brief Pop the diagonal block off of the matrix.
      void popDiagonalBlock();

      /// \brief update the diagonal block
      void updateDiagonalBlock(const Eigen::VectorXd& diagonal);

      /// \brief update the diagonal block with a constant value
      void updateConstantDiagonalBlock(double diagonal);

      /// \brief right multiply the vector y = A x
      void rightMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const;

      /// \brief left multiply the vector y = A^T x
      void leftMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const;


      /// \brief Initialize the matrix from a dense matrix
      virtual void fromDense(const Eigen::MatrixXd& M);

      /// \brief Initialize the matrix from a dense matrix.
      ///        Entries with absolute value less than tolerance
      ///        should be considered zeros.
      virtual void fromDenseTolerance(const Eigen::MatrixXd& M, double tolerance);

      /// Writes to standard output
      virtual void write(std::ostream& stream) const;

      /// Writes to standard stream in MATLAB format
      void writeMATLAB(std::ostream& stream) const;

      /// Initializes the matrix from a cholmod_sparse matrix
      void fromCholmodSparse(const cholmod_sparse* cs);

    private:
      void checkMatrixDbg();

      size_t _rows;
      size_t _cols;
      std::vector<double> _values;
      std::vector<index_t> _row_ind;
      std::vector<index_t> _col_ptr;

      bool _hasDiagonalAppended;
      // Keep one of these guys around to make cs views.
      cholmod_sparse _cholmodSparse;

    };

  } // namespace backend
} // namespace aslam

#include "implementation/CompressedColumnMatrix.hpp"

#endif /* ASLAM_BACKEND_COMPRESSED_COLUMN_MATRIX_HPP */

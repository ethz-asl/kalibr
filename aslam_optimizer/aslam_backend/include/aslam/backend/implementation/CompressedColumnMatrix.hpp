#include <algorithm>
#include <cmath>
#include <limits>

#define checkMatrixDbg() \
  SM_ASSERT_EQ(Exception, (size_t)_col_ptr.back(), _values.size(), "This matrix is screwed up");\
  SM_ASSERT_EQ(Exception, (size_t)_col_ptr.back(), _row_ind.size(), "This matrix is screwed up");\
  SM_ASSERT_EQ(Exception, (size_t)_col_ptr.size(), (size_t)_cols + 1, "This matrix is screwed up");



namespace aslam {
  namespace backend {



    template<typename I>
    CompressedColumnMatrix<I>::~CompressedColumnMatrix()
    {
    }

    /// \brief initialize and set the potential number of nonzeros.
    template<typename I>
    CompressedColumnMatrix<I>::CompressedColumnMatrix(size_t rows, size_t cols, size_t nnz, size_t num_cols)
    {
      init(rows, cols, nnz, num_cols);
    }

    template<typename I>
    CompressedColumnMatrix<I>::CompressedColumnMatrix()
    {
      init(0, 0, 0, 0);
    }

    template<typename I>
    void CompressedColumnMatrix<I>::init(size_t rows, size_t cols, size_t nnz, size_t num_cols)
    {
      _rows = rows;
      _cols = cols;
      _values.reserve(nnz);
      _row_ind.reserve(nnz);
      _col_ptr.reserve(num_cols);
      _col_ptr.assign(_cols + 1, (index_t)0);
      _hasDiagonalAppended = false;
    }


    template<typename I>
    void CompressedColumnMatrix<I>::getView(cholmod_sparse* cs)
    {
      if (!cs)
        return;
      // size_t nrow ;   the matrix is nrow-by-ncol
      cs->nrow = _rows;
      // size_t ncol ;
      cs->ncol = _cols;
      // size_t nzmax ;  maximum number of entries in the matrix
      cs->nzmax = nnz();
      //  pointers to int or SuiteSparse_long:
      // void *p ;     p [0..ncol], the column pointers
      cs->p = (void*)&_col_ptr[0];
      // void *i ;     i [0..nzmax-1], the row indices
      cs->i = (void*)&_row_ind[0];
      //  for unpacked matrices only:
      // void *nz ;    nz [0..ncol-1], the # of nonzeros in each col.  In
      //                   packed form, the nonzero pattern of column j is in
      //                   A->i [A->p [j] ... A->p [j+1]-1].  In unpacked form, column j is in
      //                   A->i [A->p [j] ... A->p [j]+A->nz[j]-1] instead.  In both cases, the
      //                   numerical values (if present) are in the corresponding locations in
      //                   the array x (or z if A->xtype is CHOLMOD_ZOMPLEX).
      //cs->nz = NULL;
      //  pointers to double or float:
      // void *x ;     size nzmax or 2*nzmax, if present
      cs->x = (void*)&_values[0];
      // void *z ;     size nzmax, if present
      cs->z = NULL;
      // int stype ;     Describes what parts of the matrix are considered:
      //
      //                   0:  matrix is "unsymmetric": use both upper and lower triangular parts
      //                       (the matrix may actually be symmetric in pattern and value, but
      //                       both parts are explicitly stored and used).  May be square or
      //                       rectangular.
      //                   >0: matrix is square and symmetric, use upper triangular part.
      //                       Entries in the lower triangular part are ignored.
      //                   <0: matrix is square and symmetric, use lower triangular part.
      //                       Entries in the upper triangular part are ignored.
      //
      //                   Note that stype>0 and stype<0 are different for cholmod_sparse and
      //                   cholmod_triplet.  See the cholmod_triplet data structure for more
      //                   details.
      cs->stype = 0;
      // int itype ;     CHOLMOD_INT:     p, i, and nz are int.
      //                   CHOLMOD_INTLONG: p is SuiteSparse_long,
      //                                    i and nz are int.
      //                   CHOLMOD_LONG:    p, i, and nz are SuiteSparse_long
      cs->itype = CholmodIndexTraits<I>::IType;
      // int xtype ;     pattern, real, complex, or zomplex
      cs->xtype = CholmodValueTraits<double>::XType;
      // int dtype ;     x and z are double or float
      cs->dtype = CholmodValueTraits<double>::DType;
      // int sorted ;  TRUE if columns are sorted, FALSE otherwise
      cs->sorted = 1;
      // int packed ;  TRUE if packed (nz ignored), FALSE if unpacked
      //                   (nz is required)
      cs->packed = 1;
    }


    // /// \brief Return this matrix as a sparse Eigen matrix
    // template<typename I>
    // typename CompressedColumnMatrix<I>::eigen_sparse_t CompressedColumnMatrix<I>::asEigenMatrix()
    // {
    //     // \todo
    //     return Eigen::MappedCompressedColumnMatrix<value_t>();
    // }

    template<typename I>
    void CompressedColumnMatrix<I>::toDenseInto(Eigen::MatrixXd& outM) const
    {
      outM.resize(_rows, _cols);
      outM.setZero();
      for (size_t c = 0; c < _col_ptr.size() - 1; ++c) {
        for (I v = _col_ptr[c]; v < _col_ptr[c + 1]; ++v) {
          outM(_row_ind[v], c) = _values[v];
        }
      }
    }


    /// \brief Clear all values in this matrix
    template<typename I>
    void CompressedColumnMatrix<I>::clear()
    {
      _values.clear();
      _row_ind.clear();
      _col_ptr.assign(_rows + 1, (index_t)0);
    }


    /// \brief return the number of rows in this matrix
    template<typename I>
    size_t CompressedColumnMatrix<I>::rows() const
    {
      return _rows;
    }


    /// \brief return the number of columns in this matrix
    template<typename I>
    size_t CompressedColumnMatrix<I>::cols() const
    {
      return _cols;
    }


    /// \brief get the element at row r and column c
    template<typename I>
    double CompressedColumnMatrix<I>::value(size_t r, size_t c) const
    {
      SM_ASSERT_LT(Exception, r, _rows, "Index out of bounds");
      SM_ASSERT_LT(Exception, c, _cols, "Index out of bounds");
      return (*this)(r, c);
    }


    /// \brief get the element at row r and column c
    template<typename I>
    double CompressedColumnMatrix<I>::operator()(size_t r, size_t c) const
    {
      SM_ASSERT_LT_DBG(Exception, r, _rows, "Index out of bounds");
      SM_ASSERT_LT_DBG(Exception, c, _cols, "Index out of bounds");
      // Search for the element in the array.
      // The column is sorted so we can use a binary search.
      const I* colBegin = &_row_ind[_col_ptr[c]];
      const I* colEnd = &_row_ind[_col_ptr[c + 1]];
      const I* result = std::lower_bound(colBegin, colEnd, r);
      // Found if
      if (result != colEnd && // (a) the search did not go off the end, and
          *result == (I)r) {  // (b) we found the exact row we were looking for
        // Here, the result was found to be at the parallel spot in the values array
        // Use pointer arithmetic to recover.
        return _values[result - &_row_ind[0]];
      } else {
        return 0.0;
      }
    }


    template<typename I>
    size_t CompressedColumnMatrix<I>::nnz() const
    {
      return _values.size();
    }

    /// \brief Get the underlying values
    template<typename I>
    const std::vector<double>& CompressedColumnMatrix<I>::values() const
    {
      return _values;
    }

    /// \brief Get the underlying row indices
    template<typename I>
    const std::vector<I>& CompressedColumnMatrix<I>::row_ind() const
    {
      return _row_ind;
    }

    /// \brief Get the underlying column pointers
    template<typename I>
    const std::vector<I>& CompressedColumnMatrix<I>::col_ptr() const
    {
      return _col_ptr;
    }

    template<typename I>
    void CompressedColumnMatrix<I>::appendJacobians(const JacobianContainer& jc)
    {
      std::vector<DesignVariable*> dvs;
      JacobianContainer::map_t::const_iterator it = jc.begin();
      for (; it != jc.end(); ++it) {
        dvs.push_back(it->first);
      }
      JacobianColumnPointer jcp = appendJacobiansSymbolic(jc.rows(), dvs);
      writeJacobians(jc, jcp);
    }

    template<typename I>
    JacobianColumnPointer CompressedColumnMatrix<I>::appendErrorJacobiansSymbolic(const ErrorTerm& e)
    {
      return appendJacobiansSymbolic(e.dimension(), e.designVariables());
    }

    /// \brief Adds the Jacobians to the right of this matrix
    template<typename I>
    JacobianColumnPointer CompressedColumnMatrix<I>::appendJacobiansSymbolic(int Jrows, const std::vector<DesignVariable*>& dvs)
    {
      SM_ASSERT_FALSE(Exception, _hasDiagonalAppended, "Adding more values after appending a diagonal is unsupported");
      // Build a list, sorted by block index, of the block indices and block sizes.
      std::vector<DesignVariable*> activeDvs;
      activeDvs.reserve(dvs.size());
      // The number of new elements we are adding per column.
      int elementsPerColumn = 0;
      for (size_t i = 0; i < dvs.size(); ++i) {
        DesignVariable* dv = dvs[i];
        if (dv->isActive()) {
          SM_ASSERT_GE_DBG(Exception, dv->blockIndex(), 0, "This design variable (" << i << ") is active but does not have a block index!");
          activeDvs.push_back(dv);
          elementsPerColumn += dv->minimalDimensions();
        }
      }
      //std::cout << "Elements per column: " << elementsPerColumn << ", rows: " << Jrows << std::endl;
      SM_ASSERT_FALSE_DBG(Exception, activeDvs.empty(), "There are no active design variables associated with this error term.");
      // Sort the indices by block index.
      std::sort(activeDvs.begin(), activeDvs.end(), DesignVariable::BlockIndexOrdering());
      size_t startValueIndex = _row_ind.size();
      _row_ind.resize(_row_ind.size() + elementsPerColumn * Jrows);
      _values.resize(_values.size() + elementsPerColumn * Jrows);
      size_t startColIdx = _col_ptr.size();
      _col_ptr.resize(_col_ptr.size() + Jrows);
      for (int r = 0; r < Jrows; r++) {
        //SM_ASSERT_GE_LT_DBG(Exception, (int)startColIdx + r, 0, (int)_col_ptr.size(), "Index out of bounds");
        _col_ptr[startColIdx + r] = _col_ptr[startColIdx + r - 1] + elementsPerColumn;
      }
      int rowOffset = 0;
      for (std::vector<DesignVariable*>::const_iterator it = activeDvs.begin(); it != activeDvs.end(); ++it) {
        // This is the first column of block index j
        const DesignVariable& dv = *(*it);
        for (int c = 0; c < Jrows; ++c) {
          int valueIndex = startValueIndex + c * elementsPerColumn + rowOffset;
          for (int r = 0; r < dv.minimalDimensions(); ++r) {
            SM_ASSERT_LT_DBG(Exception, (int)(dv.columnBase() + r), (int)_rows, "This element is outside of the matrix bounds");
            //SM_ASSERT_GE_LT_DBG(Exception, valueIndex, 0, (int)_row_ind.size(), "index out of bounds");
            // Plug the values of J^T into the sparse matrix.
            _row_ind[valueIndex] = dv.columnBase() + r;
            ++valueIndex;
          }
        }
        rowOffset += dv.minimalDimensions();
      }
      // Good. We have updated the three elements of this matrix and it should be fine.
      _cols = _cols + Jrows;
      checkMatrixDbg();
      return JacobianColumnPointer(startValueIndex, elementsPerColumn);
    }

    template<typename I>
    void CompressedColumnMatrix<I>::writeJacobians(const JacobianContainer& jc, const JacobianColumnPointer& cp)
    {
      JacobianContainer::map_t::const_iterator it = jc.begin();
      int rowOffset = 0;
      for (; it != jc.end(); ++it) {
        for (int c = 0; c < it->second.rows(); ++c) {
          int ind = cp.startValueIndex + c * cp.elementsPerColumn + rowOffset;
          //SM_ASSERT_GE_LT_DBG(Exception, ind, 0, (int)_values.size(), "Index out of bounds");
          //SM_ASSERT_LE_DBG(Exception, ind + it->second.cols(), (int)_values.size(), "Index out of bounds");
          double* vp = &_values[ind];
          for (int r = 0; r < it->second.cols(); ++r) {
            *(vp++) = it->second(c, r);
          }
        }
        rowOffset += it->second.cols();
      }
    }


    template<typename I>
    void CompressedColumnMatrix<I>::pushConstantDiagonalBlock(double constant)
    {
      pushDiagonalBlock(Eigen::VectorXd::Constant(_rows, constant));
    }

    template<typename I>
    void CompressedColumnMatrix<I>::pushDiagonalBlock(const Eigen::VectorXd& diagonal)
    {
      checkMatrixDbg();
      SM_ASSERT_EQ(Exception, (size_t)diagonal.size(), _rows, "The diagonal vector must match the number of rows");
      SM_ASSERT_FALSE(Exception, _hasDiagonalAppended, "Adding a second diagonal is unsupported");
      // Resize all internal vectors
      size_t vsize = _values.size();
      _values.resize(vsize + _rows);
      _row_ind.resize(vsize + _rows);
      size_t csize = _col_ptr.size();
      _col_ptr.resize(csize + _rows);
      // Adjust the matrix size.
      _cols += _rows;
      // Set up the row indices and column pointers.
      size_t cp = _col_ptr[csize - 1];
      for (unsigned i = 0; i < _rows; ++i) {
        SM_ASSERT_LT(Exception, vsize + i, _row_ind.size(), "Bad");
        SM_ASSERT_LT(Exception, csize + i, _col_ptr.size(), "Bad");
        _row_ind[vsize + i] = i;
        _col_ptr[csize + i] = ++cp;
      }
      checkMatrixDbg();
      // Copy over the values.
      memcpy(&_values[vsize], &diagonal[0], sizeof(double)*_rows);
      _hasDiagonalAppended = true;
      checkMatrixDbg();
    }

    template<typename I>
    void CompressedColumnMatrix<I>::popDiagonalBlock()
    {
      SM_ASSERT_TRUE(Exception, _hasDiagonalAppended, "No diagonal is appended.");
      _values.resize(_values.size() - _rows);
      _row_ind.resize(_row_ind.size() - _rows);
      _col_ptr.resize(_col_ptr.size() - _rows);
      _cols -= _rows;
      _hasDiagonalAppended = false;
      checkMatrixDbg();
    }


    /// \brief update the diagonal block
    template<typename I>
    void CompressedColumnMatrix<I>::updateDiagonalBlock(const Eigen::VectorXd& diagonal)
    {
      SM_ASSERT_TRUE(Exception, _hasDiagonalAppended, "No diagonal is appended.");
      Eigen::Map<Eigen::VectorXd> diag(&_values[_values.size() - _rows], _rows);
      diag = diagonal;
    }

    /// \brief update the diagonal block with a constant value
    template<typename I>
    void CompressedColumnMatrix<I>::updateConstantDiagonalBlock(double diagonal)
    {
      SM_ASSERT_TRUE(Exception, _hasDiagonalAppended, "No diagonal is appended.");
      Eigen::Map<Eigen::VectorXd> diag(&_values[_values.size() - _rows], _rows);
      diag = Eigen::VectorXd::Constant(_rows, diagonal);
    }



    template<typename I>
    void CompressedColumnMatrix<I>::rightMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const
    {
      size_t cols = _hasDiagonalAppended ? _cols - _rows : _cols;
      SM_ASSERT_EQ(Exception, (size_t)x.size(), cols, "The input array is the wrong size");
      outY.resize(_rows);
      outY.setZero();
      for (size_t c = 0; c < cols; ++c) {
        for (I idx = _col_ptr[c]; idx < _col_ptr[c + 1]; ++idx) {
          outY[_row_ind[idx]] += _values[idx] * x[c];
        }
      }
    }




    template<typename I>
    void CompressedColumnMatrix<I>::leftMultiply(const Eigen::VectorXd& x, Eigen::VectorXd& outY) const
    {
      size_t cols = _hasDiagonalAppended ? _cols - _rows : _cols;
      SM_ASSERT_EQ(Exception, (size_t)x.size(), _rows, "The input array is the wrong size");
      outY.resize(cols);
      outY.setZero();
      for (size_t c = 0; c < cols; ++c) {
        for (I idx = _col_ptr[c]; idx < _col_ptr[c + 1]; ++idx) {
          outY[c] += _values[idx] * x[_row_ind[idx]];
        }
      }
    }



    template<typename I>
    void CompressedColumnMatrix<I>:: fromDense(const Eigen::MatrixXd& M)
    {
      fromDenseTolerance(M, 0.0);
    }

    template<typename I>
    void CompressedColumnMatrix<I>:: fromDenseTolerance(const Eigen::MatrixXd& M, double tolerance)
    {
      _rows = M.rows();
      _cols = M.cols();
      _values.clear();
      _row_ind.clear();
      _col_ptr.resize(1);
      _col_ptr[0] = 0;
      tolerance = fabs(tolerance);
      for (size_t c = 0; c < _cols; ++c) {
        for (size_t r = 0; r < _rows; ++r) {
          if (fabs(M(r, c)) > tolerance) {
            _values.push_back(M(r, c));
            _row_ind.push_back(r);
          }
        }
        _col_ptr.push_back(_values.size());
      }
      checkMatrixDbg();
    }

    template<typename I>
    void CompressedColumnMatrix<I>::write(std::ostream& stream) const {
      stream << "M = " << rows() << ", N = " << cols() << std::endl;
      stream << "values(" << values().size() << "):\n";
      sm::toStream(stream, values().begin(), values().end(), ",", "[", "]");
      stream << "\nrow_ind(" << row_ind().size() << "):\n";
      sm::toStream(stream, row_ind().begin(), row_ind().end(), ",", "[", "]");
      stream << "\ncol_ptr(" << col_ptr().size() << "):\n";
      sm::toStream(stream, col_ptr().begin(), col_ptr().end(), ",", "[", "]");
      stream << std::endl;
      Matrix::write(stream);
    }

    template<typename I>
    void CompressedColumnMatrix<I>::writeMATLAB(std::ostream& stream) const {
      for (size_t j = 0; j < _cols; ++j)
        for (I p = _col_ptr[j]; p < _col_ptr[j + 1]; ++p)
          if (std::fabs(_values[p]) > std::numeric_limits<double>::epsilon())
            stream << std::fixed << std::setprecision(18) <<
              _row_ind[p] + 1 << " " << j + 1 << " " << _values[p] << std::endl;
    }

    template<typename I>
    void CompressedColumnMatrix<I>::
        fromCholmodSparse(const cholmod_sparse* cs) {
      if (cs == NULL)
        return;
      _rows = cs->nrow;
      _cols = cs->ncol;
      const I* row_ind = reinterpret_cast<const I*>(cs->i);
      const I* col_ptr = reinterpret_cast<const I*>(cs->p);
      const double* values = reinterpret_cast<const double*>(cs->x);
      const size_t nzmax = cs->nzmax;
      _col_ptr.resize(_cols + 1);
      std::copy(col_ptr, col_ptr + _cols + 1, _col_ptr.begin());
      _row_ind.resize(nzmax);
      std::copy(row_ind, row_ind + nzmax, _row_ind.begin());
      _values.resize(nzmax);
      std::copy(values, values + nzmax, _values.begin());
      checkMatrixDbg();
    }

  } // namespace backend
} // namespace aslam

// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef __SPARSE_BLOCK_MATRIX__
#define __SPARSE_BLOCK_MATRIX__

#include <map>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <Eigen/Core>

#include "matrix_structure.h"
#include <sm/assert_macros.hpp>
#include <boost/algorithm/minmax.hpp>
#include "sparse_helper.h"

namespace sparse_block_matrix {
  using namespace Eigen;
/**
 * \brief Sparse matrix which using blocks
 *
 * Template class that specifies a sparse block matrix.  A block
 * matrix is a sparse matrix made of dense blocks.  These blocks
 * cannot have a random pattern, but follow a (variable) grid
 * structure. This structure is specified by a partition of the rows
 * and the columns of the matrix.  The blocks are represented by the
 * Eigen::Matrix structure, thus they can be statically or dynamically
 * allocated. For efficiency reasons it is convenient to allocate them
 * statically, when possible. A static block matrix has all blocks of
 * the same size, and the size of rhe block is specified by the
 * template argument.  If this is not the case, and you have different
 * block sizes than you have to use a dynamic-block matrix (default
 * template argument).  
 */
template <class MatrixType = MatrixXd >
class SparseBlockMatrix {
 public:
  //! this is the type of the elementary block, it is an Eigen::Matrix.
  typedef MatrixType SparseMatrixBlock;
  //! this is the type of the elementary block, it is an Eigen::Matrix.
  typedef MatrixType Block;
  //! this is the scalar type of the matrix entries.
  typedef typename MatrixType::Scalar Scalar;
  
  typedef typename std::remove_const<decltype(MatrixType().transpose().eval())>::type TransposedMatrixType;

  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
  SM_DEFINE_EXCEPTION(IndexException, Exception);

  //! columns of the matrix
  inline int cols() const {return _colBlockIndices.size() ? _colBlockIndices.back() : 0;}
  //! rows of the matrix
  inline int rows() const {return _rowBlockIndices.size() ? _rowBlockIndices.back() : 0;}
  
  //! block columns of the matrix
  inline int bCols() const {return _colBlockIndices.size();}
  //! block rows of the matrix
  inline int bRows() const {return _rowBlockIndices.size();}

  bool isBlockSet(int br, int bc) const { return block(br,bc) != NULL; }

  
  Scalar operator()(int r, int c) const;

  //! A map from block row index to a matrix
  typedef std::map<int, SparseMatrixBlock*> IntBlockMap;

    /**
     * constructs a sparse block matrix having a specific layout
     * @param rbi: array of int containing the row layout of the blocks. 
     * the component i of the array should contain the index of the first row of the block i+1.
     * @param cbi: array of int containing the column layout of the blocks. 
     *  the component i of the array should contain the index of the first col of the block i+1.
     * @param rb: number of row blocks
     * @param cb: number of col blocks
     * @param hasStorage: set it to true if the matrix "owns" the blocks, thus it deletes it on destruction.
     * if false the matrix is only a "view" over an existing structure.
     */
  SparseBlockMatrix( const int * rbi, const int* cbi, int rb, int cb, bool hasStorage=true);

  // This guy is handy for the python interface.
  SparseBlockMatrix( const Eigen::VectorXi & rbi, const Eigen::VectorXi & cbi, bool hasStorage=true);

 
  SparseBlockMatrix( const std::vector<int> & rbi, const std::vector<int> & cbi, bool hasStorage=true);

  // a copy constructor
  SparseBlockMatrix(const SparseBlockMatrix& source);

  // move constructor
  SparseBlockMatrix(SparseBlockMatrix&& source);

  SparseBlockMatrix();

  ~SparseBlockMatrix();

    
  //! this zeroes all the blocks. If dealloc=true the blocks are removed from memory
  void clear(bool dealloc=false);

  //! returns the block at location r,c. if alloc=true he block is created if it does not exist and the values are set to 0
  SparseMatrixBlock* block(int r, int c, bool alloc=false);

  //! returns the block at location r,c
  const SparseMatrixBlock* block(int r, int c) const;

  //! how many rows does the block at block-row r have?
  inline int rowsOfBlock(int r) const { return r ? _rowBlockIndices[r] - _rowBlockIndices[r-1] : _rowBlockIndices[0] ; }

  //! how many cols does the block at block-col c have?
  inline int colsOfBlock(int c) const { return c ? _colBlockIndices[c] - _colBlockIndices[c-1] : _colBlockIndices[0]; }

  //! where does the row at block-row r starts?
  inline int rowBaseOfBlock(int r) const { return r ? _rowBlockIndices[r-1] : 0 ; }

  //! where does the col at block-col r starts?
  inline int colBaseOfBlock(int c) const { return c ? _colBlockIndices[c-1] : 0 ; }

  //! number of non-zero elements
  size_t nonZeros() const;

  //! number of allocated blocks
  size_t nonZeroBlocks() const;

  //! deep copy of a sparse-block-matrix;
  SparseBlockMatrix* clone() const ;
  void cloneInto(SparseBlockMatrix & destination) const ;
  //! assign operator
  inline SparseBlockMatrix & operator= (const SparseBlockMatrix & source){ source.cloneInto(*this); return *this; }
  //! move assign operator
  SparseBlockMatrix & operator= (SparseBlockMatrix&& source);



  Eigen::MatrixXd toDense() const;
  void toDenseInto(Eigen::MatrixXd & M) const;

  /**
   * returns a view or a copy of the block matrix
   * @param rmin: starting block row
   * @param rmax: ending  block row
   * @param cmin: starting block col
   * @param cmax: ending  block col
   * @param alloc: if true it makes a deep copy, if false it creates a view.
   */
  SparseBlockMatrix*  slice(int rmin, int rmax, int cmin, int cmax, bool alloc=true) const;

  void sliceInto(int rmin, int rmax, int cmin, int cmax, SparseBlockMatrix & outMatrix) const;

  //! transposes a block matrix, The transposed type should match the argument false on failure
  template <class MatrixTransposedType>
  bool transpose(SparseBlockMatrix<MatrixTransposedType>*& dest) const;
  //! return transposed sparse matrix
  inline SparseBlockMatrix<TransposedMatrixType> transpose() const;

  //! adds the current matrix to the destination
  bool add(SparseBlockMatrix<MatrixType>*& dest) const ;

  //! dest = (*this) *  M
  template <class MatrixResultType, class MatrixFactorType>
  bool multiply(SparseBlockMatrix<MatrixResultType> *& dest, const SparseBlockMatrix<MatrixFactorType>* M) const;
  //multiply via operator
  template <class MatrixFactorType>
  inline SparseBlockMatrix<Eigen::MatrixXd> operator * (const SparseBlockMatrix<MatrixFactorType> & M) const;

  //! dest = (*this) *  src // returns dense
  void multiply(double*& dest, const double* src) const;

  //! dest = (*this) * src (Eigen::Vector) // returns dense
  void multiply(Eigen::VectorXd * dest, const Eigen::VectorXd &src) const;

  void rightMultiply(VectorXd * dest, const VectorXd & src) const;

  //! dest = M * (*this)
  void rightMultiply(double*& dest, const double* src) const;

  //! *this *= a
  void scale( double a);
  //scale inplace operator
  inline SparseBlockMatrix & operator *= (double a) { scale(a); return *this; }
  //copy and scale as operator
  inline SparseBlockMatrix operator * (double a) const { SparseBlockMatrix ret(*this); ret.scale(a); return ret; }

  /**
   * writes in dest a block permutaton specified by pinv.
   * @param pinv: array such that new_block[i] = old_block[pinv[i]]
   */
  bool symmPermutation(SparseBlockMatrix<MatrixType>*& dest, const int* pinv, bool onlyUpper=false) const;

  /**
   * fill the CCS arrays of a matrix, arrays have to be allocated beforehand
   */

  template<typename IntType>
  IntType fillCCS(IntType* Cp, IntType* Ci, double* Cx, bool upperTriangle = false) const;

  // creates a full matrix given an upper triangle sparse block matrix
  template<typename IntType>
  IntType fillUpperTriangleCCS(IntType* Cp, IntType* Ci, double* Cx) const;
  template<typename IntType>
  IntType fillUpperTriangleCCS(double* Cx) const;
  /**
   * fill the CCS arrays of a matrix, arrays have to be allocated beforehand. This function only writes
   * the values and assumes that column and row structures have already been written.
   */
  template<typename IntType>
  IntType fillCCS(double* Cx, bool upperTriangle = false) const;

  //! exports the non zero blocks in the structure matrix ms
  void fillBlockStructure(MatrixStructure& ms) const;

  //! the block matrices per block-column
  const std::vector<IntBlockMap>& blockCols() const { return _blockCols;}
  std::vector<IntBlockMap>& blockCols() { return _blockCols;}

  //! indices of the row blocks
  const std::vector<int>& rowBlockIndices() const { return _rowBlockIndices;}
  std::vector<int>& rowBlockIndices() { return _rowBlockIndices;}

  //! indices of the column blocks
  const std::vector<int>& colBlockIndices() const { return _colBlockIndices;}
  std::vector<int>& colBlockIndices() { return _colBlockIndices;}

  /**
   * write the content of this matrix to a stream loadable by Octave
   * @param upperTriangle does this matrix store only the upper triangular blocks
   */
  bool writeOctave(const char* filename, bool upperTriangle = true) const;

  //! Returns just a reference to *this (at the moment). But useful already to have more interface compatibility with Eigen dense matrices.
  inline SparseBlockMatrix & eval() { return *this; }

 protected:
  std::vector<int> _rowBlockIndices; ///< vector of the indices of the blocks along the rows.
  std::vector<int> _colBlockIndices; ///< vector of the indices of the blocks along the cols
  //! array of maps of blocks. The index of the array represent a block column of the matrix
  //! and the block column is stored as a map row_block -> matrix_block_ptr.
  std::vector <IntBlockMap> _blockCols;
  bool _hasStorage;

  template <typename M> friend class SparseBlockMatrix;
};

template < class  MatrixType >
std::ostream& operator << (std::ostream&, const SparseBlockMatrix<MatrixType>& m);

  typedef SparseBlockMatrix<MatrixXd> SparseBlockMatrixXd;   

} //end namespace

#include "implementation/sparse_block_matrix.hpp"

#endif

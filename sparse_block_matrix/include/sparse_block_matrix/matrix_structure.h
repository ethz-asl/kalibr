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

#ifndef SBM_MATRIX_STRUCTURE_H
#define SBM_MATRIX_STRUCTURE_H

namespace sparse_block_matrix {

/**
 * \brief representing the structure of a matrix in column compressed structure (only the upper triangular part of the matrix)
 */
class MatrixStructure
{
  public:
    MatrixStructure();
    ~MatrixStructure();
    /**
     * allocate space for the Matrix Structure. You may call this on an already allocated struct, it will
     * then reallocate the memory + additional space (double the required space).
     */
    void alloc(int n_, int nz);

    void free();
    
    /**
     * Write the matrix pattern to a file. File is also loadable by octave, e.g., then use spy(matrix)
     */
    bool write(const char* filename) const;

    int n;    ///< A is m-by-n.  n must be >= 0.
    int m;    ///< A is m-by-n.  m must be >= 0.
    int* Ap;  ///< column pointers for A, of size n+1
    int* Aii; ///< row indices of A, of size nz = Ap [n]

    //! max number of non-zeros blocks
    int nzMax() const { return maxNz;}

  protected:
    int maxN;     ///< size of the allocated memory
    int maxNz;    ///< size of the allocated memory
};

} // end namespace

#endif

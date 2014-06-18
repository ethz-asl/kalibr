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

#ifndef SBM_LINEAR_SOLVER_H
#define SBM_LINEAR_SOLVER_H
#include "sparse_block_matrix.h"

namespace sparse_block_matrix {

/**
 * \brief basic solver for Ax = b
 *
 * basic solver for Ax = b which has to reimplemented for different linear algebra libraries.
 * A is assumed to be symmetric (only upper triangular block is stored) and positive-semi-definit.
 */
template <typename MatrixType>
class LinearSolver
{
  public:
  typedef MatrixType matrix_t; 

    LinearSolver() {};
    virtual ~LinearSolver() {}

    /**
     * init for operating on matrices with a different non-zero pattern like before
     */
    virtual bool init() = 0;

    /**
     * Assumes that A has the same non-zero pattern over several calls.
     * If the pattern changes call init() before.
     * solve system Ax = b, x and b have to allocated beforehand!!
     */
    virtual bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) = 0;

    /**
     * Inverts the diagonal blocks of A
     * @returns false if not defined.
     */
    virtual bool solveBlocks(double**&blocks, const SparseBlockMatrix<MatrixType>& A) { (void)blocks; (void) A; return false; }


    /**
     * Inverts the a block pattern of A in spinv
     * @returns false if not defined.
     */
    virtual bool solvePattern(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices, const SparseBlockMatrix<MatrixType>& A){
      (void) spinv;
      (void) blockIndices;
      (void) A;
      return false;
    }
};

} // end namespace

#endif

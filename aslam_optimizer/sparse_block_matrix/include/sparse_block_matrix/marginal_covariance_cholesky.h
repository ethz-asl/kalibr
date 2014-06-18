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

#ifndef SBM_MARGINAL_COVARIANCE_CHOLESKY_H
#define SBM_MARGINAL_COVARIANCE_CHOLESKY_H

//#include "optimizable_graph.h"
#include "sparse_block_matrix.h"

#include <cassert>
#include <vector>

#include <unordered_map>

namespace sparse_block_matrix {

  /**
   * \brief computing the marginal covariance given a cholesky factor (lower triangle of the factor)
   */
  class MarginalCovarianceCholesky {
    protected:
      /**
       * hash struct for storing the matrix elements needed to compute the covariance
       */
      typedef std::unordered_map<int, double>     LookupMap;
    
    public:
      MarginalCovarianceCholesky();
      ~MarginalCovarianceCholesky();

      /**
       * compute the covariance for the given vertices, directly store inside the uncertainty field of the vertex
       */
      //void computeCovariance(const std::vector<OptimizableGraph::Vertex*>& vertices);

      /**
       * compute the marginal cov for the given block indices, write the result to the covBlocks memory (which has to
       * be provided by the caller).
       */
      void computeCovariance(double** covBlocks, const std::vector<int>& blockIndices);


      /**
       * compute the marginal cov for the given block indices, write the result in spinv).
       */
      void computeCovariance(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<int>& rowBlockIndices, const std::vector< std::pair<int, int> >& blockIndices);


      /**
       * set the CCS representation of the cholesky factor along with the inverse permutation used to reduce the fill-in.
       * permInv might be 0, will then not permute the entries.
       */
      void setCholeskyFactor(int n, int* Lp, int* Li, double* Lx, int* permInv);

    protected:
      // information about the cholesky factor (lower triangle)
      int _n;           ///< L is an n X n matrix
      int* _Ap;         ///< column pointer of the CCS storage
      int* _Ai;         ///< row indices of the CCS storage
      double* _Ax;      ///< values of the cholesky factor
      int* _perm;       ///< permutation of the cholesky factor. Variable re-ordering for better fill-in

      LookupMap _map;             ///< hash look up table for the already computed entries
      std::vector<double> _diag;  ///< cache 1 / H_ii to avoid recalculations

      //! compute the index used for hashing
      int computeIndex(int r, int c) const { /*assert(r <= c);*/ return r*_n + c;}
      /**
       * compute one entry in the covariance, r and c are values after applying the permutation, and upper triangular.
       * May issue recursive calls to itself to compute the missing values.
       */
      double computeEntry(int r, int c);
  };

}

#endif

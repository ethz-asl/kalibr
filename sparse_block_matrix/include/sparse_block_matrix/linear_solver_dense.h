// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef SBM_LINEAR_SOLVER_DENSE_H
#define SBM_LINEAR_SOLVER_DENSE_H

#include <sparse_block_matrix/linear_solver.h>
//#include "g2o/core/batch_stats.h"

#include <vector>
#include <utility>
#include<Eigen/Core>
#include<Eigen/Cholesky>


namespace sparse_block_matrix {

  template <typename MatrixType>
  class LinearSolverDense : public LinearSolver<MatrixType>
  {
    public:
      LinearSolverDense() :
      LinearSolver<MatrixType>()
      {


      }

      virtual ~LinearSolverDense()
      {
      }

      virtual bool init()
      {

        return true;
      }

      bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b)
      {

        int n = A.cols();
        int m = A.cols();

        MatrixXd H(n,m);
        H.setZero();


        int c_idx = 0;


        for (size_t i = 0; i < A.blockCols().size(); ++i)
        {
          int c_size = A.colsOfBlock(i);

          const typename SparseBlockMatrix<MatrixType>::IntBlockMap& col
              = A.blockCols()[i];
          if (col.size() > 0)
          {
            typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it;
            for (it = col.begin(); it != col.end(); ++it)
            {

              if (it->first <= (int)i)  // only the upper triangular block is needed
		{
                int r_size = A.rowsOfBlock(it->first);
                H.block(A.rowBaseOfBlock(it->first),c_idx,r_size,c_size)
                    = *(it->second);

              }
            }
          }


          c_idx += c_size;
        }

        // std::cerr <<"H:\n" << H << std::endl;

        Eigen::Map<Eigen::VectorXd> xvec(x, m);
        const Eigen::Map<Eigen::VectorXd> bvec(b, n);

        // std::cerr << bvec << std::endl;
#ifdef __APPLE__
        /// THe apple compiler is buggy...crap.
        Eigen::VectorXd mm = H.selfadjointView<Eigen::Upper>().ldlt().solve(Eigen::VectorXd(bvec));
        for(int i = 0; i < mm.size(); ++i) {
            xvec[i] = mm[i];
        }
#else
        xvec = H.selfadjointView<Eigen::Upper>().ldlt().solve(Eigen::VectorXd(bvec));
#endif
        // std::cerr << xvec << std::endl;
        
        return true;
      }



    protected:

  };


}// end namespace

#endif

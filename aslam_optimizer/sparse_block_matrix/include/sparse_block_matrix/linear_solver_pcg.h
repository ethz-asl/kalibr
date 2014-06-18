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

#ifndef LINEAR_SOLVER_PCG_H
#define LINEAR_SOLVER_PCG_H

#include "linear_solver.h"

#include <vector>
#include <utility>
#include<Eigen/Core>
//#ifndef EIGEN_USE_NEW_STDVECTOR
//#define EIGEN_USE_NEW_STDVECTOR
//#endif
#include<Eigen/StdVector>

namespace sparse_block_matrix {

  /**
   * \brief linear solver using PCG, pre-conditioner is block Jacobi
   */
  template <typename MatrixType>
  class LinearSolverPCG : public LinearSolver<MatrixType>
  {
    public:
      LinearSolverPCG() :
      LinearSolver<MatrixType>()
      {
        _tolerance = 1e-6;
        _verbose = false;
        _absoluteTolerance = true;
        _residual = -1.0;
        _maxIter = -1;
      }

      virtual ~LinearSolverPCG()
      {
      }

      virtual bool init()
      {
        _residual = -1.0;
        _indices.clear();
        _sparseMat.clear();
        return true;
      }

      bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b);

      //! return the tolerance for terminating PCG before convergence
      double tolerance() const { return _tolerance;}
      void setTolerance(double tolerance) { _tolerance = tolerance;}

      int maxIterations() const { return _maxIter;}
      void setMaxIterations(int maxIter) { _maxIter = maxIter;}

      bool absoluteTolerance() const { return _absoluteTolerance;}
      void setAbsoluteTolerance(bool absoluteTolerance) { _absoluteTolerance = absoluteTolerance;}

      bool verbose() const { return _verbose;}
      void setVerbose(bool verbose) { _verbose = verbose;}

    protected:
      typedef std::vector< MatrixType, Eigen::aligned_allocator<MatrixType> > MatrixVector;
      typedef std::vector< const MatrixType* > MatrixPtrVector;

      double _tolerance;
      double _residual;
      bool _absoluteTolerance;
      bool _verbose;
      int _maxIter;

      MatrixPtrVector _diag;
      MatrixVector _J;

      std::vector<std::pair<int, int> > _indices;
      MatrixPtrVector _sparseMat;

      void multDiag(const std::vector<int>& colBlockIndices, MatrixVector& A, const Eigen::VectorXd& src, Eigen::VectorXd& dest);
      void multDiag(const std::vector<int>& colBlockIndices, MatrixPtrVector& A, const Eigen::VectorXd& src, Eigen::VectorXd& dest);
      void mult(const std::vector<int>& colBlockIndices, const Eigen::VectorXd& src, Eigen::VectorXd& dest);
  };

#include "implementation/linear_solver_pcg.hpp"

}// end namespace

#endif

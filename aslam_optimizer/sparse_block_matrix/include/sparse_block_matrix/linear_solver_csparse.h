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

#ifndef SBM_LINEAR_SOLVERCSPARSE_H
#define SBM_LINEAR_SOLVERCSPARSE_H

#include "csparse_helper.h"

#include <sparse_block_matrix/linear_solver.h>
#include <sparse_block_matrix/marginal_covariance_cholesky.h>
#include <sparse_block_matrix/sparse_helper.h>
#include <cholmod.h>

#include <iostream>

namespace sparse_block_matrix {

/**
 * \brief Our C++ version of the csparse struct
 */
struct CSparseExt : public cs
{
  CSparseExt()
  {
    nzmax = 0;
    m = 0;
    n = 0;
    p = 0;
    i = 0;
    x = 0;
    nz = 0;
    columnsAllocated = 0;
  }
  ~CSparseExt()
  {
    delete[] p;
    delete[] i;
    delete[] x;
  }
  int columnsAllocated;
};

/**
 * \brief linear solver which uses CSparse
 */
template <typename MatrixType>
class LinearSolverCSparse : public LinearSolver<MatrixType>
{
  public:
    LinearSolverCSparse() :
      LinearSolver<MatrixType>()
    {
      _symbolicDecomposition = 0;
      _csWorkspaceSize = -1;
      _csWorkspace = 0;
      _csIntWorkspace = 0;
      _ccsA = new CSparseExt;
      _blockOrdering = true;
    }

    virtual ~LinearSolverCSparse()
    {
      if (_symbolicDecomposition) {
        cs_sfree(_symbolicDecomposition);
        _symbolicDecomposition = 0;
      }
      delete[] _csWorkspace; _csWorkspace = 0;
      delete[] _csIntWorkspace; _csIntWorkspace = 0;
      delete _ccsA;
    }

    virtual bool init()
    {
      if (_symbolicDecomposition) {
        cs_sfree(_symbolicDecomposition);
        _symbolicDecomposition = 0;
      }
      return true;
    }

    bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b)
    {
      fillCSparse(A, _symbolicDecomposition);
      // perform symbolic cholesky once
      if (_symbolicDecomposition == 0) {
        computeSymbolicDecomposition(A);
      }
      // re-allocate the temporary workspace for cholesky
      if (_csWorkspaceSize < _ccsA->n) {
        _csWorkspaceSize = 2 * _ccsA->n;
        delete[] _csWorkspace;
        _csWorkspace = new double[_csWorkspaceSize];
        delete[] _csIntWorkspace;
        _csIntWorkspace = new int[2*_csWorkspaceSize];
      }

      //double t=get_time();
      // _x = _b for calling csparse
      if (x != b)
        memcpy(x, b, _ccsA->n * sizeof(double));
      int ok = cs_cholsolsymb(_ccsA, x, _symbolicDecomposition, _csWorkspace, _csIntWorkspace);
      if (! ok) {
        std::cerr << "Cholesky failure, writing debug.txt (Hessian loadable by Octave)" << std::endl;
        writeCs2Octave("debug.txt", _ccsA, true);
        return false;
      }

      //if (globalStats){
      //  globalStats->timeNumericDecomposition = get_time() - t;
      //  globalStats->choleskyNNZ = _symbolicDecomposition->lnz;
      // }

      return ok;
    }

    bool solveBlocks(double**& blocks, const SparseBlockMatrix<MatrixType>& A) {
      fillCSparse(A, _symbolicDecomposition);
      // perform symbolic cholesky once
      if (_symbolicDecomposition == 0) {
        computeSymbolicDecomposition(A);
        assert(_symbolicDecomposition && "Symbolic cholesky failed");
      }
      // re-allocate the temporary workspace for cholesky
      if (_csWorkspaceSize < _ccsA->n) {
        _csWorkspaceSize = 2 * _ccsA->n;
        delete[] _csWorkspace;
        _csWorkspace = new double[_csWorkspaceSize];
        delete[] _csIntWorkspace;
        _csIntWorkspace = new int[2*_csWorkspaceSize];
      }

      if (! blocks){
        blocks=new double*[A.rows()];
        double **block=blocks;
        for (size_t i=0; i < A.rowBlockIndices().size(); ++i){
          int dim = A.rowsOfBlock(i) * A.colsOfBlock(i);
          *block = new double [dim];
          block++;
        }
      }

      int ok = 1;
      csn* numericCholesky = cs_chol_workspace(_ccsA, _symbolicDecomposition, _csIntWorkspace, _csWorkspace);
      if (numericCholesky) {
        MarginalCovarianceCholesky mcc;
        mcc.setCholeskyFactor(_ccsA->n, numericCholesky->L->p, numericCholesky->L->i, numericCholesky->L->x, _symbolicDecomposition->pinv);
        mcc.computeCovariance(blocks, A.rowBlockIndices());
        cs_nfree(numericCholesky);
      } else {
        ok = 0;
        std::cerr << "inverse fail (numeric decomposition)" << std::endl;
      }

      //if (globalStats){
      //  globalStats->choleskyNNZ = _symbolicDecomposition->lnz;
      //}

      return ok;
    }

    virtual bool solvePattern(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices, const SparseBlockMatrix<MatrixType>& A) {
      fillCSparse(A, _symbolicDecomposition);
      // perform symbolic cholesky once
      if (_symbolicDecomposition == 0) {
        computeSymbolicDecomposition(A);
        assert(_symbolicDecomposition && "Symbolic cholesky failed");
      }
      // re-allocate the temporary workspace for cholesky
      if (_csWorkspaceSize < _ccsA->n) {
        _csWorkspaceSize = 2 * _ccsA->n;
        delete[] _csWorkspace;
        _csWorkspace = new double[_csWorkspaceSize];
        delete[] _csIntWorkspace;
        _csIntWorkspace = new int[2*_csWorkspaceSize];
      }


      int ok = 1;
      csn* numericCholesky = cs_chol_workspace(_ccsA, _symbolicDecomposition, _csIntWorkspace, _csWorkspace);
      if (numericCholesky) {
        MarginalCovarianceCholesky mcc;
        mcc.setCholeskyFactor(_ccsA->n, numericCholesky->L->p, numericCholesky->L->i, numericCholesky->L->x, _symbolicDecomposition->pinv);
	mcc.computeCovariance(spinv, A.rowBlockIndices(), blockIndices);
        cs_nfree(numericCholesky);
      } else {
        ok = 0;
        std::cerr << "inverse fail (numeric decomposition)" << std::endl;
      }

      //if (globalStats){
      //  globalStats->choleskyNNZ = _symbolicDecomposition->lnz;
      // }

      return ok;
    }

    //! do the AMD ordering on the blocks or on the scalar matrix
    bool blockOrdering() const { return _blockOrdering;}
    void setBlockOrdering(bool blockOrdering) { _blockOrdering = blockOrdering;}

  protected:
    css* _symbolicDecomposition;
    int _csWorkspaceSize;
    double* _csWorkspace;
    int* _csIntWorkspace;
    CSparseExt* _ccsA;
    bool _blockOrdering;
    MatrixStructure _matrixStructure;
    VectorXi _scalarPermutation;

    void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A)
    {
      //double t=get_time();
      if (! _blockOrdering) {
        _symbolicDecomposition = cs_schol (1, _ccsA) ;
      } else {
        A.fillBlockStructure(_matrixStructure);

        // prepare block structure for the CSparse call
        cs auxBlock;
        auxBlock.nzmax = _matrixStructure.nzMax();
        auxBlock.m = auxBlock.n = _matrixStructure.n;
        auxBlock.p = _matrixStructure.Ap;
        auxBlock.i = _matrixStructure.Aii;
        auxBlock.x = NULL; // no values
        auxBlock.nz = -1; // CCS format

        // AMD ordering on the block structure
        const int& n = _ccsA->n;
        int* P = cs_amd(1, &auxBlock);

        // blow up the permutation to the scalar matrix
        if (_scalarPermutation.size() == 0)
          _scalarPermutation.resize(n);
        if (_scalarPermutation.size() < n)
          _scalarPermutation.resize(2*n);
        size_t scalarIdx = 0;
        for (int i = 0; i < _matrixStructure.n; ++i) {
          const int& p = P[i];
          int base  = A.colBaseOfBlock(p);
          int nCols = A.colsOfBlock(p);
          for (int j = 0; j < nCols; ++j)
            _scalarPermutation(scalarIdx++) = base++;
        }
        assert((int)scalarIdx == n);
        cs_free(P);

        // apply the scalar permutation to finish symbolic decomposition
        _symbolicDecomposition = (css*) cs_calloc(1, sizeof(css));       /* allocate result S */
        _symbolicDecomposition->pinv = cs_pinv(_scalarPermutation.data(), n);
        cs* C = cs_symperm(_ccsA, _symbolicDecomposition->pinv, 0);
        _symbolicDecomposition->parent = cs_etree(C, 0);
        int* post = cs_post(_symbolicDecomposition->parent, n);
        int* c = cs_counts(C, _symbolicDecomposition->parent, post, 0);
        cs_free(post);
        cs_spfree(C);
        _symbolicDecomposition->cp = (int*) cs_malloc(n+1, sizeof(int));
        _symbolicDecomposition->unz = _symbolicDecomposition->lnz = cs_cumsum(_symbolicDecomposition->cp, c, n);
        cs_free(c);
        if (_symbolicDecomposition->lnz < 0) {
          cs_sfree(_symbolicDecomposition);
          _symbolicDecomposition = 0;
        }

      }
      //if (globalStats){
      //  globalStats->timeSymbolicDecomposition = get_time() - t;
      // }

      /* std::cerr << "# Number of nonzeros in L: " << (int)_symbolicDecomposition->lnz << " by " */
      /*   << (_blockOrdering ? "block" : "scalar") << " AMD ordering " << std::endl; */
    }

    void fillCSparse(const SparseBlockMatrix<MatrixType>& A, bool onlyValues)
    {
      int m = A.rows();
      int n = A.cols();

      if (_ccsA->columnsAllocated < n) {
        _ccsA->columnsAllocated = _ccsA->columnsAllocated == 0 ? n : 2 * n; // pre-allocate more space if re-allocating
        delete[] _ccsA->p;
        _ccsA->p = new int[_ccsA->columnsAllocated+1];
      }

      if (! onlyValues) {
        int nzmax = A.nonZeros();
        if (_ccsA->nzmax < nzmax) {
          _ccsA->nzmax = _ccsA->nzmax == 0 ? nzmax : 2 * nzmax; // pre-allocate more space if re-allocating
          delete[] _ccsA->x;
          delete[] _ccsA->i;
          _ccsA->i = new int[_ccsA->nzmax];
          _ccsA->x = new double[_ccsA->nzmax];
        }
      }
      _ccsA->m = m;
      _ccsA->n = n;

      if (onlyValues) {
        A.template fillCCS<int>(_ccsA->x, true);
      } else {
        int nz = A.template fillCCS<int>(_ccsA->p, _ccsA->i, _ccsA->x, true); (void) nz;
        assert(nz <= _ccsA->nzmax);
      }
      _ccsA->nz=-1; // tag as CCS formatted matrix
    }
};

} // end namespace

#endif

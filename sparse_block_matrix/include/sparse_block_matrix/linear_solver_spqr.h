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

#ifndef SBM_LINEAR_SOLVER_SPQR
#define SBM_LINEAR_SOLVER_SPQR

#include <sparse_block_matrix/linear_solver.h>
#include <SuiteSparseQR.hpp>
#include <sparse_block_matrix/sparse_helper.h>
#include <cholmod.h>

namespace sparse_block_matrix {

/**
 * \brief basic solver for Ax = b which has to reimplemented for different linear algebra libraries
 */
template <typename MatrixType>
class LinearSolverQr : public LinearSolver<MatrixType>
{
  public:
    LinearSolverQr() :
      LinearSolver<MatrixType>()
    {
      _blockOrdering = false;
      _cholmodSparse = new CholmodExt<long>();
      _cholmodFactor = 0;
      cholmod_start(&_cholmodCommon);

      // setup ordering strategy
        _cholmodCommon.nmethods = 1;
        _cholmodCommon.method[0].ordering = SPQR_ORDERING_METIS; //CHOLMOD_COLAMD; //CHOLMOD_COLAMD
        _cholmodCommon.postorder = 0;

        _cholmodCommon.supernodal = CHOLMOD_AUTO; //CHOLMOD_SUPERNODAL; //CHOLMOD_SIMPLICIAL;
        _cholmodCommon.SPQR_nthreads = -1;	// let tbb choose whats best
        _cholmodCommon.SPQR_grain = 12;		// +/-2* number of cores
    }

    virtual ~LinearSolverQr()
    {
      delete _cholmodSparse;
      if ( _cholmodFactor)
    	  SuiteSparseQR_free(&_cholmodFactor, &_cholmodCommon);
      cholmod_finish(&_cholmodCommon);
    }

    virtual bool init()
    {
    	if (_cholmodFactor) {
    		SuiteSparseQR_free(&_cholmodFactor, &_cholmodCommon);
    		_cholmodFactor = 0;
    	}
    	return true;
    }

    bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b)
    {
		fillCholmodExt(A, _cholmodFactor);

	//	std::cerr << "Cholesky failure, writing debug.txt (Hessian loasdable by Octave)" << std::endl;
	//	writeCCSMatrix<long>("debug.txt", _cholmodSparse->nrow, _cholmodSparse->ncol, (long*)_cholmodSparse->p, (long*)_cholmodSparse->i, (double*)_cholmodSparse->x, false);


		cholmod_common *cc ;
		cholmod_dense *X, *B;
		// start CHOLMOD
		cc = &_cholmodCommon ;
		cholmod_l_start (cc) ;
		// load A
		cholmod_dense bcholmod;
		bcholmod.nrow  = bcholmod.d = _cholmodSparse->nrow;
		bcholmod.ncol  = 1;
		bcholmod.x     = b;
		bcholmod.xtype = CHOLMOD_REAL;
		bcholmod.dtype = CHOLMOD_DOUBLE;
		B = &bcholmod;


		/// \todo this is inefficient as we copy everything twice but for now this workaround is ok.
		/// the fillCholmodExt should generate the correct structure immediately.
	//	cholmod_sparse *A1 ;
	//	A1 = cholmod_l_copy (_cholmodSparse, 0, 1, cc);

		// symbolic factorization
		if (! _cholmodFactor) {
			//_cholmodFactor = SuiteSparseQR_symbolic <double>(0, SPQR_DEFAULT_TOL, _cholmodSparse, cc) ;
			_cholmodFactor = SuiteSparseQR_symbolic <double>(SPQR_ORDERING_BEST, SPQR_NO_TOL, _cholmodSparse, cc) ;
		}

		//delete _cholmodSparse;
		//cholmod_l_free_sparse (&_cholmodSparse, cc) ;
		//A = A1;

		// try QR only:
		/*  SuiteSparseQR_factorization <double> *QR = SuiteSparseQR_factorize <double>(0, SPQR_DEFAULT_TOL, _cholmodSparse, cc) ;

		cholmod_dense *Y;
		Y = SuiteSparseQR_qmult (SPQR_QTX, QR, B, cc) ;
		cholmod_dense *XX;
		XX = SuiteSparseQR_solve (SPQR_RX_EQUALS_B, QR, B, cc) ;*/
		/*
		double* k = (double*)Y->x;
		for (int i = 0; i < Y->nrow; i++) {
			std::cout << "Y: " << k[i] << std::endl;
		}
		double* k1 = (double*)XX->x;
		for (int i = 0; i < XX->nrow; i++) {
			std::cout << "XX: " << k1[i] << std::endl;
		}*/

		// X = A\B
		//X = SuiteSparseQR <double> (0, SPQR_NO_TOL, A1, B, cc) ;

		// refactor using numerics:$
		SuiteSparseQR_numeric(SPQR_NO_TOL, _cholmodSparse, _cholmodFactor, cc);

		// solve:
		cholmod_dense* Y = SuiteSparseQR_qmult (SPQR_QTX, _cholmodFactor, B, cc) ;
		X = SuiteSparseQR_solve (SPQR_RETX_EQUALS_B, _cholmodFactor, Y, cc) ;


		if(cc->status != CHOLMOD_OK)
			return false;

		//std::cout << " status:" << cc->status << std::endl;
		/*
		double* k = (double*)X->x;
		for (int i = 0; i < X->nrow; i++) {
			std::cout << "b: " << k[i] << std::endl;
		}*/

		// copy into output array:
		memcpy(x, X->x, sizeof(double) * X->nrow); // copy back to our array

		// free everything and finish CHOLMOD
		cholmod_l_free_dense (&X, cc) ;
		cholmod_l_free_dense (&Y, cc) ;
		//cholmod_l_free_sparse (&A1, cc) ;

		return true;
    }

    //! do the AMD ordering on the blocks or on the scalar matrix
    bool blockOrdering() const { return _blockOrdering;}
    void setBlockOrdering(bool blockOrdering) { _blockOrdering = blockOrdering;}

  protected:
    // temp used for cholesky with cholmod
    cholmod_common _cholmodCommon;
    CholmodExt<long>* _cholmodSparse;
    SuiteSparseQR_factorization<double> * _cholmodFactor;
    bool _blockOrdering;
    MatrixStructure _matrixStructure;
    VectorXi _scalarPermutation, _blockPermutation;

    void fillCholmodExt(const SparseBlockMatrix<MatrixType>& A, bool onlyValues)
    {
      size_t m = A.rows();
      size_t n = A.cols();

      if (_cholmodSparse->columnsAllocated < n) {
        //std::cerr << __PRETTY_FUNCTION__ << ": reallocating columns" << std::endl;
        _cholmodSparse->columnsAllocated = _cholmodSparse->columnsAllocated == 0 ? n : 2 * n; // pre-allocate more space if re-allocating
        delete[] (long*)_cholmodSparse->p;
        _cholmodSparse->p = new long[_cholmodSparse->columnsAllocated+1];
      }
      if (! onlyValues) {
        size_t nzmax = A.nonZeros()*2;
        if (_cholmodSparse->nzmax < nzmax) {
          //std::cerr << __PRETTY_FUNCTION__ << ": reallocating row + values" << std::endl;
          _cholmodSparse->nzmax = _cholmodSparse->nzmax == 0 ? nzmax : 2 * nzmax; // pre-allocate more space if re-allocating
          delete[] (double*)_cholmodSparse->x;
          delete[] (long*)_cholmodSparse->i;
          _cholmodSparse->i = new long[_cholmodSparse->nzmax];
          _cholmodSparse->x = new double[_cholmodSparse->nzmax];
        }
      }
      _cholmodSparse->ncol = n;
      _cholmodSparse->nrow = m;
      // Set the s-type to "Symmetric Upper Triangular"
      _cholmodSparse->stype = 0;

      if (onlyValues){
        A.template fillUpperTriangleCCS<long>((double*)_cholmodSparse->x);
      }
      else {
        A.template fillUpperTriangleCCS<long>((long*)_cholmodSparse->p, (long*)_cholmodSparse->i, (double*)_cholmodSparse->x);
      }
    }

};

} // end namespace

#endif

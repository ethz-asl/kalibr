#ifndef SuiteSparse_long
#define SuiteSparse_long UF_long
#endif

namespace aslam {
  namespace backend {

    // XType
    // #define CHOLMOD_PATTERN 0  /* pattern only, no numerical values */
    // #define CHOLMOD_REAL 1   /* a real matrix */
    // #define CHOLMOD_COMPLEX 2  /* a complex matrix (ANSI C99 compatible) */
    // #define CHOLMOD_ZOMPLEX 3  /* a complex matrix (MATLAB compatible) */

    // DType
    // #define CHOLMOD_DOUBLE 0 /* all numerical values are double */
    // #define CHOLMOD_SINGLE 1 /* all numerical values are float */

    template<>
    struct CholmodValueTraits<double> {
      enum { XType = CHOLMOD_REAL };
      enum { DType = CHOLMOD_DOUBLE };
    };

    template<>
    struct CholmodValueTraits<float> {
      enum { XType = CHOLMOD_REAL };
      enum { DType = CHOLMOD_SINGLE };
    };

    // int itype ;     CHOLMOD_INT:     p, i, and nz are int.
    //                   CHOLMOD_INTLONG: p is SuiteSparse_long,
    //                                    i and nz are int.
    //                   CHOLMOD_LONG:    p, i, and nz are SuiteSparse_long

    // those build a templated interface to the int/SuiteSparse_long interfaces of
    // cholmod

    template<>
    struct CholmodIndexTraits<int> {
      enum { IType = CHOLMOD_INT };

      static int start(cholmod_common* c) {
        return cholmod_start(c);
      }
      static int finish(cholmod_common* c) {
        return cholmod_finish(c);
      }
      static int print_sparse(cholmod_sparse* A, const char* name, cholmod_common* c) {
        return cholmod_print_sparse(A, name, c);
      }
      static cholmod_factor* analyze(cholmod_sparse* A, cholmod_common* c) {
        return cholmod_analyze(A, c);
      }
      static int free_sparse(cholmod_sparse** A, cholmod_common* c) {
        return cholmod_free_sparse(A, c);
      }
      static int free_dense(cholmod_dense** A, cholmod_common* c) {
        return cholmod_free_dense(A, c);
      }
      static int free_factor(cholmod_factor** A, cholmod_common* c) {
        return cholmod_free_factor(A, c);
      }
      static void* free(size_t n, size_t size, void* p, cholmod_common* c) {
        return cholmod_free(n, size, p, c);
      }
      static int factorize(cholmod_sparse* A, cholmod_factor* L, cholmod_common* c) {
        return cholmod_factorize(A, L, c);
      }
      static cholmod_dense* solve(int sys, cholmod_factor* L, cholmod_dense* B, cholmod_common* c) {
        return cholmod_solve(sys, L, B, c);
      }
      static cholmod_sparse* aat(cholmod_sparse* A, int* fset, size_t fsize, int mode, cholmod_common* c) {
        return cholmod_aat(A, fset, fsize, mode, c);
      }
      static int scale(cholmod_dense* S, int scale, cholmod_sparse* A,
          cholmod_common* c) {
        return cholmod_scale(S, scale, A, c);
      }
      static cholmod_dense* allocate_dense(size_t nrow, size_t ncol, size_t d,
          int xtype, cholmod_common* c) {
        return cholmod_allocate_dense(nrow, ncol, d, xtype, c);
      }
    };

    template<>
    struct CholmodIndexTraits<SuiteSparse_long> {
      enum { IType = CHOLMOD_LONG };

      static int start(cholmod_common* c) {
        return cholmod_l_start(c);
      }
      static int finish(cholmod_common* c) {
        return cholmod_l_finish(c);
      }
      static int print_sparse(cholmod_sparse* A, const char* name, cholmod_common* c) {
        return cholmod_l_print_sparse(A, name, c);
      }
      static cholmod_factor* analyze(cholmod_sparse* A, cholmod_common* c) {
        return cholmod_l_analyze(A, c);
      }
      static int free_sparse(cholmod_sparse** A, cholmod_common* c) {
        return cholmod_l_free_sparse(A, c);
      }
      static int free_dense(cholmod_dense** A, cholmod_common* c) {
        return cholmod_l_free_dense(A, c);
      }
      static int free_factor(cholmod_factor** A, cholmod_common* c) {
        return cholmod_l_free_factor(A, c);
      }
      static void* free(size_t n, size_t size, void* p, cholmod_common* c) {
        return cholmod_l_free(n, size, p, c);
      }
      static int factorize(cholmod_sparse* A, cholmod_factor* L, cholmod_common* c) {
        return cholmod_l_factorize(A, L, c);
      }
      static cholmod_dense* solve(int sys, cholmod_factor* L, cholmod_dense* B, cholmod_common* c) {
        return cholmod_l_solve(sys, L, B, c);
      }
      static cholmod_sparse* aat(cholmod_sparse* A, SuiteSparse_long* fset, size_t fsize, int mode, cholmod_common* c) {
        return cholmod_l_aat(A, fset, fsize, mode, c);
      }
      static int scale(cholmod_dense* S, int scale, cholmod_sparse* A,
          cholmod_common* c) {
        return cholmod_l_scale(S, scale, A, c);
      }
      static cholmod_dense* allocate_dense(size_t nrow, size_t ncol, size_t d,
          int xtype, cholmod_common* c) {
        return cholmod_l_allocate_dense(nrow, ncol, d, xtype, c);
      }
    };



    // Some bits of this code were derived from the ceres solver:
    //
    // Ceres Solver - A fast non-linear least squares minimizer
    // Copyright 2010, 2011, 2012 Google Inc. All rights reserved.
    // http://code.google.com/p/ceres-solver/
    //
    // Redistribution and use in source and binary forms, with or without
    // modification, are permitted provided that the following conditions are met:
    //
    // * Redistributions of source code must retain the above copyright notice,
    //   this list of conditions and the following disclaimer.
    // * Redistributions in binary form must reproduce the above copyright notice,
    //   this list of conditions and the following disclaimer in the documentation
    //   and/or other materials provided with the distribution.
    // * Neither the name of Google Inc. nor the names of its contributors may be
    //   used to endorse or promote products derived from this software without
    //   specific prior written permission.
    //
    // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    // POSSIBILITY OF SUCH DAMAGE.
    //
    // Author: sameeragarwal@google.com (Sameer Agarwal)


    template<typename I>
    Cholmod<I>::Cholmod()
    {
      CholmodIndexTraits<index_t>::start(&_cholmod);
    }

    template<typename I>
    Cholmod<I>::~Cholmod()
    {
      CholmodIndexTraits<index_t>::finish(&_cholmod);
    }

    template<typename I>
    cholmod_factor* Cholmod<I>::analyze(cholmod_sparse* J)
    {
      //std::cout << "Cholmod:" << std::endl;
      //CholmodIndexTraits<index_t>::print_sparse(J, "J", &_cholmod);
      //std::cout << "/Cholmod" << std::endl;
      //std::cout << "Checking common\n";
      //cholmod_print_common("Common", &_cholmod);
      //int rval = cholmod_check_common(&_cholmod);
      //std::cout << "common result: " << rval << std::endl;
      //std::cout << "Checking the sparse matrix\n";
      //cholmod_print_sparse(J, "J", &_cholmod);
      //rval = cholmod_check_sparse(J, &_cholmod);
      //std::cout << "sparse matrix result: " << rval << std::endl;
      // From the cholmod header:
      //
      // * If you know the method that is best for your matrix, set Common->nmethods
      // * to 1 and set Common->method [0] to the set of parameters for that method.
      // * If you set it to 1 and do not provide a permutation, then only AMD will
      // * be called.
      _cholmod.nmethods = 1;
      //  AMD may be used with both J or J*J'
      _cholmod.method[0].ordering = CHOLMOD_AMD;
      // From the cholmod header:
      // CHOLMOD_SIMPLICIAL   always do simplicial
      // CHOLMOD_AUTO         select simpl/super depending on matrix
      // CHOLMOD_SUPERNODAL   always do supernodal
      //  * If Common->supernodal <= CHOLMOD_SIMPLICIAL
      //  * (0) then cholmod_analyze performs a
      //  * simplicial analysis.  If >= CHOLMOD_SUPERNODAL (2), then a supernodal
      //  * analysis is performed.  If == CHOLMOD_AUTO (1) and
      //  * flop/nnz(L) < Common->supernodal_switch, then a simplicial analysis
      //  * is done.  A supernodal analysis done otherwise.
      //  * Default:  CHOLMOD_AUTO.  Default supernodal_switch = 40
      _cholmod.supernodal = CHOLMOD_AUTO;
      cholmod_factor* factor = NULL;
      factor = CholmodIndexTraits<index_t>::analyze(J, &_cholmod);
      SM_ASSERT_EQ(Exception, _cholmod.status, CHOLMOD_OK, "The symbolic cholesky factorization failed.");
      SM_ASSERT_FALSE(Exception, factor == NULL, "cholmod_analyze returned a null factor");
      return factor;
    }

#ifndef QRSOLVER_DISABLED
    template<typename I>
    spqr_factor* Cholmod<I>::analyzeQR(cholmod_sparse* J)
    {
      // From the cholmod header:
      //
      // * If you know the method that is best for your matrix, set Common->nmethods
      // * to 1 and set Common->method [0] to the set of parameters for that method.
      // * If you set it to 1 and do not provide a permutation, then only AMD will
      // * be called.
      // _cholmod.nmethods = 1;
      // same properties apply as cholmod_factor analyze
      //_cholmod.method[0].ordering = CHOLMOD_AMD;
      //_cholmod.supernodal = CHOLMOD_AUTO;
      _cholmod.SPQR_nthreads = -1;  // let tbb choose whats best
      _cholmod.SPQR_grain = 12;   // +/-2* number of cores
      spqr_factor* factor = NULL;
      cholmod_sparse* qrJ = cholmod_l_transpose(J, 1, &_cholmod) ;
      factor = SuiteSparseQR_symbolic <double>(SPQR_ORDERING_BEST, SPQR_DEFAULT_TOL, qrJ, &_cholmod) ;
      CholmodIndexTraits<index_t>::free_sparse(&qrJ, &_cholmod);
      SM_ASSERT_EQ(Exception, _cholmod.status, CHOLMOD_OK, "The symbolic qr factorization failed.");
      SM_ASSERT_FALSE(Exception, factor == NULL, "SuiteSparseQR_symbolic returned a null factor");
      return factor;
    }
#endif


    template<typename I>
    void Cholmod<I>::free(cholmod_factor* factor)
    {
      if (factor)
        CholmodIndexTraits<index_t>::free_factor(&factor, &_cholmod);
    }

#ifndef QRSOLVER_DISABLED
    template<typename I>
    void Cholmod<I>::free(spqr_factor* factor)
    {
      if (factor)
        SuiteSparseQR_free(&factor, &_cholmod);
    }
#endif

    /// \brief free a dense vector
    template<typename I>
    void Cholmod<I>::free(cholmod_dense* dense)
    {
      if (dense)
        CholmodIndexTraits<index_t>::free_dense(&dense, &_cholmod);
    }

    template<typename I>
    void Cholmod<I>::free(cholmod_sparse* sparse)
    {
      if (sparse)
        CholmodIndexTraits<index_t>::free_sparse(&sparse, &_cholmod);
    }

    template<typename I>
    void Cholmod<I>::free(size_t n, size_t size, void* p)
    {
      if (p)
        CholmodIndexTraits<index_t>::free(n, size, p, &_cholmod);
    }

    template<typename I>
    bool Cholmod<I>::factorize(cholmod_sparse* A, cholmod_factor* L)
    {
      SM_ASSERT_TRUE(Exception, A != NULL, "Null input");
      SM_ASSERT_TRUE(Exception, L != NULL, "Null input");
      _cholmod.quick_return_if_not_posdef = 1;
      int status = CholmodIndexTraits<index_t>::factorize(A, L, &_cholmod);
      switch (_cholmod.status) {
        case CHOLMOD_NOT_INSTALLED:
          std::cerr << "Cholmod failure: method not installed.";
          return false;
        case CHOLMOD_OUT_OF_MEMORY:
          std::cerr << "Cholmod failure: out of memory.";
          return false;
        case CHOLMOD_TOO_LARGE:
          std::cerr << "Cholmod failure: integer overflow occured.";
          return false;
        case CHOLMOD_INVALID:
          std::cerr << "Cholmod failure: invalid input.";
          return false;
        case CHOLMOD_NOT_POSDEF:
          // TODO(sameeragarwal): These two warnings require more
          // sophisticated handling going forward. For now we will be
          // strict and treat them as failures.
          std::cerr << "Cholmod warning: matrix not positive definite.";
          return false;
        case CHOLMOD_DSMALL:
          std::cerr << "Cholmod warning: D for LDL' or diag(L) or "
                    << "LL' has tiny absolute value.";
          return false;
        case CHOLMOD_OK:
          if (status != 0) {
            return true;
          }
          std::cerr << "Cholmod failure: cholmod_factorize returned zero "
                    << "but cholmod_common::status is CHOLMOD_OK.";
          return false;
        default:
          std::cerr << "Unknown cholmod return code. ";
          return false;
      }
      return false;
    }

#ifndef QRSOLVER_DISABLED
    template<typename I>
    bool Cholmod<I>::factorize(cholmod_sparse* A, spqr_factor* L, double tol,
        bool transpose) {
      cholmod_sparse* At = A;
      if (transpose)
        At = cholmod_l_transpose(A, 1, &_cholmod) ;
      SM_ASSERT_TRUE(Exception, At != NULL, "Null input");
      SM_ASSERT_TRUE(Exception, L != NULL, "Null input");
      _cholmod.quick_return_if_not_posdef = 1;
      int status = SuiteSparseQR_numeric(tol, At, L, &_cholmod);
      // TODO: check if those ones are the same for cholmod and spqr
      switch (_cholmod.status) {
        case CHOLMOD_NOT_INSTALLED:
          std::cerr << "Cholmod failure: method not installed.";
          break;
        case CHOLMOD_OUT_OF_MEMORY:
          std::cerr << "Cholmod failure: out of memory.";
          break;
        case CHOLMOD_TOO_LARGE:
          std::cerr << "Cholmod failure: integer overflow occured.";
          break;
        case CHOLMOD_INVALID:
          std::cerr << "Cholmod failure: invalid input.";
          break;
        case CHOLMOD_NOT_POSDEF:
          // TODO(sameeragarwal): These two warnings require more
          // sophisticated handling going forward. For now we will be
          // strict and treat them as failures.
          std::cerr << "Cholmod warning: matrix not positive definite.";
          break;
        case CHOLMOD_DSMALL:
          std::cerr << "Cholmod warning: D for LDL' or diag(L) or "
                    << "LL' has tiny absolute value.";
          break;
        case CHOLMOD_OK:
          if (status != 0) {
            break;
          }
          std::cerr << "Cholmod failure: cholmod_factorize returned zero "
                    << "but cholmod_common::status is CHOLMOD_OK.";
          break;
        default:
          std::cerr << "Unknown cholmod return code. ";
          break;
      }
      if (transpose)
        CholmodIndexTraits<index_t>::free_sparse(&At, &_cholmod);
      if (_cholmod.status == CHOLMOD_OK && status == 1)
        return true;
      else
        return false;
    }
#endif


    template<typename I>
    cholmod_dense* Cholmod<I>::solve(cholmod_sparse* A,
                                     cholmod_factor* L,
                                     cholmod_dense* b)
    {
      if (factorize(A, L)) {
        //cholmod_print_dense(b, "b", &_cholmod);
        //cholmod_print_sparse(A,"A", &_cholmod);
        cholmod_dense* X = CholmodIndexTraits<index_t>::solve(CHOLMOD_A, L, b, &_cholmod);
        //cholmod_print_dense(X, "X", &_cholmod);
        return X;
      }
      return NULL;
    }


#ifndef QRSOLVER_DISABLED
    template<typename I>
    cholmod_dense* Cholmod<I>::solve(cholmod_sparse* A, spqr_factor* L,
        cholmod_dense* b, double tol, bool norm, double normTol) {
      cholmod_sparse* qrJ = cholmod_l_transpose(A, 1, &_cholmod);
      cholmod_dense* scaling = NULL;
      if (norm) {
        scaling =
          CholmodIndexTraits<index_t>::allocate_dense(qrJ->ncol, 1, qrJ->ncol,
          CHOLMOD_REAL, &_cholmod);
        double* values =
          reinterpret_cast<double*>(scaling->x);
        for (size_t i = 0; i < qrJ->ncol; ++i) {
          const double normCol = colNorm(qrJ, i);
          if (normCol < normTol)
            values[i] = 0.0;
          else
            values[i] = 1.0 / normCol;
        }
        SM_ASSERT_TRUE(Exception, scale(scaling, CHOLMOD_COL, qrJ),
          "Scaling failed");
      }
      cholmod_dense* res = NULL;
      if (factorize(qrJ, L, tol)) {
        cholmod_dense* qrY = SuiteSparseQR_qmult(SPQR_QTX, L, b, &_cholmod);
        res = SuiteSparseQR_solve(SPQR_RETX_EQUALS_B, L, qrY, &_cholmod);
        CholmodIndexTraits<index_t>::free_dense(&qrY, &_cholmod);
      }
      if (norm) {
        const double* svalues =
          reinterpret_cast<const double*>(scaling->x);
        double* rvalues =
          reinterpret_cast<double*>(res->x);
        for (size_t i = 0; i < qrJ->ncol; ++i)
          rvalues[i] = svalues[i] * rvalues[i];
        CholmodIndexTraits<index_t>::free_dense(&scaling, &_cholmod);
      }
      CholmodIndexTraits<index_t>::free_sparse(&qrJ, &_cholmod);
      return res;
    }
#endif

#ifndef QRSOLVER_DISABLED
    template<typename I>
    void Cholmod<I>::getR(cholmod_sparse* A, cholmod_sparse** R) {
      cholmod_sparse* qrJ = cholmod_l_transpose(A, 1, &_cholmod);
      SuiteSparseQR<double>(SPQR_ORDERING_FIXED, SPQR_NO_TOL, qrJ->ncol, 0,
        qrJ, NULL, NULL, NULL, NULL, R, NULL, NULL, NULL, NULL, &_cholmod);
      SM_ASSERT_EQ(Exception, _cholmod.status, CHOLMOD_OK,
        "QR factorization failed");
      CholmodIndexTraits<index_t>::free_sparse(&qrJ, &_cholmod);
    }
#endif

    template<typename I>
    void Cholmod<I>::view(const Eigen::VectorXd& v, cholmod_dense* outDense)
    {
      if (!outDense)
        return;
      // size_t nrow ;  /* the matrix is nrow-by-ncol */
      outDense->nrow = v.size();
      // size_t ncol ;
      outDense->ncol = 1;
      // size_t nzmax ; /* maximum number of entries in the matrix */
      outDense->nzmax = v.size();
      // size_t d ;   /* leading dimension (d >= nrow must hold) */
      outDense->d = v.size();
      // void *x ;    /* size nzmax or 2*nzmax, if present */
      outDense->x = (void*)&v[0];
      // void *z ;    /* size nzmax, if present */
      outDense->z = NULL;
      // int xtype ;    /* pattern, real, complex, or zomplex */
      outDense->xtype = CholmodValueTraits<double>::XType;

      // int dtype ;    /* x and z double or float */
      outDense->dtype = CholmodValueTraits<double>::DType;
    }

    template<typename I>
    cholmod_sparse* Cholmod<I>::aat(cholmod_sparse* A)
    {
      cholmod_sparse* AAt =  CholmodIndexTraits<index_t>::aat(A, NULL, A->nrow, 1, &_cholmod);
      // AAt is upper diagonal
      AAt->stype = 1;
      return AAt;
    }

    template<typename I>
    int Cholmod<I>::scale(cholmod_dense* S, int scale, cholmod_sparse* A) {
      return CholmodIndexTraits<index_t>::scale(S, scale, A, &_cholmod);
    }

    template<typename I>
    double Cholmod<I>::colNorm(cholmod_sparse* A, size_t n) {
      SM_ASSERT_LT(Exception, n, A->ncol, "Index out of bounds");
      const I* col_ptr = reinterpret_cast<const I*>(A->p);
      const double* values = reinterpret_cast<const double*>(A->x);
      const I p = col_ptr[n];
      const I numElements = col_ptr[n + 1] - p;
      double norm = 0;
      for (I i = 0; i < numElements; ++i)
        norm += values[p + i] * values[p + i];
      return sqrt(norm);
    }

    template<typename I>
    size_t Cholmod<I>::getMemoryUsage() const {
      return _cholmod.memory_inuse;
    }

  } // namespace backend
} // namespace aslam

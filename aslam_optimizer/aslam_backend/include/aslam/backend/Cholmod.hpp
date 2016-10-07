#ifndef ASLAM_BACKEND_CHOLMOD_HPP
#define ASLAM_BACKEND_CHOLMOD_HPP

#include <cholmod.h>
#ifndef QRSOLVER_DISABLED
#include <SuiteSparseQR.hpp>
#endif
#include <sm/assert_macros.hpp>
#include <Eigen/Core>

namespace aslam {
  namespace backend {

    template<typename VALUE_T>
    struct CholmodValueTraits { };

    template<typename INDEX_T>
    struct CholmodIndexTraits { };

#ifndef QRSOLVER_DISABLED
    typedef SuiteSparseQR_factorization<double> spqr_factor;  // make it readable
#endif


    template<typename INDEX_T = int>
    class Cholmod {
    public:
      typedef INDEX_T index_t;

      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

      Cholmod();
      virtual ~Cholmod();

      /**
       * \brief Wraps the cholmod_analyze function
       *
       * @param J the sparse matrix to analyze
       *
       * @return a cholmod factor for the matrix. This must be freed using Cholmod::free()
       */
      cholmod_factor* analyze(cholmod_sparse* J);

      /// \brief wraps the spqr analyze functions
#ifndef QRSOLVER_DISABLED
      spqr_factor* analyzeQR(cholmod_sparse* J);
#endif

      /// \brief Wraps the cholmod_factorize function. Returns true for success.
      bool factorize(cholmod_sparse* A, cholmod_factor* L);

#ifndef QRSOLVER_DISABLED
      bool factorize(cholmod_sparse* A, spqr_factor* L,
        double tol = SPQR_DEFAULT_TOL, bool transpose = false);
#endif

      /// \brief free a cholmod_factor
      void free(cholmod_factor* factor);

      /// \brief free a spqr_factor
#ifndef QRSOLVER_DISABLED
      void free(spqr_factor* factor);
#endif

      /// \brief free a dense vector
      void free(cholmod_dense* dense);

      /// \brief free a sparse matrix
      void free(cholmod_sparse* sparse);

      /// \brief free whatever data
      void free(size_t n, size_t size, void* p);

      /// \brief view vector v as a cholmod_dense type
      void view(const Eigen::VectorXd& v, cholmod_dense* outDense);

      /// \brief solve a linear system.
      ///
      /// If the solution is successful, the solution is returned (otherwise NULL)
      /// The return value must be freed with Cholmod::free()
      cholmod_dense* solve(cholmod_sparse* A,
                           cholmod_factor* L,
                           cholmod_dense* b);

#ifndef QRSOLVER_DISABLED
      cholmod_dense* solve(cholmod_sparse* A, spqr_factor* L, cholmod_dense* b,
                           double tol = SPQR_DEFAULT_TOL, bool norm = true,
                           double normTol = 1e-8);
#endif

      cholmod_sparse* aat(cholmod_sparse* A);

      /// Scale a matrix by S
      int scale(cholmod_dense* S, int scale, cholmod_sparse* A);

      /// Returns the 2-norm of column n
      double colNorm(cholmod_sparse* A, size_t n);

#ifndef QRSOLVER_DISABLED
      /// Get the R matrix from the QR decomposition
      void getR(cholmod_sparse* A, cholmod_sparse** R);
#endif

      /// Returns the current memory usage in bytes
      size_t getMemoryUsage() const;

    private:

      cholmod_common _cholmod;

//      cholmod_sparse* _qrJ;
//      cholmod_dense* _qrY;
    };

  } // namespace backend
} // namespace aslam

#include "implementation/Cholmod.hpp"

#endif /* ASLAM_BACKEND_CHOLMOD_HPP */

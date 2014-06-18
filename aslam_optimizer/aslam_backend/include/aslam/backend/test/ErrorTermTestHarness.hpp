#ifndef ASLAM_BACKEND_ERROR_TERM_TEST_HARNESS_HPP
#define ASLAM_BACKEND_ERROR_TERM_TEST_HARNESS_HPP

#include <sm/eigen/gtest.hpp>
#include <aslam/backend/ErrorTerm.hpp>

namespace aslam {
  namespace backend {

  // \todo remove this template argument.
    template<int D>
    class ErrorTermTestHarness {
    public:
      typedef ErrorTerm error_t;

      ErrorTermTestHarness(error_t* error) : _error(error) {       }

      virtual ~ErrorTermTestHarness() {}

      void testAll(double tolerance = 1e-6) {
        ASSERT_TRUE(_error != NULL);
        ASSERT_NE(0u, _error->numDesignVariables());
        for (size_t i = 0; i < _error->numDesignVariables(); ++i) {
          DesignVariable* dv = _error->designVariable(i);
          ASSERT_TRUE(dv != NULL);
          dv->setActive(true);
          dv->setBlockIndex(i);
        }
        JacobianContainer estJ(_error->dimension());
        _error->evaluateJacobiansFiniteDifference(estJ);
        JacobianContainer J(_error->dimension());
        _error->evaluateJacobians(J);

        sm::eigen::assertNear(J.asSparseMatrix(), estJ.asSparseMatrix(), tolerance, SM_SOURCE_FILE_POS, "Testing jacobians vs. finite differences (Matrix A are the analytical Jacobians, Matrix B is from finite differences)");
      }

      error_t* _error;
    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_ERROR_TERM_TEST_HARNESS_HPP */

#ifndef ASLAM_BACKEND_ERROR_TERM_TESTER_HPP
#define ASLAM_BACKEND_ERROR_TERM_TESTER_HPP

#include <sm/eigen/gtest.hpp>
#include "../DesignVariable.hpp"
#include "../JacobianContainerSparse.hpp"

namespace aslam {
  namespace backend {
    class ErrorTerm;
    class ScalarNonSquaredErrorTerm;

#define DefaultErrorTermTestTolerance 1e-6

    template<typename E>
    class ErrorTermTester {
    public:
      typedef E error_t;

      ErrorTermTester(error_t& error) : _error(error) {}

      void testAll(double tolerance = DefaultErrorTermTestTolerance){
        ASSERT_NE(0u, _error.numDesignVariables());
        for (size_t i = 0; i < _error.numDesignVariables(); ++i) {
          DesignVariable* dv = _error.designVariable(i);
          ASSERT_TRUE(dv != NULL);
          dv->setActive(true);
          dv->setBlockIndex(i);
        }
        JacobianContainerSparse<> estJ(_error.dimension());
        _error.evaluateJacobiansFiniteDifference(estJ);
        JacobianContainerSparse<> J(_error.dimension());
        _error.evaluateJacobians(J);

        sm::eigen::assertNear(J.asSparseMatrix(), estJ.asSparseMatrix(), tolerance, SM_SOURCE_FILE_POS, "Testing jacobians vs. finite differences (Matrix A are the analytical Jacobians, Matrix B is from finite differences)");
      }

      ~ErrorTermTester() {}

      error_t& _error;
    };

#if __cplusplus >= 201103L
    template <typename ET>
    void testErrorTerm(ET && errorTerm, double tolerance = DefaultErrorTermTestTolerance){
      ErrorTermTester<typename std::remove_reference<ET>::type>(errorTerm).testAll(tolerance);
    }
#else
    template <typename ET>
    void testErrorTerm(ET & errorTerm, double tolerance = DefaultErrorTermTestTolerance){
      ErrorTermTester<ET>(errorTerm).testAll(tolerance);
    }
#endif
    template <typename ET>
    void testErrorTerm(::boost::shared_ptr<ET> errorTerm, double tolerance = DefaultErrorTermTestTolerance){
      testErrorTerm(*errorTerm, tolerance);
    }

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_ERROR_TERM_TESTER_HPP */

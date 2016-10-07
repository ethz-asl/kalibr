#ifndef ASLAM_BACKEND_ERROR_TERM_TEST_HARNESS_HPP
#define ASLAM_BACKEND_ERROR_TERM_TEST_HARNESS_HPP

#include "ErrorTermTester.hpp"

namespace aslam {
  namespace backend {

    //TODO(HannesSommer) Issue somehow compiler warning about deprecation
    
    /// \brief Deprecated: use ErrorTermTester or testErrorTerm(...) in ErrorTermTester.hpp instead.
    template<int D>
    class ErrorTermTestHarness : public ErrorTermTester<ErrorTerm> {
    public:
      void assertNotNull(ErrorTerm* error){
        ASSERT_TRUE(error != NULL);
      }
      ErrorTermTestHarness(ErrorTerm* error) : ErrorTermTester<ErrorTerm>(*error) {
        assertNotNull(error);
      }
    };
  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_ERROR_TERM_TEST_HARNESS_HPP */

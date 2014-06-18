#include <aslam/backend/OptimizationProblemBase.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>

namespace aslam {
  namespace backend {

    OptimizationProblemBase::OptimizationProblemBase()
    {
    }

    OptimizationProblemBase::~OptimizationProblemBase()
    {
    }

    /// \brief The number of design variables stored in this optimization problem
    size_t OptimizationProblemBase::numDesignVariables() const
    {
      return numDesignVariablesImplementation();
    }
    /// \brief Get design variable i
    DesignVariable* OptimizationProblemBase::designVariable(size_t i)
    {
      SM_ASSERT_LT_DBG(Exception, i, numDesignVariables(), "index out of bounds");
      return designVariableImplementation(i);
    }
    /// \brief Get design variable i
    const DesignVariable* OptimizationProblemBase::designVariable(size_t i) const
    {
      SM_ASSERT_LT_DBG(Exception, i, numDesignVariables(), "index out of bounds");
      return designVariableImplementation(i);
    }

    /// \brief the number of measurements stored in this optimization problem
    size_t OptimizationProblemBase::numErrorTerms() const
    {
      return numErrorTermsImplementation();
    }
    /// \brief get error term i.
    ErrorTerm* OptimizationProblemBase::errorTerm(size_t i)
    {
      SM_ASSERT_LT_DBG(Exception, i, numErrorTerms(), "index out of bounds");
      return errorTermImplementation(i);
    }
    /// \brief get error term i.
    const ErrorTerm* OptimizationProblemBase::errorTerm(size_t i) const
    {
      SM_ASSERT_LT_DBG(Exception, i,  numErrorTerms(), "index out of bounds");
      return errorTermImplementation(i);
    }

    void OptimizationProblemBase::getErrors(const DesignVariable* dv, std::set<ErrorTerm*>& outErrorSet)
    {
      return getErrorsImplementation(dv, outErrorSet);
    }

  } // namespace backend
} // namespace aslam

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
    /// \brief the number of non-squared error terms stored in this optimization problem
    size_t OptimizationProblemBase::numNonSquaredErrorTerms() const
    {
      return numNonSquaredErrorTermsImplementation();
    }

    /// \brief the total number of error terms (squared and non-squared) stored in this optimization problem
    size_t OptimizationProblemBase::numTotalErrorTerms() const
    {
      return numErrorTermsImplementation() + numNonSquaredErrorTermsImplementation();
    }

    /// \brief get error term i.
    ErrorTerm* OptimizationProblemBase::errorTerm(size_t i)
    {
      SM_ASSERT_LT_DBG(Exception, i, numErrorTerms(), "index out of bounds");
      return errorTermImplementation(i);
    }
    /// \brief get error term i.
    ScalarNonSquaredErrorTerm* OptimizationProblemBase::nonSquaredErrorTerm(size_t i)
    {
      SM_ASSERT_LT_DBG(Exception, i, numNonSquaredErrorTerms(), "index out of bounds");
      return nonSquaredErrorTermImplementation(i);
    }
    /// \brief get error term i.
    const ErrorTerm* OptimizationProblemBase::errorTerm(size_t i) const
    {
      SM_ASSERT_LT_DBG(Exception, i,  numErrorTerms(), "index out of bounds");
      return errorTermImplementation(i);
    }
    /// \brief get error term i.
    const ScalarNonSquaredErrorTerm* OptimizationProblemBase::nonSquaredErrorTerm(size_t i) const
    {
      return nonSquaredErrorTermImplementation(i);
    }

    void OptimizationProblemBase::getErrors(const DesignVariable* dv, std::set<ErrorTerm*>& outErrorSet)
    {
      return getErrorsImplementation(dv, outErrorSet);
    }
    void OptimizationProblemBase::getNonSquaredErrors(const DesignVariable* dv, std::set<ScalarNonSquaredErrorTerm*>& outErrorSet)
    {
      return getNonSquaredErrorsImplementation(dv, outErrorSet);
    }

  } // namespace backend
} // namespace aslam

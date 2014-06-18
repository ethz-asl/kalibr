#include <aslam/backend/SimpleOptimizationProblem.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <sm/boost/null_deleter.hpp>

namespace aslam {
  namespace backend {

    SimpleOptimizationProblem::SimpleOptimizationProblem()
    {
    }

    SimpleOptimizationProblem::~SimpleOptimizationProblem()
    {
    }


    /// \brief Add a design variable to the problem. If the second
    /// argument is true, the design variable will be deleted
    /// when the problem is cleared or goes out of scope.
    void SimpleOptimizationProblem::addDesignVariable(DesignVariable* dv, bool problemOwnsVariable)
    {
      // \todo This is also bad and slow.
      for (unsigned i = 0; i < _designVariables.size(); ++i) {
        SM_ASSERT_NE(std::runtime_error, dv, _designVariables[i].get(), "That design variable has already been added");
      }
      if (problemOwnsVariable)
        _designVariables.push_back(boost::shared_ptr<DesignVariable>(dv));
      else
        _designVariables.push_back(boost::shared_ptr<DesignVariable>(dv, sm::null_deleter()));
    }


    /// \brief Add a design variable to the problem.
    void SimpleOptimizationProblem::addDesignVariable(boost::shared_ptr<DesignVariable> dv)
    {
      // \todo This is also bad and slow.
      for (unsigned i = 0; i < _designVariables.size(); ++i) {
        SM_ASSERT_NE(std::runtime_error, dv.get(), _designVariables[i].get(), "That design variable has already been added");
      }
      _designVariables.push_back(dv);
    }


    /// \brief Add an error term to the problem. If the second
    /// argument is true, the error term will be deleted when the
    /// problem is cleared or goes out of scope.
    void SimpleOptimizationProblem::addErrorTerm(ErrorTerm* ev, bool problemOwnsVariable)
    {
      if (problemOwnsVariable)
        addErrorTerm(boost::shared_ptr<ErrorTerm>(ev));
      else
        addErrorTerm(boost::shared_ptr<ErrorTerm>(ev, sm::null_deleter()))
        ;
    }


    /// \brief Add an error term to the problem
    void SimpleOptimizationProblem::addErrorTerm(const boost::shared_ptr<ErrorTerm> & et)
    {
      _errorTerms.push_back(et);
    }


    bool SimpleOptimizationProblem::isDesignVariableInProblem(const DesignVariable* dv)
    {
      for (size_t i = 0; i < _designVariables.size(); ++i) {
        if (_designVariables[i].get() == dv)
          return true;
      }
      return false;
    }

    /// \brief clear the design variables and error terms.
    void SimpleOptimizationProblem::clear()
    {
      _errorTerms.clear();
      _designVariables.clear();
    }


    size_t SimpleOptimizationProblem::numDesignVariablesImplementation() const
    {
      return _designVariables.size();
    }

    DesignVariable* SimpleOptimizationProblem::designVariableImplementation(size_t i)
    {
      return _designVariables[i].get();
    }

    const DesignVariable* SimpleOptimizationProblem::designVariableImplementation(size_t i) const
    {
      return _designVariables[i].get();
    }


    size_t SimpleOptimizationProblem::numErrorTermsImplementation() const
    {
      return _errorTerms.size();
    }

    ErrorTerm* SimpleOptimizationProblem::errorTermImplementation(size_t i)
    {
      return _errorTerms[i].get();
    }

    const ErrorTerm* SimpleOptimizationProblem::errorTermImplementation(size_t i) const
    {
      return _errorTerms[i].get();
    }

  void SimpleOptimizationProblem::getErrorsImplementation(const DesignVariable* /*dv*/, std::set<ErrorTerm*>& /*outErrorSet*/)
    {
      SM_THROW(std::runtime_error, "Not implemented");
    }



  } // namespace backend
} // namespace aslam

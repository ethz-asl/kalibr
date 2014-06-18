#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <sm/boost/null_deleter.hpp>

namespace aslam {
  namespace backend {

    OptimizationProblem::OptimizationProblem()
    {
    }

    OptimizationProblem::~OptimizationProblem()
    {
    }


    /// \brief Add a design variable to the problem. If the second
    /// argument is true, the design variable will be deleted
    /// when the problem is cleared or goes out of scope.
    void OptimizationProblem::addDesignVariable(DesignVariable* dv, bool problemOwnsVariable)
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
    void OptimizationProblem::addDesignVariable(boost::shared_ptr<DesignVariable> dv)
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
    void OptimizationProblem::addErrorTerm(ErrorTerm* ev, bool problemOwnsVariable)
    {
      if (problemOwnsVariable)
        addErrorTerm(boost::shared_ptr<ErrorTerm>(ev));
      else
        addErrorTerm(boost::shared_ptr<ErrorTerm>(ev, sm::null_deleter()))
        ;
    }


    /// \brief Add an error term to the problem
    void OptimizationProblem::addErrorTerm(const boost::shared_ptr<ErrorTerm> & et)
    {
      _errorTerms.push_back(et);
      // add this error term to the multi-map
      for (size_t i = 0; i < et->numDesignVariables(); ++i) {
        DesignVariable* dv = et->designVariable(i);
        SM_ASSERT_TRUE_DBG(aslam::InvalidArgumentException, isDesignVariableInProblem(dv), "It is illegal to add an error term that contains a missing design variable. Add the design variables to the problem before adding the error terms.");
        _errorTermMap.insert(std::make_pair(dv, et.get()));
      }
    }


    bool OptimizationProblem::isDesignVariableInProblem(const DesignVariable* dv)
    {
      for (size_t i = 0; i < _designVariables.size(); ++i) {
        if (_designVariables[i].get() == dv)
          return true;
      }
      return false;
    }

    /// \brief clear the design variables and error terms.
    void OptimizationProblem::clear()
    {
      _errorTerms.clear();
      _designVariables.clear();
      _errorTermMap.clear();
    }


    size_t OptimizationProblem::numDesignVariablesImplementation() const
    {
      return _designVariables.size();
    }

    DesignVariable* OptimizationProblem::designVariableImplementation(size_t i)
    {
      return _designVariables[i].get();
    }

    const DesignVariable* OptimizationProblem::designVariableImplementation(size_t i) const
    {
      return _designVariables[i].get();
    }


    size_t OptimizationProblem::numErrorTermsImplementation() const
    {
      return _errorTerms.size();
    }

    ErrorTerm* OptimizationProblem::errorTermImplementation(size_t i)
    {
      return _errorTerms[i].get();
    }

    const ErrorTerm* OptimizationProblem::errorTermImplementation(size_t i) const
    {
      return _errorTerms[i].get();
    }

    void OptimizationProblem::getErrorsImplementation(const DesignVariable* dv, std::set<ErrorTerm*>& outErrorSet)
    {
      error_map_t::iterator it, it_end;
      // I'm not sure why this const cast is necessary but I get a failure on OSX if it isn't here.
      // Also, somehow this is painfully slow.
      boost::tie(it, it_end) = _errorTermMap.equal_range(const_cast<DesignVariable*>(dv));
      for (; it != it_end; ++it) {
        outErrorSet.insert(it->second);
      }
    }

    /// \brief Remove the error term
    void OptimizationProblem::removeErrorTerm(const ErrorTerm* et)
    {
      // For each design variable associated with this error term
      for (size_t i = 0; i < et->numDesignVariables(); ++i) {
        const DesignVariable* dv = et->designVariable(i);
        error_map_t::iterator it, it_end;
        boost::tie(it, it_end) = _errorTermMap.equal_range(const_cast<DesignVariable*>(dv));
        for (; it != it_end; it++) {
          if (it->second == et) {
            // remove the reference from the multi-map.
            _errorTermMap.erase(it);
            break;
          }
        }
      }
      // now remove the design variable.
      // This sucks. \todo Make this more efficient.
      std::vector< boost::shared_ptr<ErrorTerm> >::iterator eit = _errorTerms.begin();
      for (; eit != _errorTerms.end(); ++eit) {
        if (eit->get() == et) {
          // remove this error term from the set of error terms.
          _errorTerms.erase(eit);
          break;
        }
      }
    }

    /// \brief Remove the design variable
    void OptimizationProblem::removeDesignVariable(const DesignVariable* dv)
    {
      // Remove any error terms from the project
      error_map_t::iterator it, it2, it_end;
      boost::tie(it, it_end) = _errorTermMap.equal_range(const_cast<DesignVariable*>(dv));
      std::vector<ErrorTerm*> terms;
      for (; it != it_end; ++it) {
        terms.push_back(it->second);
      }
      for (size_t i = 0; i < terms.size(); ++i) {
        removeErrorTerm(terms[i]);
      }
      // Now remove the design variable itself.
      std::vector< boost::shared_ptr<DesignVariable> >::iterator dit = _designVariables.begin();
      for (; dit != _designVariables.end(); ++dit) {
        if (dit->get() == dv) {
          _designVariables.erase(dit);
          return;
        }
      }
    }

    size_t OptimizationProblem::countActiveDesignVariables() {
      size_t c = 0;
      for(boost::shared_ptr<DesignVariable> dv : _designVariables){
        if(dv->isActive()){
          c ++;
        }
      }
      return c;
    }
  } // namespace backend
}  // namespace aslam


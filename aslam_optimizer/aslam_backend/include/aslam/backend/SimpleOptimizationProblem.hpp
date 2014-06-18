#ifndef ASLAM_BACKEND_OPTIMIZATION_PROBLEM_SIMPLE2
#define ASLAM_BACKEND_OPTIMIZATION_PROBLEM_SIMPLE2

#include "OptimizationProblemBase.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>

#include <unordered_map>

namespace aslam {
  namespace backend {

    /**
     * \class SimpleOptimizationProblem
     *
     * \brief a simple implementation of the optimzation problem that
     *        only stores containers of design variables and error terms.
     *        This container owns the design variables and error terms and
     *        it will call delete on them when it goes out of scope.
     */
    class SimpleOptimizationProblem : public OptimizationProblemBase {
    public:
      SimpleOptimizationProblem();
      virtual ~SimpleOptimizationProblem();

      /// \brief Add a design variable to the problem. If the second
      /// argument is true, the design variable will be deleted
      /// when the problem is cleared or goes out of scope.
      void addDesignVariable(DesignVariable* dv, bool problemOwnsVariable);

      /// \brief Add a design variable to the problem.
      void addDesignVariable(boost::shared_ptr<DesignVariable> dv);

      /// \brief Add an error term to the problem. If the second
      /// argument is true, the error term will be deleted when the
      /// problem is cleared or goes out of scope.
      void addErrorTerm(ErrorTerm* dv, bool problemOwnsVariable);

      /// \brief Add an error term to the problem
      virtual void addErrorTerm(const boost::shared_ptr<ErrorTerm> & et);

      /// \brief clear the design variables and error terms.
      void clear();

      /// \brief used for debugging...is the design variable in the problem.
      bool isDesignVariableInProblem(const DesignVariable* dv);
    protected:
      virtual size_t numDesignVariablesImplementation() const;
      virtual DesignVariable* designVariableImplementation(size_t i);
      virtual const DesignVariable* designVariableImplementation(size_t i) const;

      virtual size_t numErrorTermsImplementation() const;
      virtual ErrorTerm* errorTermImplementation(size_t i);
      virtual const ErrorTerm* errorTermImplementation(size_t i) const;

      virtual void getErrorsImplementation(const DesignVariable* dv, std::set<ErrorTerm*>& outErrorSet);

      // \todo Replace these std::vectors by something better. The underlying algorithms that this object
      //       supports suck with these containers. Blerg. See "removeDesignVariable()" for an example of
      //       just how bad this is.
      std::vector< boost::shared_ptr<DesignVariable> > _designVariables;
      std::vector< boost::shared_ptr<ErrorTerm> > _errorTerms;
    };


  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_OPTIMIZATION_PROBLEM_SIMPLE2 */

#ifndef ASLAM_OPTIMIZATION_PROBLEM_HPP
#define ASLAM_OPTIMIZATION_PROBLEM_HPP

#include <aslam/Exceptions.hpp>
#include <set>
#include <boost/shared_ptr.hpp>

namespace aslam {
  namespace backend {
    class DesignVariable;
    class ErrorTerm;
    class ScalarNonSquaredErrorTerm;

    class ErrorTermReceiver {
     public:
      virtual ~ErrorTermReceiver(){}
      virtual void addErrorTerm(const boost::shared_ptr<ErrorTerm> & et) = 0;
    };

    template <typename Functor>
    class FunctionErrorTermReceiver : public ErrorTermReceiver {
     public:
      FunctionErrorTermReceiver(Functor addErrorTermFunctor) : addErrorTermFunctor_(addErrorTermFunctor){}

      Functor addErrorTermFunctor_;
      void addErrorTerm(const boost::shared_ptr<ErrorTerm> & et) override {
        addErrorTermFunctor_(et);
      }
    };

    template <typename Functor>
    FunctionErrorTermReceiver<Functor> toErrorTermReceiver(Functor addErrorTermFunctor){
      return FunctionErrorTermReceiver<Functor>(addErrorTermFunctor);
    }

    class OptimizationProblemBase : public ErrorTermReceiver {
    public:
      SM_DEFINE_EXCEPTION(Exception, aslam::Exception);

      OptimizationProblemBase();
      virtual ~OptimizationProblemBase();

      /// \brief The number of design variables stored in this optimization problem
      size_t numDesignVariables() const;

      /// \brief Get design variable i
      DesignVariable* designVariable(size_t i);

      /// \brief Get design variable i
      const DesignVariable* designVariable(size_t i) const;

      /// \brief the number of error terms stored in this optimization problem
      size_t numErrorTerms() const;

      /// \brief the number of non-squared error terms stored in this optimization problem
      size_t numNonSquaredErrorTerms() const;

      /// \brief the total number of error terms (squared and non-squared) stored in this optimization problem
      size_t numTotalErrorTerms() const;

      /// \brief get error term i.
      ErrorTerm* errorTerm(size_t i);

      /// \brief get error term i.
      ScalarNonSquaredErrorTerm* nonSquaredErrorTerm(size_t i);

      /// \brief get error term i.
      const ErrorTerm* errorTerm(size_t i) const;

      /// \brief get non-squared error term i.
      const ScalarNonSquaredErrorTerm* nonSquaredErrorTerm(size_t i) const;

      /// \brief Get the error terms associated with a specific design variable
      void getErrors(const DesignVariable* dv, std::set<ErrorTerm*>& outErrorSet);

      /// \brief Get the non-squared error terms associated with a specific design variable
      void getNonSquaredErrors(const DesignVariable* dv, std::set<ScalarNonSquaredErrorTerm*>& outErrorSet);

      /// \brief This is ugly but it is just enough abstract interface
      ///        to allow error term factories to work
      virtual void addErrorTerm(const boost::shared_ptr<ErrorTerm> & /* et */) override {
        SM_THROW(std::runtime_error, "Not Implemented");
      }
    protected:

      /// \brief the number of design variables used in this optimization problem
      virtual size_t numDesignVariablesImplementation() const = 0;
      /// \brief get design variable k
      virtual DesignVariable* designVariableImplementation(size_t k) = 0;
      /// \brief get design variable k
      virtual const DesignVariable* designVariableImplementation(size_t k) const = 0;

      /// \brief the number of error terms in this optimization problem
      virtual size_t numErrorTermsImplementation() const = 0;
      /// \brief the number of non-squared error terms in this optimization problem
      virtual size_t numNonSquaredErrorTermsImplementation() const = 0;
      /// \brief get error term n
      virtual ErrorTerm* errorTermImplementation(size_t n) = 0;
      /// \brief get non-squared error term n
      virtual ScalarNonSquaredErrorTerm* nonSquaredErrorTermImplementation(size_t n) = 0;
      /// \brief get error term n
      virtual const ErrorTerm* errorTermImplementation(size_t n) const = 0;
      /// \brief get non-squared error term n
      virtual const ScalarNonSquaredErrorTerm* nonSquaredErrorTermImplementation(size_t n) const = 0;

      /// \brief get all of the error terms associated with design variable dv.
      virtual void getErrorsImplementation(const DesignVariable* dv, std::set<ErrorTerm*>& outErrorSet) = 0;
      /// \brief get all of the non-squared error terms associated with design variable dv.
      virtual void getNonSquaredErrorsImplementation(const DesignVariable* dv, std::set<ScalarNonSquaredErrorTerm*>& outErrorSet) = 0;

    };


  } // namespace backend
} // namespace aslam


#endif /* ASLAM_OPTIMIZATION_PROBLEM_HPP */

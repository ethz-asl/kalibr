/*
 * ProblemManager.hpp
 *
 *  Created on: 10.08.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_ASLAM_BACKEND_PROBLEMMANAGER_HPP_
#define INCLUDE_ASLAM_BACKEND_PROBLEMMANAGER_HPP_

#include <vector>

#include <boost/shared_ptr.hpp>

#include "CommonDefinitions.hpp"
#include "CostFunctionInterface.hpp"

#include "../../Exceptions.hpp"
#include "../JacobianContainerDense.hpp"
#include "../JacobianContainerSparse.hpp"

namespace aslam {
namespace backend {

// Forward declarations
class OptimizationProblemBase;
class ErrorTerm;
class ScalarNonSquaredErrorTerm;
class DesignVariable;

/**
 * \class ProblemManager
 * Utility class to collect functionality for dealing with a problem.
 */
class ProblemManager {
 public:
  SM_DEFINE_EXCEPTION(Exception, aslam::Exception);

  /// \brief Constructor with default options
  ProblemManager();

  /// \brief Convenience constructor instead of setProblem() + initialize()
  ProblemManager(boost::shared_ptr<OptimizationProblemBase> problem);

  /// \brief Destructor
  virtual ~ProblemManager();

  /// \brief Set up to work on the optimization problem.
  void setProblem(boost::shared_ptr<OptimizationProblemBase> problem);

  /// \brief initialize the class
  virtual void initialize();

  /// \brief Is everything initialized?
  bool isInitialized() const { return _isInitialized; }

  /// \brief Mutable getter for the optimization problem
  boost::shared_ptr<OptimizationProblemBase> getProblem() { return _problem; }

  /// \brief Const getter for the optimization problem
  boost::shared_ptr<const OptimizationProblemBase> getProblem() const { return _problem; }

  /// \brief Get dense design variable i.
  DesignVariable* designVariable(size_t i);

  /// \brief how many dense design variables are involved in the problem
  size_t numDesignVariables() const { return _designVariables.size(); };

  /// \brief Get the design variables
  const std::vector<DesignVariable*>& designVariables() const { return _designVariables; };

  /// \brief how many scalar parameters (design variables with their minimal dimension)
  ///        are involved in the problem
  size_t numOptParameters() const { return _numOptParameters; }

  /// \brief how many error terms are involved in the problem
  size_t numErrorTerms() const { return _numErrorTerms; }

  /// \brief return the total dimension of all squared error terms together
  size_t getTotalDimSquaredErrorTerms() const { return _dimErrorTermsS; }

  /// \brief Do a bunch of checks to see if the problem is well-defined. This includes checking that every error term is
  ///        hooked up to design variables and running finite differences on error terms where this is possible.
  void checkProblemSetup() const;

  /// \brief Evaluate the value of the objective function
  double evaluateError(const size_t nThreads = 1) const;

  /// \brief Signal that the problem changed.
  void signalProblemChanged() { setInitialized(false); }

  /// \brief Apply the update vector to the design variables
  void applyStateUpdate(const ColumnVectorType& dx);

  /// \brief Undo the last state update to the design variables
  void revertLastStateUpdate();

  /// \brief Save the current state of the design variables
  void saveDesignVariables();
  /// \brief Revert to the last state saved by a call to saveDesignVariables()
  void restoreDesignVariables();

  /// \brief Returns a flattened version of the design variables' parameters
  Eigen::VectorXd getFlattenedDesignVariableParameters() const;

  /// \brief compute the current gradient of the objective function
  void computeGradient(RowVectorType& outGrad, size_t nThreads, bool useMEstimator, bool applyDvScaling, bool useDenseJacobianContainer);

  /// \brief Apply the scaling of the design variables to \p outGrad
  void applyDesignVariableScaling(RowVectorType& outGrad) const;

  /// \brief computes the gradient of a specific error term
  void addGradientForErrorTerm(RowVectorType& J, ErrorTerm* e, bool useMEstimator, bool useDenseJacobianContainer);
  void addGradientForErrorTerm(JacobianContainerSparse<1>& jc, RowVectorType& J, ScalarNonSquaredErrorTerm* e, bool useMEstimator);
  void addGradientForErrorTerm(JacobianContainerDense<RowVectorType&, 1>& jc, ScalarNonSquaredErrorTerm* e, bool useMEstimator);

  const std::vector<ErrorTerm*>& getErrorTerms() const {
    return _errorTermsS;
  }
 protected:
  /// \brief Set the initialized status
  void setInitialized(bool isInitialized) { _isInitialized = isInitialized; }

 private:
  /// \brief Evaluate the gradient of the objective function
  void evaluateGradients(size_t threadId, size_t startIdx, size_t endIdx, RowVectorType& grad, bool useMEstimator, bool useDenseJacobianContainer);

  /// \brief Evaluate the objective function
  void sumErrorTerms(size_t /* threadId */, size_t startIdx, size_t endIdx, double& err) const;

 private:

  /// \brief The current optimization problem.
  boost::shared_ptr<OptimizationProblemBase> _problem;

  /// \brief all design variables...first the non-marginalized ones (the dense ones), then the marginalized ones.
  std::vector<DesignVariable*> _designVariables;

   /// \brief State of the design variables, will only be filled upon saveDesignVariables()
  std::vector< std::pair<DesignVariable*, Eigen::MatrixXd> > _dvState;

  /// \brief all of the error terms involved in this problem
  std::vector<ErrorTerm*> _errorTermsS;
  std::vector<ScalarNonSquaredErrorTerm*> _errorTermsNS;

  /// \brief the total number of parameters of this problem, given by number of design variables and their dimensionality
  std::size_t _numOptParameters = 0;

  /// \brief the total number of error terms as the sum of squared and non-squared error terms
  std::size_t _numErrorTerms = 0;

  /// \brief the total dimension of squared error terms
  std::size_t _dimErrorTermsS = 0;

  /// \brief Whether the optimizer is correctly initialized
  bool _isInitialized = false;

};

namespace details
{
  template <bool IS_REFERENCE=false>
  struct CostFunctionParameterTraits { typedef const bool const_bool_t; typedef const std::size_t const_size_t; };
  template <>
  struct CostFunctionParameterTraits<true> { typedef const bool& const_bool_t; typedef const std::size_t& const_size_t; };
}

/// \brief Creates a cost function from a problem manager
/// \tparam UseReference Use true iff parameters shall be stored as references
template <bool UseMEstimatorRef = false, bool UseDenseJacobianContainerRef = false,
    bool ApplyDvScalingRef = false, bool NumThreadsGradientRef = false, bool NumThreadsErrorRef = false>
inline boost::shared_ptr<CostFunctionInterface> getCostFunction(ProblemManager& pm,
                                                                const bool& useMEstimator,
                                                                const bool& useDenseJacobianContainer,
                                                                const bool& applyDvScaling,
                                                                const std::size_t& numThreadsGradient,
                                                                const std::size_t& numThreadsError)
{

  /// \brief Implements CostFunctionInterface via ProblemManager
  struct CostFunctionPM : public CostFunctionInterface
  {
    CostFunctionPM(ProblemManager& pm,
                   const bool& useMEstimator,
                   const bool& useDenseJacobianContainer,
                   const bool& applyDvScaling,
                   const std::size_t& numThreadsGradient,
                   const std::size_t& numThreadsError)
        : _pm(pm),
          _useMEstimator(useMEstimator),
          _useDenseJacobianContainer(useDenseJacobianContainer),
          _applyDvScaling(applyDvScaling),
          _numThreadsGradient(numThreadsGradient),
          _numThreadsError(numThreadsError)
    {

    }
    ~CostFunctionPM() { }
    double evaluateError() const override { return _pm.evaluateError(_numThreadsError); }
    void computeGradient(RowVectorType& gradient) override { _pm.computeGradient(gradient, _numThreadsGradient, _useMEstimator, _applyDvScaling, _useDenseJacobianContainer); }
    const std::vector<DesignVariable*>& getDesignVariables() override { return _pm.designVariables(); };
   private:
    ProblemManager& _pm;
    typename details::CostFunctionParameterTraits<UseMEstimatorRef>::const_bool_t _useMEstimator;
    typename details::CostFunctionParameterTraits<UseDenseJacobianContainerRef>::const_bool_t _useDenseJacobianContainer;
    typename details::CostFunctionParameterTraits<ApplyDvScalingRef>::const_bool_t _applyDvScaling;
    typename details::CostFunctionParameterTraits<NumThreadsGradientRef>::const_size_t _numThreadsGradient;
    typename details::CostFunctionParameterTraits<NumThreadsErrorRef>::const_size_t _numThreadsError;
  };

  return boost::shared_ptr<CostFunctionInterface>(new CostFunctionPM(pm, useMEstimator, useDenseJacobianContainer, applyDvScaling,
                                                                     numThreadsGradient, numThreadsError));

}


} // namespace backend
} // namespace aslam

#endif /* INCLUDE_ASLAM_BACKEND_PROBLEMMANAGER_HPP_ */

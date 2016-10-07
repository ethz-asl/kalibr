#include <aslam/backend/util/ProblemManager.hpp>
#include <aslam/backend/OptimizationProblemBase.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>
#include <aslam/backend/JacobianContainerSparse.hpp>
#include <aslam/backend/JacobianContainerDense.hpp>

#include <aslam/backend/util/ThreadedRangeProcessor.hpp>


#include <sm/logging.hpp>

namespace aslam {
namespace backend {

ProblemManager::ProblemManager()
{

}

ProblemManager::ProblemManager(boost::shared_ptr<OptimizationProblemBase> problem)
{
  setProblem(problem);
  initialize();
}

ProblemManager::~ProblemManager()
{
}

/// \brief Set up to work on the optimization problem.
void ProblemManager::setProblem(boost::shared_ptr<OptimizationProblemBase> problem)
{
  _problem = problem;
  _isInitialized = false;
}

/// \brief initialize the class
void ProblemManager::initialize()
{

  SM_ASSERT_FALSE(Exception, _problem == nullptr, "No optimization problem has been set");
  Timer init("ProblemManager: Initialize total");
  _designVariables.clear();
  _designVariables.reserve(_problem->numDesignVariables());
  _errorTermsNS.clear();
  _errorTermsNS.reserve(_problem->numNonSquaredErrorTerms());
  _errorTermsS.clear();
  _errorTermsS.reserve(_problem->numErrorTerms());
  Timer initDv("ProblemManager: Initialize design Variables");
  // Run through all design variables adding active ones to an active list.
  for (size_t i = 0; i < _problem->numDesignVariables(); ++i) {
    DesignVariable* dv = _problem->designVariable(i);
    if (dv->isActive())
      _designVariables.push_back(dv);
  }
  SM_ASSERT_FALSE(Exception, _problem->numDesignVariables() > 0 && _designVariables.empty(),
                  "It is illegal to run the optimizer with all marginalized design variables. Did you forget to set the design variables as active?");
  SM_ASSERT_FALSE(Exception, _designVariables.empty(), "It is illegal to run the optimizer with all marginalized design variables.");
  // Assign block indices to the design variables.
  // "blocks" will hold the structure of the left-hand-side of Gauss-Newton
  _numOptParameters = 0;
  for (size_t i = 0; i < _designVariables.size(); ++i) {
    _designVariables[i]->setBlockIndex(i);
    _designVariables[i]->setColumnBase(_numOptParameters);
    _numOptParameters += _designVariables[i]->minimalDimensions();
  }
  initDv.stop();

  Timer initEt("ProblemManager: Initialize error terms");
  // Get all of the error terms that work on these design variables.
  _numErrorTerms = 0;
  for (unsigned i = 0; i < _problem->numNonSquaredErrorTerms(); ++i) {
    ScalarNonSquaredErrorTerm* e = _problem->nonSquaredErrorTerm(i);
    _errorTermsNS.push_back(e);
    _numErrorTerms++;
  }
  _dimErrorTermsS = 0;
  for (unsigned i = 0; i < _problem->numErrorTerms(); ++i) {
    ErrorTerm* e = _problem->errorTerm(i);
    _errorTermsS.push_back(e);
    e->setRowBase(_dimErrorTermsS);
    _dimErrorTermsS += e->dimension();
    _numErrorTerms++;
  }
  initEt.stop();
  SM_ASSERT_FALSE(Exception, _errorTermsNS.empty() && _errorTermsS.empty(), "It is illegal to run the optimizer with no error terms.");

  _isInitialized = true;

  SM_FINEST_STREAM_NAMED("optimization",
                         "ProblemManager: Initialized problem with " << _problem->numDesignVariables() <<
                         " design variable(s), " << _errorTermsNS.size() << " non-squared error term(s) and " <<
                         _errorTermsS.size() << " squared error term(s)");

}

DesignVariable* ProblemManager::designVariable(size_t i)
{
  SM_ASSERT_LT_DBG(Exception, i, _designVariables.size(), "index out of bounds");
  return _designVariables[i];
}

void ProblemManager::checkProblemSetup() const
{
  // Check that all error terms are hooked up to design variables.
  // TODO: Is this check really necessary? It's not wrong by default, but one could simply remove this error term.
  for (std::size_t i=0; i<_errorTermsNS.size(); i++)
    SM_ASSERT_GT(Exception, _errorTermsNS[i]->numDesignVariables(), 0, "Non-squared error term " << i << " has no design variable(s) attached.");
  for (std::size_t i=0; i<_errorTermsS.size(); i++)
    SM_ASSERT_GT(Exception, _errorTermsS[i]->numDesignVariables(), 0, "Squared error term " << i << " has no design variable(s) attached.");
}

/**
 * Computes the gradient of the scalar objective function
 * @param[out] outGrad The gradient
 * @param nThreads How many threads to use
 * @param useMEstimator Whether to use an MEstimator
 */
void ProblemManager::computeGradient(RowVectorType& outGrad, size_t nThreads, bool useMEstimator, bool applyDvScaling, bool useDenseJacobianContainer)
{
  SM_ASSERT_GT(Exception, nThreads, 0, "");
  Timer t("ProblemManager: Compute gradient", false);
  std::vector<RowVectorType> gradients(nThreads, RowVectorType::Zero(1, _numOptParameters)); // compute gradients separately in different threads and add in the end
  boost::function<void(size_t, size_t, size_t, RowVectorType&)> job(boost::bind(&ProblemManager::evaluateGradients, this, _1, _2, _3, _4, useMEstimator, useDenseJacobianContainer));
  util::runThreadedFunction(job, _numErrorTerms, gradients);
  // Add up the gradients
  outGrad = gradients[0];
  for (std::size_t i = 1; i<gradients.size(); i++)
    outGrad += gradients[i];
  if (applyDvScaling)
    applyDesignVariableScaling(outGrad);
}

void ProblemManager::applyDesignVariableScaling(RowVectorType& outGrad) const {
  for (const auto dv : _designVariables)
    outGrad.block(0, dv->columnBase(), outGrad.rows(), dv->minimalDimensions()) *= dv->scaling();
}

void ProblemManager::addGradientForErrorTerm(RowVectorType& J, ErrorTerm* e, bool useMEstimator, bool useDenseJacobianContainer) {
  e->updateRawSquaredError();
  ColumnVectorType ev;
  e->getWeightedError(ev, useMEstimator);
  ev *= 2.0;

  if (useDenseJacobianContainer) {
    Eigen::MatrixXd J2 = Eigen::MatrixXd::Zero(e->dimension(), J.cols());
    JacobianContainerDense<Eigen::MatrixXd&, Eigen::Dynamic> jc(J2);
    e->getWeightedJacobians(jc, useMEstimator);
    J += ev.transpose() * J2;
  } else {
    JacobianContainerSparse<Eigen::Dynamic> jc(e->dimension());
    e->getWeightedJacobians(jc, useMEstimator);
    for (const auto& dvJacPair : jc) { // iterate over design variables of this error term
      RowVectorType grad = ev.transpose()*dvJacPair.second;
      J.block(0 /*e->rowBase()*/, dvJacPair.first->columnBase(), grad.rows(), grad.cols()) += grad;
    }
  }
}

void ProblemManager::addGradientForErrorTerm(JacobianContainerSparse<1>& jc, RowVectorType& J, ScalarNonSquaredErrorTerm* e, bool useMEstimator) {
  e->evaluateJacobians(jc, useMEstimator);
  for (const auto& dvJacPair : jc) // iterate over design variables of this error term
    J.block(0 /*e->rowBase()*/, dvJacPair.first->columnBase(), dvJacPair.second.rows(), dvJacPair.second.cols()) += dvJacPair.second;
}

void ProblemManager::addGradientForErrorTerm(JacobianContainerDense<RowVectorType&, 1>& jc, ScalarNonSquaredErrorTerm* e, bool useMEstimator) {
  e->evaluateJacobians(jc, useMEstimator);
}


double ProblemManager::evaluateError(const size_t nThreads /*= 1*/) const {

  std::vector<double> errors(nThreads, 0.0);
  boost::function<void(size_t, size_t, size_t, double&)> job(boost::bind(&ProblemManager::sumErrorTerms, this, _1, _2, _3, _4));
  util::runThreadedFunction(job, _numErrorTerms, errors);

  double error = 0.0;
  for (auto e : errors)
    error += e;

  return error;

}


void ProblemManager::applyStateUpdate(const ColumnVectorType& dx)
{
  Timer t("ProblemManager: Apply state update", false);
  // Apply the update to the dense state.
  int startIdx = 0;
  for (size_t i = 0; i < _designVariables.size(); i++) {
    DesignVariable* d = _designVariables[i];
    const int dbd = d->minimalDimensions();
    Eigen::VectorXd dxS = dx.segment(startIdx, dbd);
    dxS *= d->scaling();
    d->update(&dxS[0], dbd);
    startIdx += dbd;
  }
}

void ProblemManager::revertLastStateUpdate()
{
  Timer t("ProblemManager: Revert last state update", false);
  for (size_t i = 0; i < _designVariables.size(); i++)
    _designVariables[i]->revertUpdate();
}

void ProblemManager::saveDesignVariables() {
  _dvState.resize(numDesignVariables());
  for (size_t i = 0; i < numDesignVariables(); i++) {
    _dvState[i].first = designVariable(i);
    _dvState[i].first->getParameters(_dvState[i].second);
  }
}

void ProblemManager::restoreDesignVariables() {
  for (auto& dvParamPair : _dvState)
    dvParamPair.first->setParameters(dvParamPair.second);
}

Eigen::VectorXd ProblemManager::getFlattenedDesignVariableParameters() const {

  // Allocate memory so vector-space design variables fit into the output vector
  int numParametersMin = 0;
  for (auto& dv : _designVariables)
    numParametersMin += dv->minimalDimensions();

  Eigen::VectorXd rval(numParametersMin);

  int cnt = 0;
  for (auto& dv : _designVariables) {
    Eigen::MatrixXd p;
    dv->getParameters(p);
    int d = p.size();
    p.conservativeResize(d, 1);

    // Resize has to be performed if we have non-vector-space design variables
    int newSize = cnt + d;
    if (newSize > numParametersMin)
      rval.conservativeResize(newSize);

    rval.segment(cnt, d) = p;
    cnt += d;
  }

  return rval;

}

void ProblemManager::sumErrorTerms(size_t /* threadId */, size_t startIdx, size_t endIdx, double& err) const {
  SM_ASSERT_LE_DBG(Exception, endIdx, _numErrorTerms, "");
  for (size_t i = startIdx; i < endIdx; ++i) { // iterate through error terms
    if (i < _errorTermsNS.size())
      err += _errorTermsNS[i]->evaluateError();
    else
      err += _errorTermsS[i - _errorTermsNS.size()]->evaluateError();
  }
}

/**
 * Evaluate the gradient of the objective function
 * @param
 * @param startIdx First error term index (including)
 * @param endIdx Last error term index (excluding)
 * @param useMEstimator Whether or not to use an MEstimator
 * @param J The gradient for the specified error terms
 */
void ProblemManager::evaluateGradients(size_t /* threadId */, size_t startIdx, size_t endIdx, RowVectorType& J, bool useMEstimator, bool useDenseJacobianContainer)
{
  SM_ASSERT_LE_DBG(Exception, endIdx, _numErrorTerms, "");

  size_t cnt = startIdx;

  // process non-squared error terms
  if (useDenseJacobianContainer)
  {
    JacobianContainerDense<RowVectorType&, 1> jc(J);
    for (; cnt < endIdx && cnt < _errorTermsNS.size(); ++cnt)
      addGradientForErrorTerm(jc, _errorTermsNS[cnt], useMEstimator);
  }
  else
  {
    JacobianContainerSparse<1> jc(1);
    for (; cnt < endIdx && cnt < _errorTermsNS.size(); ++cnt)
    {
      jc.clear();
      addGradientForErrorTerm(jc, J, _errorTermsNS[cnt], useMEstimator);
    }
  }

  // process squared error terms
  for (; cnt < endIdx; ++cnt)
  {
    addGradientForErrorTerm(J, _errorTermsS[cnt - _errorTermsNS.size()], useMEstimator, useDenseJacobianContainer);
  }

}

} // namespace backend
} // namespace aslam

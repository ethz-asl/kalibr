#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/util/ProblemManager.hpp>
#include <aslam/backend/SimpleOptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost::python;
using namespace aslam::backend;


DesignVariable * (OptimizationProblemBase::*dvptr)(size_t) = &OptimizationProblemBase::designVariable;

ErrorTerm * (OptimizationProblemBase::*etptr)(size_t i) = &OptimizationProblemBase::errorTerm;
ScalarNonSquaredErrorTerm * (OptimizationProblemBase::*nsetptr)(size_t i) = &OptimizationProblemBase::nonSquaredErrorTerm;

void (OptimizationProblem::*adv)( boost::shared_ptr<DesignVariable>) = &OptimizationProblem::addDesignVariable;

void (OptimizationProblem::*aet)( const boost::shared_ptr<ErrorTerm> &) = &OptimizationProblem::addErrorTerm;

void (OptimizationProblem::*asnset)( const boost::shared_ptr<ScalarNonSquaredErrorTerm> &) = &OptimizationProblem::addErrorTerm;

void (SimpleOptimizationProblem::*sadv)( boost::shared_ptr<DesignVariable>) = &SimpleOptimizationProblem::addDesignVariable;

void (SimpleOptimizationProblem::*saet)( const boost::shared_ptr<ErrorTerm> &) = &SimpleOptimizationProblem::addErrorTerm;

void (SimpleOptimizationProblem::*sasnset)( const boost::shared_ptr<ScalarNonSquaredErrorTerm> &) = &SimpleOptimizationProblem::addErrorTerm;

RowVectorType computeGradientForScalarNonSquaredErrorTerm(boost::shared_ptr<OptimizationProblemBase> problem, ScalarNonSquaredErrorTerm* errorTerm, bool useMEstimator = true) {

  ProblemManager pm;
  pm.setProblem(problem);
  pm.initialize();

  RowVectorType J = RowVectorType::Zero(1, pm.numOptParameters());
  JacobianContainerDense<RowVectorType&, 1> jc(J);
  pm.addGradientForErrorTerm(jc, errorTerm, useMEstimator);
  return J;
}

RowVectorType computeGradientForErrorTerm(boost::shared_ptr<OptimizationProblemBase> problem, ErrorTerm* errorTerm, bool useMEstimator = true, bool useDenseJacobianContainer = true) {

  ProblemManager pm;
  pm.setProblem(problem);
  pm.initialize();

  RowVectorType J = RowVectorType::Zero(1, pm.numOptParameters());
  pm.addGradientForErrorTerm(J, errorTerm, useMEstimator, useDenseJacobianContainer);
  return J;
}

RowVectorType computeGradient(boost::shared_ptr<OptimizationProblemBase> problem, std::size_t nThreads = 2, bool useMEstimator = true, bool applyDvScaling = true, bool useDenseJacobianContainer = true) {

  ProblemManager pm;
  pm.setProblem(problem);
  pm.initialize();

  RowVectorType J = RowVectorType::Zero(1, pm.numOptParameters());
  pm.computeGradient(J, nThreads, useMEstimator, applyDvScaling, useDenseJacobianContainer);
  return J;
}

double evaluateError(boost::shared_ptr<OptimizationProblemBase> problem, std::size_t nThreads = 2) {
  ProblemManager pm;
  pm.setProblem(problem);
  pm.initialize();
  return pm.evaluateError(nThreads);
}

BOOST_PYTHON_FUNCTION_OVERLOADS(computeGradientForErrorTerm_overloads, computeGradientForErrorTerm, 2, 4);
BOOST_PYTHON_FUNCTION_OVERLOADS(computeGradientForScalarNonSquaredErrorTerm_overloads, computeGradientForScalarNonSquaredErrorTerm, 2, 3);
BOOST_PYTHON_FUNCTION_OVERLOADS(computeGradient_overloads, computeGradient, 1, 5);
BOOST_PYTHON_FUNCTION_OVERLOADS(evaluateError_overloads, evaluateError, 1, 2);

void exportOptimizationProblem()
{

  class_<OptimizationProblemBase, boost::shared_ptr<OptimizationProblemBase>, boost::noncopyable>("OptimizationProblemBase",no_init)
      /// \brief The number of design variables stored in this optimization problem
    .def("numDesignVariables", &OptimizationProblemBase::numDesignVariables)
      /// \brief Get design variable i
    .def("designVariable", dvptr, return_internal_reference<>())
    /// \brief the number of error terms stored in this optimization problem
    .def("numErrorTerms", &OptimizationProblemBase::numErrorTerms)
    /// \brief the number of non-squared error terms stored in this optimization problem
    .def("numNonSquaredErrorTerms", &OptimizationProblemBase::numNonSquaredErrorTerms)
    /// \brief get error term i.
    .def("errorTerm", etptr, return_internal_reference<>())
    /// \brief get scalar non squared error term i
    .def("scalarNonSquaredErrorTerm", nsetptr, return_internal_reference<>())
    /// \brief get the gradient contribution of the scalar non squared error term i
    .def("computeGradientForScalarNonSquaredErrorTerm", &computeGradientForScalarNonSquaredErrorTerm,
         computeGradientForScalarNonSquaredErrorTerm_overloads("computeGradientForScalarNonSquaredErrorTerm(ScalarNonSquaredErrorTerm e, bool useMEstimator)"))
    /// \brief get the gradient contribution of the scalar non squared error term i
    .def("computeGradientForErrorTerm", &computeGradientForErrorTerm,
         computeGradientForErrorTerm_overloads("computeGradientForErrorTerm(ErrorTerm e, int nThreads, bool useMEstimator)"))
    /// \brief compute the full gradient
    .def("computeGradient", &computeGradient, computeGradient_overloads("computeGradient(int nThreads, bool useMEstimator, bool applyDvScaling)"))
    /// \brief Evaluates the full error
    .def("evaluateError", &evaluateError, evaluateError_overloads("evaluateError(int nThreads)"))
    ;

  class_<OptimizationProblem, boost::shared_ptr<OptimizationProblem>, bases<OptimizationProblemBase> >("OptimizationProblem", init<>())
    /// \brief Add a design variable to the problem.
    .def("addDesignVariable", adv)
    /// \brief Add an error term to the problem
    .def("addErrorTerm", aet)
    /// \brief Add a scalar non-squared error term to the problem
    .def("addScalarNonSquaredErrorTerm", asnset)
    /// \brief clear the design variables and error terms.
    .def("clear", &OptimizationProblem::clear)
    /// \brief remove an error term:
    .def("removeErrorTerm", &OptimizationProblem::removeErrorTerm)
    ;

  class_<SimpleOptimizationProblem, boost::shared_ptr<SimpleOptimizationProblem>, bases<OptimizationProblemBase> >("SimpleOptimizationProblem", init<>())
      /// \brief Add a design variable to the problem.
      .def("addDesignVariable", sadv)
      /// \brief Add an error term to the problem
      .def("addErrorTerm", saet)
      /// \brief Add a scalar non-squared error term to the problem
      .def("addScalarNonSquaredErrorTerm", sasnset)
      /// \brief clear the design variables and error terms.
      .def("clear", &SimpleOptimizationProblem::clear)
    ;

}

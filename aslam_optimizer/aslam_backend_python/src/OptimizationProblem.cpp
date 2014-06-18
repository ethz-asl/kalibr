#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/SimpleOptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost::python;
using namespace aslam::backend;


DesignVariable * (OptimizationProblemBase::*dvptr)(size_t) = &OptimizationProblemBase::designVariable;

ErrorTerm * (OptimizationProblemBase::*etptr)(size_t i) = &OptimizationProblemBase::errorTerm;

void (OptimizationProblem::*adv)( boost::shared_ptr<DesignVariable>) = &OptimizationProblem::addDesignVariable;

void (OptimizationProblem::*aet)( const boost::shared_ptr<ErrorTerm> &) = &OptimizationProblem::addErrorTerm;

void (SimpleOptimizationProblem::*sadv)( boost::shared_ptr<DesignVariable>) = &SimpleOptimizationProblem::addDesignVariable;

void (SimpleOptimizationProblem::*saet)( const boost::shared_ptr<ErrorTerm> &) = &SimpleOptimizationProblem::addErrorTerm;



void exportOptimizationProblem()
{

  class_<OptimizationProblemBase, boost::shared_ptr<OptimizationProblemBase>, boost::noncopyable>("OptimizationProblemBase",no_init)
      /// \brief The number of design variables stored in this optimization problem
    .def("numDesignVariables", &OptimizationProblemBase::numDesignVariables)
      /// \brief Get design variable i
    .def("designVariable", dvptr, return_internal_reference<>())
    /// \brief the number of error terms stored in this optimization problem
    .def("numErrorTerms", &OptimizationProblemBase::numErrorTerms)
    /// \brief get error term i.
    .def("errorTerm", etptr, return_internal_reference<>())
    ;

  class_<OptimizationProblem, boost::shared_ptr<OptimizationProblem>, bases<OptimizationProblemBase> >("OptimizationProblem", init<>())
      /// \brief Add a design variable to the problem.
.def("addDesignVariable", adv)
      /// \brief Add an error term to the problem
.def("addErrorTerm", aet)      
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
      /// \brief clear the design variables and error terms.
      .def("clear", &SimpleOptimizationProblem::clear)
    ;

}

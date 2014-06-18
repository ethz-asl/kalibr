#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost::python;
using namespace aslam::backend;

DesignVariable * (ErrorTerm::*err_dvptr)(size_t) = &ErrorTerm::designVariable;


void exportErrorTerm()
{
  
  class_<ErrorTerm, boost::shared_ptr<ErrorTerm>, boost::noncopyable>("ErrorTerm", no_init)
    /// \brief evaluate the error term.
    ///        After this is called, the _squaredError is filled in with \f$ \mathbf e^T \mathbf R^{-1} \mathbf e \f$
    .def("evaluateError", &ErrorTerm::evaluateError)

      .def("invR", &ErrorTerm::vsInvR)

      .def("setInvR", &ErrorTerm::vsSetInvR)


    /// \brief evaluate the Jacobians.
    .def("evaluateJacobians", &ErrorTerm::evaluateJacobians)

    /// \brief set the M-Estimator policy. This function takes a squared error
    ///        and returns a weight to apply to that error term.
    .def("setMEstimatorPolicy", &ErrorTerm::setMEstimatorPolicy)
      
    /// \brief clear the m-estimator policy.
    .def("clearMEstimatorPolicy", &ErrorTerm::clearMEstimatorPolicy)
      
    /// \brief compute the M-estimator weight from a squared error.
    .def("getMEstimatorWeight", &ErrorTerm::getMEstimatorWeight)
      .def("getCurrentMEstimatorWeight", &ErrorTerm::getCurrentMEstimatorWeight)
      
    /// \brief get the name of the M-Estimator.
    .def("getMEstimatorName", &ErrorTerm::getMEstimatorName)

    /// \brief build this error term's part of the Hessian matrix.
    /// 
    /// the i/o variables outHessian and outRhs are the full Hessian and rhs in the Gauss-Newton
    /// problem. The correct blocks for each design varible are available from the design
    /// variable as dv.blockIndex()
    .def("buildHessian", &ErrorTerm::buildHessian)
      
    /// \brief How many design varibles is this error term connected to?
    .def("numDesignVariables", &ErrorTerm::numDesignVariables)
      
    /// \brief Get design variable i.
    .def("designVariable", err_dvptr, return_internal_reference<>())

    .def("error", &ErrorTerm::vsError)
	.def("getRawSquaredError", &ErrorTerm::getRawSquaredError)
	.def("getWeightedSquaredError", &ErrorTerm::getWeightedSquaredError)
      .def("dimension", &ErrorTerm::dimension)
;

  
  ;

}

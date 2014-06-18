#ifndef ASLAM_PYTHON_EXPORT_BACKEND_EXPRESSION_HPP
#define ASLAM_PYTHON_EXPORT_BACKEND_EXPRESSION_HPP
#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/JacobianContainer.hpp>
#include <sstream>

namespace aslam {
	namespace python {

	using namespace aslam::backend;
	using namespace boost::python;

	template<typename ExpressionType>
	boost::python::list getDesignVariables(const ExpressionType * et)
	{
		JacobianContainer::set_t dv;
		et->getDesignVariables(dv);

		JacobianContainer::set_t::iterator it = dv.begin();
        boost::python::list dvlist;
		for( ; it != dv.end(); ++it)
		  {
			// \todo this (the null_deleter) could be dangerous if the objects go out of scope while Python has them.
			boost::shared_ptr<DesignVariable> val(*it, sm::null_deleter());
			dvlist.append(val);
		  }
		return dvlist;
	}

	template<typename ExpressionType>
	void evaluateJacobians1(const ExpressionType * et, JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule)
	{
		et->evaluateJacobians(outJacobians, applyChainRule);
	}

	template<typename ExpressionType>
	void evaluateJacobians2(const ExpressionType * et, JacobianContainer & outJacobians)
	{
		et->evaluateJacobians(outJacobians);
	}

	}


}


#endif /* ASLAM_PYTHON_EXPORT_BACKEND_EXPRESSION_HPP */

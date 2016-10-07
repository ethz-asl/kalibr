/*
 * L1Regularizer.cpp
 *
 *  Created on: 22.09.2015
 *      Author: Ulrich Schwesinger
 */

#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <aslam/backend/L1Regularizer.hpp>
#include <boost/shared_ptr.hpp>

using namespace boost::python;
using namespace aslam::backend;

void exportL1Regularizer()
{

  typedef std::vector<Scalar*> ScalarDesignVariables;

  class_<ScalarDesignVariables>("ScalarDesignVariableVector")
    .def(boost::python::vector_indexing_suite<ScalarDesignVariables>() )
    .def("__iter__", boost::python::iterator<ScalarDesignVariables>())
  ;

  class_<L1Regularizer, boost::shared_ptr<L1Regularizer>, bases<ScalarNonSquaredErrorTerm>, boost::noncopyable>("L1Regularizer", no_init)
    .def(init<const ScalarDesignVariables&, const double>())
    .def("setBeta", &L1Regularizer::setBeta)
  ;

}

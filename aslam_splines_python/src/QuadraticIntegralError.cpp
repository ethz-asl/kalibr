/*
 * QuadraticIntegralErrorTerm.cpp
 *
 *  Created on: Dec 4, 2013
 *      Author: hannes
 */


#include <sstream>
#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>
#include <Eigen/Core>
#include <aslam/backend/QuadraticIntegralError.hpp>

using namespace boost::python;

template <int dim, typename Expression = aslam::backend::VectorExpression<dim>>
inline void addQuadraticIntegralExpressionErrorTerms(aslam::backend::OptimizationProblem & problem, double a, double b, int numberOfPoints, const object & expressionFactory, const Eigen::Matrix<double, dim, dim> & sqrtInvR){
  using namespace aslam::backend::integration;

  struct ExpressionFactoryAdapter {
    ExpressionFactoryAdapter(const object & o) : o(o){}

    Expression operator()(double time) const {
      return extract<Expression>(o(time));
    }
   private:
    const object & o;
  };

  addQuadraticIntegralExpressionErrorTerms(problem, a, b, numberOfPoints, ExpressionFactoryAdapter(expressionFactory), sqrtInvR);
}

const char * BaseName = "addQuadraticIntegralExpressionErrorTermsToProblem";

template <int dimMax>
void defAddQuadraticIntegralExpressionErrorTerms(){
  std::stringstream s;
  s << BaseName << dimMax;

  boost::python::def(
      s.str().c_str(),
      &addQuadraticIntegralExpressionErrorTerms<dimMax>,
      boost::python::args("problem, timeA, timeB, nIntegrationPoints, expressionFactory, sqrtInvR")
   );
  defAddQuadraticIntegralExpressionErrorTerms<dimMax -1>();
}

template <>
void defAddQuadraticIntegralExpressionErrorTerms<0>(){
}

void exportAddQuadraticIntegralExpressionErrorTerms(){
  defAddQuadraticIntegralExpressionErrorTerms<3>();

  boost::python::def(
      "addQuadraticIntegralEuclideanExpressionErrorTermsToProblem",
      &addQuadraticIntegralExpressionErrorTerms<3, aslam::backend::EuclideanExpression>,
      boost::python::args("problem, timeA, timeB, nIntegrationPoints, expressionFactory, sqrtInvR")
   );
}

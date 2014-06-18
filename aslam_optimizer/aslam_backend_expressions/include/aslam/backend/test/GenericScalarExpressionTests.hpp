#ifndef GENERIC_SCALAR_EXPRESSIONTESTS_HPP_
#define GENERIC_SCALAR_EXPRESSIONTESTS_HPP_

#include <aslam/backend/test/ExpressionTests.hpp>
#include <aslam/backend/GenericScalarExpression.hpp>
#include <aslam/backend/FixedPointNumber.hpp>

namespace aslam {
namespace backend {
namespace test {

template<typename Scalar_>
struct ExpressionValueTraits<GenericScalarExpression<Scalar_> > {
  typedef Eigen::Matrix<Scalar_, 1, 1> value_t;
};

template<typename Scalar_, std::uintmax_t Divider>
struct ExpressionValueTraits<GenericScalarExpression<FixedPointNumber<Scalar_, Divider> > > {
  typedef Eigen::Matrix<Scalar_, 1, 1> value_t;
};

template<typename Scalar_, std::uintmax_t Divider>
struct ExpressionTraits<GenericScalarExpression<FixedPointNumber<Scalar_, Divider> > > {
  typedef double scalar_t;
  typedef std::numeric_limits<FixedPointNumber<Scalar_, Divider>> Limits;

  static double defaultTolerance() {
    return (double)Limits::epsilon() * 1E2;
  }
  static double defaulEps() {
    return (double)Limits::epsilon() * 10;
  }
};

template<typename Scalar_>
struct ExpressionEvaluationTraits<GenericScalarExpression<Scalar_> > {
  typedef typename ExpressionValueTraits<GenericScalarExpression<Scalar_> >::value_t value_t;
  static value_t evaluate(const GenericScalarExpression<Scalar_> & exp) {
    value_t ret;
    ret(0, 0) = exp.evaluate();
    return ret;
  }
};

template<typename Scalar_, std::uintmax_t Divider>
struct ExpressionEvaluationTraits<GenericScalarExpression<FixedPointNumber<Scalar_, Divider> > > {
  typedef typename ExpressionValueTraits<GenericScalarExpression<FixedPointNumber<Scalar_, Divider> > >::value_t value_t;
  static value_t evaluate(const GenericScalarExpression<FixedPointNumber<Scalar_, Divider> > & exp) {
    value_t ret;
    ret(0, 0) = exp.evaluate().getNumerator();
    return ret;
  }
};

template<typename Scalar_, std::uintmax_t Divider>
struct ExpressionNumDiffTraits<GenericScalarExpression<FixedPointNumber<Scalar_, Divider> > > {
  typedef GenericScalarExpression<FixedPointNumber<Scalar_, Divider> > TExpression;
  typedef typename ExpressionValueTraits<TExpression>::value_t value_t;
  typedef typename ExpressionTraits<TExpression>::scalar_t scalar_t;
  inline static Eigen::MatrixXd  numericallyCalcJacobian(TExpression expression, int jacobianCols, std::vector<DesignVariable*>dvs, double eps) {
      typename ExpressionNodeFunctor<TExpression>::input_t dp(jacobianCols);
      dp.setZero();
      sm::eigen::NumericalDiff<ExpressionNodeFunctor<TExpression>> numdiff(ExpressionNodeFunctor<TExpression>(expression, dvs), scalar_t(eps));
      return numdiff.estimateJacobian(dp).template cast<double>() / (double)Divider;
  }
};


}
}
}

#endif /* GENERIC_SCALAR_EXPRESSIONTESTS_HPP_ */

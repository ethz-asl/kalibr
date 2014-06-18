/*
 * ExpressionTests.hpp
 *
 *  Created on: Jul 16, 2013
 *      Author: hannes
 */

#ifndef EXPRESSIONTESTS_HPP_
#define EXPRESSIONTESTS_HPP_

#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <aslam/backend/JacobianContainer.hpp>

namespace aslam {
namespace backend {
namespace test {

template <typename TExpression>
struct ExpressionValueTraits {
  typedef typename TExpression::value_t value_t;
};

template <typename TExpression>
struct ExpressionTraits {
  typedef typename ExpressionValueTraits<TExpression>::value_t::Scalar scalar_t;
  typedef std::numeric_limits<scalar_t> Limits;

  static double defaultTolerance() {
    return Limits::is_integer ? 0.001 : sqrt(Limits::epsilon()) * 1E2;
  }
  static double defaulEps() {
    return Limits::is_integer ? 1 : sqrt(Limits::epsilon()) * 10;
  }
};

template <typename TExpression>
struct ExpressionEvaluationTraits {
  typedef typename ExpressionValueTraits<TExpression>::value_t value_t;
  static value_t evaluate(const TExpression & expr) {
    return expr.evaluate();
  }
};

template <typename TExpression>
struct ExpressionNodeFunctor {
  typedef typename ExpressionValueTraits<TExpression>::value_t value_t;
  typedef typename ExpressionTraits<TExpression>::scalar_t scalar_t;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> input_t;
  typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> jacobian_t;

  ExpressionNodeFunctor(TExpression & exp, const std::vector<DesignVariable*> & dvs)
      : _expression(exp),
        _dvs(dvs) {
  }

  // TODO optimize : this update should directly affect the design variables without having this extra input_t.

  input_t update(const input_t & x, int c, scalar_t delta) {
    input_t xnew = x;
    xnew[c] += delta;
    return xnew;
  }

  value_t operator()(const input_t & dr) {
    int offset = 0;
    for (auto d : _dvs) {
      SM_ASSERT_LE(std::runtime_error, offset + d->minimalDimensions(), dr.size(), "Bug in ExpressionNodeFunctor!");
      d->update((const double *) &dr[offset], d->minimalDimensions());
      offset += d->minimalDimensions();
    }

    auto p = ExpressionEvaluationTraits<TExpression>::evaluate(_expression);

    for (auto d : _dvs) {
      d->revertUpdate();
    }
    return p;
  }

  TExpression & _expression;
  const std::vector<DesignVariable*> & _dvs;
};

template <typename TExpression>
struct ExpressionNumDiffTraits {
  typedef typename ExpressionValueTraits<TExpression>::value_t value_t;
  typedef typename ExpressionTraits<TExpression>::scalar_t scalar_t;
  inline static Eigen::MatrixXd  numericallyCalcJacobian(TExpression expression, int jacobianCols, std::vector<DesignVariable*>dvs, double eps) {
      typename ExpressionNodeFunctor<TExpression>::input_t dp(jacobianCols);
      dp.setZero();
      sm::eigen::NumericalDiff<ExpressionNodeFunctor<TExpression>> numdiff(ExpressionNodeFunctor<TExpression>(expression, dvs), scalar_t(eps));
      return numdiff.estimateJacobian(dp).template cast<double>();
  }
};

template<typename TExpression>
class ExpressionJacobianTestTraits;

template<typename TExpression>
class ExpressionTester {
 public:
  typedef typename ExpressionValueTraits<TExpression>::value_t value_t;
  typedef typename ExpressionTraits<TExpression>::scalar_t scalar_t;


  ExpressionTester(const TExpression & exp, int expectedNumberOfDesignVariables, bool setBlockIndices, bool setActive, bool printResult = false, double tolerance = test::ExpressionTraits<TExpression>::defaultTolerance(), double eps = test::ExpressionTraits<TExpression>::defaulEps())
  : _exp(exp),
    _expectedNumberOfDesignVariables(expectedNumberOfDesignVariables),
    _printResult(printResult),
    _tolerance(tolerance),
    _eps(eps),
    _colIndices(),
    _dvs(prepareAndCheckDesignVariables(exp, setBlockIndices, setActive, _colIndices))
  {
  }

  inline static std::vector<DesignVariable *> prepareAndCheckDesignVariables(const TExpression & exp, bool setBlockIndices, bool setActive, std::vector<int> & colIndices){
    DesignVariable::set_t designVariables;
    exp.getDesignVariables(designVariables);

    std::vector<DesignVariable *> dvs(designVariables.begin(), designVariables.end());
    if(!setBlockIndices){
      sort(dvs.begin(), dvs.end(), [](DesignVariable * a, DesignVariable * b) { return a->blockIndex() < b->blockIndex();});
    }

    int minimalDimensionSum = 0;
    int blockIndex = 0;
    for(DesignVariable * dp : dvs){
      minimalDimensionSum += dp->minimalDimensions();
      colIndices.push_back(minimalDimensionSum);
      if(setBlockIndices){
        dp->setBlockIndex(blockIndex++);
      }
      if(setActive){
        dp->setActive(true);
      }
      else{
        SM_ASSERT_TRUE(std::runtime_error, dp->isActive(), "There are design variables inactive. This is most likely not intended here!");
      }
    }

    SM_ASSERT_EQ_DBG(std::runtime_error, colIndices.size(), designVariables.size(), "");
    return dvs;
  }

  void test(){
    if(_expectedNumberOfDesignVariables > 0)
      ASSERT_EQ(_expectedNumberOfDesignVariables, _dvs.size());
    ExpressionJacobianTestTraits<TExpression>::testJacobian(*this);
  }

  const TExpression & getExp() const { return _exp; }
  bool getPrintResult() const { return _printResult; }
  double getEps() const { return _eps; }
  double getTolerance() const { return _tolerance; }
  const std::vector<DesignVariable *> & getDesignVariables() const { return _dvs; }
  inline Eigen::MatrixXd getJacobian(const JacobianContainer & jc) const { return jc.asDenseMatrix(_colIndices);}
 private:
  TExpression _exp;
  int _expectedNumberOfDesignVariables;
  bool _printResult;
  double _tolerance;
  double _eps;
  std::vector<int> _colIndices;
  std::vector<DesignVariable *> _dvs;
};

template<typename TExpression>
class ExpressionJacobianTestTraits {
 public:
  typedef typename ExpressionValueTraits<TExpression>::value_t value_t;
  typedef typename ExpressionTraits<TExpression>::scalar_t scalar_t;

  static void testJacobian(const ExpressionTester<TExpression> & expressionTester){
    const TExpression & expression = expressionTester.getExp();
    auto val = ExpressionEvaluationTraits<TExpression>::evaluate(expression);
    const size_t rows = val.rows();

    JacobianContainer Jc(rows);
    JacobianContainer Jccr(rows);
    expression.evaluateJacobians(Jc);
    expression.evaluateJacobians(Jccr, Eigen::MatrixXd::Identity(rows, rows));

    auto JcM = expressionTester.getJacobian(Jc);
    int jacobianCols = JcM.cols();
    Eigen::MatrixXd Jest = ExpressionNumDiffTraits<TExpression>::numericallyCalcJacobian(expression, jacobianCols, expressionTester.getDesignVariables(), expressionTester.getEps());

    auto JccrM = expressionTester.getJacobian(Jccr);
    
    sm::eigen::assertNear(Jest, JcM, expressionTester.getTolerance(), SM_SOURCE_FILE_POS, "Testing the Jacobian with finite differences");
    sm::eigen::assertEqual(JcM, JccrM, SM_SOURCE_FILE_POS, "Testing whether chaining identity changes nothing.");

    if(expressionTester.getPrintResult()){
      std::cout << "Jest=\n" << Jest << std::endl;
      std::cout << "Jc=\n" << JcM << std::endl;
      std::cout << "Jccr=\n" << JccrM << std::endl;
    }
  }
};

} // namespace test

template<typename TExpression>
inline void testJacobian(TExpression expression, int expectedNumberOfDesignVariables = -1, bool printResult = false, double tolerance = test::ExpressionTraits<TExpression>::defaultTolerance(), double eps = test::ExpressionTraits<TExpression>::defaulEps()) {
  test::ExpressionTester<TExpression>(expression, expectedNumberOfDesignVariables, false, false, printResult, tolerance, eps).test();
}

/**
 * Tests an expression. So far the number of design variables is checked against expectedNumberOfDesignVariables and the expression's jacobian is compared to a finite difference approach (parameterized by tolerance and eps).
 * This method activates all the expressions design variables and sets their blockIndex().
 * @param expression
 * @param expectedNumberOfDesignVariables
 * @param printResult
 * @param tolerance
 * @param eps
 */
template<typename TExpression>
inline void testExpression(TExpression expression, int expectedNumberOfDesignVariables, bool printResult = false, double tolerance = test::ExpressionTraits<TExpression>::defaultTolerance(), double eps = test::ExpressionTraits<TExpression>::defaulEps()) {
  test::ExpressionTester<TExpression>(expression, expectedNumberOfDesignVariables, true, true, printResult, tolerance, eps).test();
}


}
}
#endif /* EXPRESSIONTESTS_HPP_ */

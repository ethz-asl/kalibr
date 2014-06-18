#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/ScalarExpressionNode.hpp>
#include <sm/boost/null_deleter.hpp>

namespace aslam {
namespace backend {
ScalarExpression::ScalarExpression() {

}

ScalarExpression::ScalarExpression(double value)
    : _root(new ScalarExpressionNodeConstant(value)) {

}

ScalarExpression::ScalarExpression(ScalarExpressionNode * designVariable)
    : _root(designVariable, sm::null_deleter()) {

}

ScalarExpression::ScalarExpression(boost::shared_ptr<ScalarExpressionNode> root)
    : _root(root) {

}

ScalarExpression::~ScalarExpression() {

}

double ScalarExpression::toScalar() const {
  return _root->toScalar();
}

void ScalarExpression::evaluateJacobians(JacobianContainer & outJacobians) const {
  _root->evaluateJacobians(outJacobians);
}

void ScalarExpression::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
  _root->evaluateJacobians(outJacobians, applyChainRule);
}

void ScalarExpression::getDesignVariables(DesignVariable::set_t & designVariables) const {
  _root->getDesignVariables(designVariables);
}

ScalarExpression ScalarExpression::operator+(const ScalarExpression & s) {
  boost::shared_ptr<ScalarExpressionNode> newRoot(new ScalarExpressionNodeAdd(_root, s._root));
  return ScalarExpression(newRoot);
}

ScalarExpression ScalarExpression::operator-(const ScalarExpression & s) {
  boost::shared_ptr<ScalarExpressionNode> newRoot(new ScalarExpressionNodeAdd(_root, s._root, -1.0));
  return ScalarExpression(newRoot);
}

ScalarExpression ScalarExpression::operator-(double s) {
  boost::shared_ptr<ScalarExpressionNode> constant(new ScalarExpressionNodeConstant(-s));
  boost::shared_ptr<ScalarExpressionNode> newRoot(new ScalarExpressionNodeAdd(_root, constant));
  return ScalarExpression(newRoot);

}

ScalarExpression ScalarExpression::operator/(const ScalarExpression & s) {
  boost::shared_ptr<ScalarExpressionNode> newRoot(new ScalarExpressionNodeDivide(_root, s._root));
  return ScalarExpression(newRoot);
}

ScalarExpression ScalarExpression::operator/(double s) {
  boost::shared_ptr<ScalarExpressionNode> constant(new ScalarExpressionNodeConstant(s));
  boost::shared_ptr<ScalarExpressionNode> newRoot(new ScalarExpressionNodeDivide(_root, constant));
  return ScalarExpression(newRoot);
}

ScalarExpression ScalarExpression::operator+(double s) {
  boost::shared_ptr<ScalarExpressionNode> constant(new ScalarExpressionNodeConstant(s));
  boost::shared_ptr<ScalarExpressionNode> newRoot(new ScalarExpressionNodeAdd(_root, constant));
  return ScalarExpression(newRoot);
}

ScalarExpression ScalarExpression::operator*(double s) {
  boost::shared_ptr<ScalarExpressionNode> constant(new ScalarExpressionNodeConstant(s));
  boost::shared_ptr<ScalarExpressionNode> newRoot(new ScalarExpressionNodeMultiply(_root, constant));
  return ScalarExpression(newRoot);
}

ScalarExpression ScalarExpression::operator*(const ScalarExpression & s) {
  boost::shared_ptr<ScalarExpressionNode> newRoot(new ScalarExpressionNodeMultiply(_root, s._root));
  return ScalarExpression(newRoot);
}

}  // namespace backend
}  // namespace aslam

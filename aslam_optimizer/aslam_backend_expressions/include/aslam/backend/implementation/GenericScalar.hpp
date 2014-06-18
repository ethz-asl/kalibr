#include <aslam/backend/Scalar.hpp>

namespace aslam {
namespace backend {

#define _CLASS GenericScalar<Scalar_>
#define MEMBER(RET, DECL) template<typename Scalar_> RET _CLASS::DECL

MEMBER(,GenericScalar(Scalar_ p))
    : _p(p),
      _p_p(p)
{
}

MEMBER(,~GenericScalar()) {
}

MEMBER(void, revertUpdateImplementation()) {
  _p = _p_p;
}

/// \brief Update the design variable.
MEMBER(void, updateImplementation(const double * dp, int /* size */)) {
  _p_p = _p;
  _p += Scalar(double(*dp));
}


MEMBER(Scalar_, toScalarImplementation()) const {
  return _p;
}

MEMBER(void, evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd * applyChainRule)) const {
  if(applyChainRule)
    outJacobians.add(const_cast<_CLASS *>(this), *applyChainRule);
  else {
    Eigen::Matrix<double, 1, 1> J;
    J(0, 0) = 1;
    outJacobians.add(const_cast<_CLASS *>(this), J);
  }
}

MEMBER(void, getDesignVariablesImplementation(DesignVariable::set_t & designVariables)) const {
  designVariables.insert(const_cast<_CLASS *>(this));
}

MEMBER(GenericScalarExpression<Scalar_>, toExpression()) {
  return GenericScalarExpression<Scalar_>(this);
}

MEMBER(Eigen::MatrixXd, getParameters()) {
  Eigen::Matrix<double, 1, 1> M;
  M << (double)this->toScalar();
  return M;
}

MEMBER(void, getParametersImplementation(Eigen::MatrixXd& value)) const {
  value.resize(1,1);
  value(0, 0) = (double)_p;
}

MEMBER(void, setParametersImplementation(const Eigen::MatrixXd& value)) {
  _p_p = _p;
  _p = Scalar(value(0, 0));
}

MEMBER(void, minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference)) const {
  SM_ASSERT_TRUE(aslam::InvalidArgumentException, (xHat.rows() == 1) & (xHat.cols() == 1), "xHat has incompatible dimensions");
  outDifference = Eigen::VectorXd(1);
  outDifference(0) = (double)(_p - Scalar(xHat(0, 0)));
}

MEMBER(void, minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian)) const {
  minimalDifferenceImplementation(xHat, outDifference);
  outJacobian = Eigen::MatrixXd::Identity(1, 1);
}

#undef MEMBER
#undef _CLASS
}  // namespace backend
}  // namespace aslam

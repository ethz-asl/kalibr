#include <aslam/ScalarExpressionNodeKeypointTime.hpp>

namespace aslam {

ScalarExpressionNodeKeypointTime::ScalarExpressionNodeKeypointTime(
    const aslam::Time & stamp,
    const Eigen::VectorXd & y,
    boost::shared_ptr<
        backend::DesignVariableAdapter<CameraGeometryDesignVariableContainer> > dv)
    : _stamp(stamp),
      _y(y),
      _dv(dv) {
  SM_ASSERT_TRUE(std::runtime_error, _dv.get() != NULL,
                 "Unable to intialize with a null design variable");
}

ScalarExpressionNodeKeypointTime::~ScalarExpressionNodeKeypointTime() {

}

double ScalarExpressionNodeKeypointTime::toScalarImplementation() const {
  return (_stamp + _dv->value().camera()->temporalOffset(_y)).toSec();
}

void ScalarExpressionNodeKeypointTime::evaluateJacobiansImplementation(
    backend::JacobianContainer & outJacobians) const {
  Eigen::MatrixXd Ji;

  _dv->value().temporalOffsetIntrinsicsJacobian(_y, Ji);
  outJacobians.add(_dv.get(), Ji);
}

void ScalarExpressionNodeKeypointTime::evaluateJacobiansImplementation(
    backend::JacobianContainer & outJacobians,
    const Eigen::MatrixXd & applyChainRule) const {
  Eigen::MatrixXd Ji;

  _dv->value().temporalOffsetIntrinsicsJacobian(_y, Ji);
  outJacobians.add(_dv.get(), applyChainRule * Ji);
}

void ScalarExpressionNodeKeypointTime::getDesignVariablesImplementation(
    backend::DesignVariable::set_t & designVariables) const {
  designVariables.insert(_dv.get());
}

}  // namespace aslam

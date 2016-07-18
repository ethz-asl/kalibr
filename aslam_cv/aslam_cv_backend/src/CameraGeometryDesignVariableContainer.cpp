#include <aslam/CameraGeometryDesignVariableContainer.hpp>
#include <aslam/ReprojectionError.hpp>
#include <aslam/ScalarExpressionNodeKeypointTime.hpp>
#include <sm/logging.hpp>

/// \remarks This file is neither in use nor compiled.
namespace aslam {

CameraGeometryDesignVariableContainer::CameraGeometryDesignVariableContainer(
    const boost::shared_ptr<cameras::CameraGeometryBase> & camera,
    bool estimateProjection, bool estimateDistortion, bool estimateShutter)
    : _estimateProjection(estimateProjection),
      _estimateDistortion(estimateDistortion),
      _estimateShutter(estimateShutter),
      _camera(camera) {
  SM_ASSERT_TRUE(std::runtime_error, _camera.get() != NULL,
                 "The camera can not be NULL");
  _cameraDv.reset(
      new backend::DesignVariableAdapter<CameraGeometryDesignVariableContainer>(
          this, false));
  _cameraDv->setActive(true);

  if (_estimateShutter && _camera->minimalDimensionsShutter() == 0) {
    SM_WARN_STREAM(
        "Disabling shutter of camera type \"" << typeid(_camera).name()
            << "\" because it has no variables");
    _estimateShutter = false;
  }

  if (_estimateDistortion && _camera->minimalDimensionsDistortion() == 0) {
    SM_WARN_STREAM(
        "Disabling distortion of camera type \"" << typeid(_camera).name()
            << "\" because it has no variables");
    _estimateDistortion = false;
  }

  if (_estimateProjection && _camera->minimalDimensionsProjection() == 0) {
    SM_WARN_STREAM(
        "Disabling projection of camera type \"" << typeid(_camera).name()
            << "\" because it has no variables");
    _estimateProjection = false;
  }

  if (_camera->minimalDimensions(_estimateProjection, _estimateDistortion,
                                 _estimateShutter) == 0) {
    _cameraDv->setActive(false);
  }

}

CameraGeometryDesignVariableContainer::~CameraGeometryDesignVariableContainer() {

}

/// \brief Get the keypoint time as an expression. If the shutter
///        parameters are being estimated, this will be hooked up
///        to the camera design variable
backend::ScalarExpression CameraGeometryDesignVariableContainer::keypointTime(
    const aslam::Time & imageStamp, const Eigen::VectorXd & y) {
  if (_estimateShutter) {
    boost::shared_ptr < backend::ScalarExpressionNode
        > root(new ScalarExpressionNodeKeypointTime(imageStamp, y, _cameraDv));
    return backend::ScalarExpression(root);
  } else {
    return backend::ScalarExpression(
        (imageStamp + _camera->temporalOffset(y)).toSec());
  }
}

backend::ScalarExpression CameraGeometryDesignVariableContainer::temporalOffset(
    const Eigen::VectorXd & y) {
  return keypointTime(aslam::Time(), y);
}

/// \brief Get a reprojection error based on the measurement, y,
///        inverse measurement uncertainty, inv(R), and an expression
///        representing the point, p_c. For a rolling shutter, the measurement
///        time used to produce p_c should be derived from keypointTime() above
boost::shared_ptr<ReprojectionError> CameraGeometryDesignVariableContainer::createReprojectionError(
    const Eigen::VectorXd & y, const Eigen::MatrixXd & invR,
    backend::HomogeneousExpression p_c) {
  boost::shared_ptr<ReprojectionError> re;
  if (_camera->minimalDimensions(_estimateProjection, _estimateDistortion,
                                 _estimateShutter) == 0) {
    re.reset(new ReprojectionError(y, invR, p_c, _camera.get()));
  } else {
    re.reset(new ReprojectionError(y, invR, p_c, this));
  }
  return re;
}

/// \brief Project the point into the image
Eigen::VectorXd CameraGeometryDesignVariableContainer::homogeneousToKeypoint(
    Eigen::Vector4d ph) {
  Eigen::VectorXd y;
  _camera->vsHomogeneousToKeypoint(ph, y);
  return y;
}

/// \brief Evaluate the jacobians of the homogeneousToKeypoint() with respect to the camera parameters.
void CameraGeometryDesignVariableContainer::evaluateJacobians(
    backend::JacobianContainer & outJacobians, Eigen::Vector4d ph) const {
  Eigen::MatrixXd Ji;
  _camera->homogeneousToKeypointIntrinsicsJacobian(ph, Ji, _estimateProjection,
                                                   _estimateDistortion,
                                                   _estimateShutter);
  outJacobians.add(_cameraDv.get(), Ji);

}

/// \brief Evaluate the jacobians of the homogeneousToKeypoint() with respect to the camera parameters.
void CameraGeometryDesignVariableContainer::evaluateJacobians(
    backend::JacobianContainer & outJacobians,
    const Eigen::MatrixXd & applyChainRule, Eigen::Vector4d ph) const {
  Eigen::MatrixXd Ji;
  _camera->homogeneousToKeypointIntrinsicsJacobian(ph, Ji, _estimateProjection,
                                                   _estimateDistortion,
                                                   _estimateShutter);
  outJacobians.add(_cameraDv.get(), applyChainRule * Ji);

}

void CameraGeometryDesignVariableContainer::getDesignVariables(
    backend::DesignVariable::set_t & designVariables) const {
  designVariables.insert(_cameraDv.get());
}

bool CameraGeometryDesignVariableContainer::isProjectionActive() const {
  return _estimateProjection;
}
bool CameraGeometryDesignVariableContainer::isDistortionActive() const {
  return _estimateDistortion;
}
bool CameraGeometryDesignVariableContainer::isShutterActive() const {
  return _estimateShutter;
}

/// \brief set the whole underlying design variable active
void CameraGeometryDesignVariableContainer::setActive(bool active) {
  _cameraDv->setActive(active);
}

/// \brief is the underlying design variable active?
bool CameraGeometryDesignVariableContainer::isActive() const {
  return _cameraDv->isActive();
}

void CameraGeometryDesignVariableContainer::update(const double * v) {
  _camera->update(v, _estimateProjection, _estimateDistortion,
                  _estimateShutter);
}

int CameraGeometryDesignVariableContainer::minimalDimensions() const {
  return _camera->minimalDimensions(_estimateProjection, _estimateDistortion,
                                    _estimateShutter);
}

void CameraGeometryDesignVariableContainer::getParameters(
    Eigen::MatrixXd & P) const {
  _camera->getParameters(P, _estimateProjection, _estimateDistortion,
                         _estimateShutter);
}

void CameraGeometryDesignVariableContainer::setParameters(
    const Eigen::MatrixXd & P) {
  _camera->setParameters(P, _estimateProjection, _estimateDistortion,
                         _estimateShutter);
}

/// \brief return the temporal offset with respect to the intrinsics.
void CameraGeometryDesignVariableContainer::temporalOffsetIntrinsicsJacobian(
    const Eigen::VectorXd & keypoint, Eigen::MatrixXd & outJi) const {
  _camera->temporalOffsetIntrinsicsJacobian(keypoint, outJi,
                                            _estimateProjection,
                                            _estimateDistortion,
                                            _estimateShutter);
}

}  // namespace aslam

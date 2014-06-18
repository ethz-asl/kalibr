#include <aslam/ReprojectionError.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>

namespace aslam {

ReprojectionError::ReprojectionError(const Eigen::VectorXd & y,
                                     const Eigen::MatrixXd & invR,
                                     backend::HomogeneousExpression p_c,
                                     aslam::cameras::CameraGeometryBase * cam)
    : ErrorTermDs(y.size()),
      _y(y),
      _p_c(p_c),
      _cam(cam),
      _camDvc(NULL),
      _enabled(true),
      _valid(false) {

  SM_ASSERT_TRUE(std::runtime_error, cam != NULL, "The frame must not be null");
  SM_ASSERT_EQ(std::runtime_error, y.size(), _cam->keypointDimension(),
               "The measurement size must match the keypoint dimension");
  SM_ASSERT_EQ(
      std::runtime_error, invR.rows(), _cam->keypointDimension(),
      "The measurement uncertainty size must match the keypoint dimension");
  SM_ASSERT_EQ(
      std::runtime_error, invR.cols(), _cam->keypointDimension(),
      "The measurement uncertainty size must match the keypoint dimension");
  setInvR(invR);
  setError(Eigen::VectorXd::Zero(_cam->keypointDimension()));
  backend::JacobianContainer::set_t dvs;
  p_c.getDesignVariables(dvs);
  setDesignVariablesIterator(dvs.begin(), dvs.end());
}

ReprojectionError::ReprojectionError(
    const Eigen::VectorXd & y, const Eigen::MatrixXd & invR,
    backend::HomogeneousExpression p_c,
    CameraGeometryDesignVariableContainer * camDvc)
    : ErrorTermDs(y.size()),
      _y(y),
      _p_c(p_c),
      _camDvc(camDvc),
      _enabled(true),
      _valid(false) {
  SM_ASSERT_TRUE(std::runtime_error, _camDvc != NULL,
                 "The camera DVC must not be null!");
  _cam = camDvc->camera().get();
  SM_ASSERT_TRUE(std::runtime_error, _cam != NULL,
                 "The camera must not be null");
  SM_ASSERT_EQ(std::runtime_error, y.size(), _cam->keypointDimension(),
               "The measurement size must match the keypoint dimension");
  SM_ASSERT_EQ(
      std::runtime_error, invR.rows(), _cam->keypointDimension(),
      "The measurement uncertainty size must match the keypoint dimension");
  SM_ASSERT_EQ(
      std::runtime_error, invR.cols(), _cam->keypointDimension(),
      "The measurement uncertainty size must match the keypoint dimension");
  setInvR(invR);
  setError(Eigen::VectorXd::Zero(_cam->keypointDimension()));
  backend::JacobianContainer::set_t dvs;
  p_c.getDesignVariables(dvs);
  _camDvc->getDesignVariables(dvs);
  setDesignVariablesIterator(dvs.begin(), dvs.end());
}

ReprojectionError::~ReprojectionError() {

}

/// \brief evaluate the error term and return the weighted squared error e^T invR e
double ReprojectionError::evaluateErrorImplementation() {
  Eigen::VectorXd p = _p_c.toHomogeneous();
  Eigen::VectorXd hat_y;
  bool success = _cam->vsHomogeneousToKeypoint(p, hat_y);
  if (success) {
    _enabled = true;
    setError(hat_y - _y);
  } else {
    // If we ever fail, turn this guy off.
    _enabled = false;
    // leave the error at the previous value.
  }

  return evaluateChiSquaredError();
}

/// \brief is the measurement enabled?
bool ReprojectionError::isEnabled() const {
  return _enabled;
}

bool ReprojectionError::isValid() const {
  return _valid;
}

void ReprojectionError::setValid(bool valid) {
  _valid = valid;
}

/// \brief Get the camera
aslam::cameras::CameraGeometryBase * ReprojectionError::getCamera() const {
  return _cam;
}

/// \brief evaluate the Jacobians
void ReprojectionError::evaluateJacobiansImplementation(
    aslam::backend::JacobianContainer & _jacobians) {

  Eigen::Vector4d p = _p_c.toHomogeneous();
  if (_enabled) {
    Eigen::MatrixXd J;
    Eigen::VectorXd hat_y;
    _cam->vsHomogeneousToKeypoint(p, hat_y, J);
    _p_c.evaluateJacobians(_jacobians, J);

    if (_camDvc) {
      _camDvc->evaluateJacobians(_jacobians, p);
    }
  } else {
    // HACK
    //Eigen::MatrixXd J = Eigen::MatrixXd::Zero(_cam->keypointDimension(), 4);
    //_p_c.evaluateJacobians(_jacobians, J);
    _jacobians.clear();
  }

}

/// \brief get the keypoint measurement
Eigen::VectorXd ReprojectionError::getKeypoint() const {
  return _y;
}

/// \brief get the keypoint measurement
Eigen::VectorXd ReprojectionError::getProjection() {
  Eigen::VectorXd p = _p_c.toHomogeneous();
  Eigen::VectorXd hat_y;
  _cam->vsHomogeneousToKeypoint(p, hat_y);
  return hat_y;
}

/// \brief get the keypoint measurement
bool ReprojectionError::getProjectionSuccess() {
  Eigen::VectorXd p = _p_c.toHomogeneous();
  Eigen::VectorXd hat_y;
  bool success = _cam->vsHomogeneousToKeypoint(p, hat_y);
  return success;
}

/// \brief get the point expressed in the camera frame
aslam::backend::HomogeneousExpression ReprojectionError::getPoint() const {
  return _p_c;
}

bool ReprojectionError::getReprojectedPoint(Eigen::VectorXd& hat_y) {
  Eigen::VectorXd p = _p_c.toHomogeneous();
  return _cam->vsHomogeneousToKeypoint(p, hat_y);
}

}  // namespace aslam


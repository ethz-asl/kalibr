#ifndef ASLAM_RE_REFACTOR_HPP
#define ASLAM_RE_REFACTOR_HPP

#include <aslam/CameraGeometryDesignVariableContainer.hpp>
#include <aslam/backend/ErrorTermDs.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

namespace aslam {

class ReprojectionError : public backend::ErrorTermDs {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReprojectionError(const Eigen::VectorXd & y, const Eigen::MatrixXd & invR,
                    backend::HomogeneousExpression p_c,
                    cameras::CameraGeometryBase * cam);

  ReprojectionError(const Eigen::VectorXd & y, const Eigen::MatrixXd & invR,
                    backend::HomogeneousExpression p_c,
                    CameraGeometryDesignVariableContainer * camDvc);

  virtual ~ReprojectionError();

  /// \brief evaluate the error term and return the weighted squared error e^T invR e
  virtual double evaluateErrorImplementation();

  /// \brief evaluate the Jacobians
  virtual void evaluateJacobiansImplementation(
      aslam::backend::JacobianContainer & J);

  /// \brief Get the camera
  aslam::cameras::CameraGeometryBase * getCamera() const;

  /// \brief get the keypoint measurement
  Eigen::VectorXd getKeypoint() const;

  /// \brief get the projection
  Eigen::VectorXd getProjection();

  /// \brief was the projection successful?
  bool getProjectionSuccess();

  /// \brief get the point expressed in the camera frame
  aslam::backend::HomogeneousExpression getPoint() const;

  bool getReprojectedPoint(Eigen::VectorXd& hat_y);

  /// \brief is the measurement enabled?
  bool isEnabled() const;
  /// \brief is the error term valid?
  bool isValid() const;

  void setValid(bool valid);

 private:
  Eigen::VectorXd _y;
  aslam::backend::HomogeneousExpression _p_c;
  aslam::cameras::CameraGeometryBase * _cam;
  CameraGeometryDesignVariableContainer * _camDvc;
  bool _enabled;
  bool _valid;
};

}  // namespace aslam

#endif /* ASLAM_RE_HPP */

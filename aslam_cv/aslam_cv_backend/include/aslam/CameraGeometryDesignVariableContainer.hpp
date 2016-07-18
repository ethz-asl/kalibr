#ifndef ASLAM_BACKEND_CAMERA_DESIGN_VARIABLE_CONTAINER_HPP
#define ASLAM_BACKEND_CAMERA_DESIGN_VARIABLE_CONTAINER_HPP

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/DesignVariableAdapter.hpp>
#include <aslam/backend/JacobianContainer.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>


/// \remarks This class is neither in use nor compiled.
namespace aslam {

class ReprojectionError;

class CameraGeometryDesignVariableContainer {
 public:
  CameraGeometryDesignVariableContainer(
      const boost::shared_ptr<cameras::CameraGeometryBase> & camera,
      bool estimateProjection, bool estimateDistortion, bool estimateShutter);

  ~CameraGeometryDesignVariableContainer();

  /// \brief Get the keypoint time as an expression. If the shutter
  ///        parameters are being estimated, this will be hooked up
  ///        to the camera design variable
  backend::ScalarExpression keypointTime(const aslam::Time & imageStamp,
                                         const Eigen::VectorXd & y);

  /// \brief Get the temporal offset from the start of the image as an expression. 
  ///        If the shutter parameters are being estimated, this will be hooked up
  ///        to the camera design variable
  backend::ScalarExpression temporalOffset(const Eigen::VectorXd & y);

  /// \brief Get a reprojection error based on the measurement, y,
  ///        inverse measurement uncertainty, inv(R), and an expression
  ///        representing the point, p_c. For a rolling shutter, the measurement
  ///        time used to produce p_c should be derived from keypointTime() above
  boost::shared_ptr<ReprojectionError> createReprojectionError(
      const Eigen::VectorXd & y, const Eigen::MatrixXd & invR,
      backend::HomogeneousExpression p_c);

  /// \brief Project the point into the image
  Eigen::VectorXd homogeneousToKeypoint(Eigen::Vector4d ph);

  /// \brief Evaluate the jacobians of the homogeneousToKeypoint() with respect to the camera parameters.
  void evaluateJacobians(backend::JacobianContainer & outJacobians,
                         Eigen::Vector4d ph) const;

  /// \brief Evaluate the jacobians of the homogeneousToKeypoint() with respect to the camera parameters.
  void evaluateJacobians(backend::JacobianContainer & outJacobians,
                         const Eigen::MatrixXd & applyChainRule,
                         Eigen::Vector4d ph) const;

  /// \brief return the temporal offset with respect to the intrinsics.
  void temporalOffsetIntrinsicsJacobian(const Eigen::VectorXd & keypoint,
                                        Eigen::MatrixXd & outJi) const;

  void getDesignVariables(
      backend::DesignVariable::set_t & designVariables) const;

  bool isProjectionActive() const;
  bool isDistortionActive() const;
  bool isShutterActive() const;

  /// \brief set the whole underlying design variable active
  void setActive(bool active);

  /// \brief is the underlying design variable active?
  bool isActive() const;

  boost::shared_ptr<cameras::CameraGeometryBase> camera() {
    return _camera;
  }

  void update(const double * v);
  int minimalDimensions() const;
  void getParameters(Eigen::MatrixXd & P) const;
  void setParameters(const Eigen::MatrixXd & P);

  boost::shared_ptr<aslam::backend::DesignVariable> getDesignVariable() const {
    return _cameraDv;
  }
 private:
  bool _estimateProjection;
  bool _estimateDistortion;
  bool _estimateShutter;
  boost::shared_ptr<cameras::CameraGeometryBase> _camera;
  boost::shared_ptr<
      backend::DesignVariableAdapter<CameraGeometryDesignVariableContainer> > _cameraDv;

};

}  // aslam

#endif /* ASLAM_BACKEND_CAMERA_VARIABLE_HPP */

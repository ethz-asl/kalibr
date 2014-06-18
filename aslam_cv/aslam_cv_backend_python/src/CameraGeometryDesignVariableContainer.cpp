// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/CameraGeometryDesignVariableContainer.hpp>
#include <sm/python/stl_converters.hpp>
#include <aslam/ReprojectionError.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include "helpers.hpp"

void exportCameraGeometryDvc() {
  using namespace boost::python;
  using namespace aslam;

  class_<CameraGeometryDesignVariableContainer,
      boost::shared_ptr<CameraGeometryDesignVariableContainer>,
      boost::noncopyable>(
      "CameraGeometryDesignVariableContainer",
      init<const boost::shared_ptr<cameras::CameraGeometryBase> &, bool, bool,
          bool>(
          "CameraGeometryDesignVariableContainer( camera, estimateProjection, estimateDistortion, estimateShutter)"))
      .def(
      "keypointTime", &CameraGeometryDesignVariableContainer::keypointTime,
      "ScalarExpression keypointTime( imageTimestamp, keypointMeasurement)").def(
      "temporalOffset", &CameraGeometryDesignVariableContainer::temporalOffset,
      "ScalarExpression temporalOffset( keypointMeasurement)")

  .def("createReprojectionError",
       &CameraGeometryDesignVariableContainer::createReprojectionError,
       "reprojectionError createReprojectionError( y, invR, p_c )")

  .def("setActive", &CameraGeometryDesignVariableContainer::setActive)

  .def("isActive", &CameraGeometryDesignVariableContainer::isActive).def(
      "isProjectionActive",
      &CameraGeometryDesignVariableContainer::isProjectionActive).def(
      "isDistortionActive",
      &CameraGeometryDesignVariableContainer::isDistortionActive).def(
      "isShutterActive",
      &CameraGeometryDesignVariableContainer::isShutterActive).def(
      "camera", &CameraGeometryDesignVariableContainer::camera).def(
      "getDesignVariable",
      &CameraGeometryDesignVariableContainer::getDesignVariable).def(
      "getDesignVariables",
      &getDesignVariablesWrap<CameraGeometryDesignVariableContainer>)

      ;

  class_<ReprojectionError, boost::shared_ptr<ReprojectionError>,
      bases<aslam::backend::ErrorTerm>, boost::noncopyable>(
      "ReprojectionError",
      init<const Eigen::VectorXd &, const Eigen::MatrixXd &,
          backend::HomogeneousExpression, cameras::CameraGeometryBase *>(
          "ReprojectionError( y, invR, p_c, camera )")).def(
      init<const Eigen::VectorXd &, const Eigen::MatrixXd &,
          backend::HomogeneousExpression,
          CameraGeometryDesignVariableContainer *>(
          "ReprojectionError( y, invR, p_c, cameraGeometryDesignVariableContainer )"))
      .def("getCamera", &ReprojectionError::getCamera,
           return_internal_reference<>()).def("getKeypoint",
                                              &ReprojectionError::getKeypoint)
      .def("getPoint", &ReprojectionError::getPoint).def(
      "isEnabled", &ReprojectionError::isEnabled).def(
      "getProjection", &ReprojectionError::getProjection).def(
      "getProjectionSuccess", &ReprojectionError::getProjectionSuccess);

}

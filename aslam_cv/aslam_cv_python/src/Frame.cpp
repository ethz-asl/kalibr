#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/Frame.hpp>
#include <sm/python/Id.hpp>
#include <opencv2/core/eigen.hpp>
#include <aslam/cameras.hpp>

//#include <aslam/BriskDescriptor.hpp>
//#include <aslam/SurfDescriptor.hpp>
#include <aslam/python/ExportFrame.hpp>
using namespace boost::python;
using namespace aslam;

Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> getImage(
    FrameBase * frame) {
  const cv::Mat & from = frame->image();
  Eigen::Matrix < boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic
      > to(from.rows, from.cols);
  cv2eigen(from, to);

  return to;
}

boost::python::tuple computeProjection3(FrameBase * fb,
                                        const Eigen::Vector3d & p) {
  Eigen::VectorXd y;
  bool success = fb->computeProjection3(p, y);
  return boost::python::make_tuple(success, y);
}

boost::python::tuple computeProjection4(FrameBase * fb,
                                        const Eigen::Vector4d & p) {
  Eigen::VectorXd y;
  bool success = fb->computeProjection4(p, y);
  return boost::python::make_tuple(success, y);
}

boost::python::tuple computeProjectionUhp(
    FrameBase * fb, const sm::kinematics::UncertainHomogeneousPoint & p) {
  Eigen::VectorXd y;
  bool success = fb->computeProjectionUhp(p, y);
  return boost::python::make_tuple(success, y);
}

void setImage(
    FrameBase * frame,
    const Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> & from) {
  cv::Mat to;
  eigen2cv(from, to);
  frame->setImage(to);
}

boost::shared_ptr<cameras::CameraGeometryBase> (FrameBase::*geometryBase)() = &FrameBase::geometryBase;

KeypointBase & (FrameBase::*keypointBase)(size_t i) = &FrameBase::keypointBase;

void exportFrame() {
  sm::python::Id_python_converter < FrameId > ::register_converter();
  sm::python::Id_python_converter < LandmarkId > ::register_converter();

  class_<KeypointBase, boost::shared_ptr<KeypointBase>, boost::noncopyable>(
      "KeypointBase", no_init).def("landmark", &KeypointBase::landmark,
                                   return_value_policy<return_by_value>())
  //void setLandmark(const sm::kinematics::UncertainHomogeneousPoint & landmark){ _landmark = landmark; }
      .def("setLandmark", &KeypointBase::setLandmark).def(
      "landmarkId", &KeypointBase::landmarkId,
      return_value_policy<copy_const_reference>())
  //const LandmarkId & landmarkId() const { return _landmarkId; }
      .def("setLandmarkId", &KeypointBase::setLandmarkId)
  //void setLandmarkId(const LandmarkId & landmarkId){ _landmarkId = landmarkId; }
      .def("isLandmarkInitialized", &KeypointBase::isLandmarkInitialized).def(
      "descriptor", &KeypointBase::descriptorPtr).def(
      "setDescriptor", &KeypointBase::setDescriptorPtr).def(
      "clearLandmark", &KeypointBase::clearLandmark).def(
      "setBackProjection", &KeypointBase::setBackProjection).def(
      "setBackProjectionVector", &KeypointBase::setBackProjectionVector).def(
      "backProjection", &KeypointBase::backProjection,
      return_value_policy<return_by_value>()).def(
      "isBackProjectionSet", &KeypointBase::isBackProjectionSet)
        .def("clearBackProjection", &KeypointBase::clearBackProjection)
        .def("isLandmarkEnabled", &KeypointBase::isLandmarkEnabled)
        .def("disableLandmark", &KeypointBase::disableLandmark)
        .def("enableLandmark", &KeypointBase::enableLandmark)
      ;

  bool (FrameBase::*computeReprojectionErrorUhp1)(
      size_t i,
      const sm::kinematics::UncertainHomogeneousPoint & p,
      double & outReprojectionError) const = &FrameBase::computeReprojectionErrorUhp;

  bool (FrameBase::*computeReprojectionErrorUhp2)(
      const KeypointIdentifier &,
      const sm::kinematics::UncertainHomogeneousPoint & p,
      double & outReprojectionError) const = &FrameBase::computeReprojectionErrorUhp;

  bool (FrameBase::*computeReprojectionError31)(
      size_t i,
      const Eigen::Vector3d & p,
      double & outReprojectionError) const = &FrameBase::computeReprojectionError3;

  bool (FrameBase::*computeReprojectionError41)(
      size_t i,
      const Eigen::Vector4d & p,
      double & outReprojectionError) const = &FrameBase::computeReprojectionError4;

  bool (FrameBase::*computeReprojectionError32)(
      const KeypointIdentifier &,
      const Eigen::Vector3d & p,
      double & outReprojectionError) const = &FrameBase::computeReprojectionError3;

  bool (FrameBase::*computeReprojectionError42)(
      const KeypointIdentifier &,
      const Eigen::Vector4d & p,
      double & outReprojectionError) const = &FrameBase::computeReprojectionError4;

  Time (FrameBase::*keypointTime1)(size_t i) const = &FrameBase::keypointTime;
  Time (FrameBase::*keypointTime2)(
      const KeypointIdentifier &) const = &FrameBase::keypointTime;

  /// \brief get the back projection for keypoint i
  void (FrameBase::*getBackProjection1)(
      size_t i,
      BackProjection & outBackProjection) const = &FrameBase::getBackProjection;

  /// \brief get the back projection for keypoint kid
  void (FrameBase::*getBackProjection2)(
      const KeypointIdentifier & kid,
      BackProjection & outBackProjection) const = &FrameBase::getBackProjection;

  /// \brief get the back projection for keypoint i
  void (FrameBase::*getUncertainBackProjection1)(
      size_t i,
      UncertainBackProjection & outBackProjection) const = &FrameBase::getUncertainBackProjection;

  /// \brief get the back projection for keypoint kid
  void (FrameBase::*getUncertainBackProjection2)(
      const KeypointIdentifier & kid,
      UncertainBackProjection & outBackProjection) const = &FrameBase::getUncertainBackProjection;

  class_<FrameBase, boost::shared_ptr<FrameBase>, boost::noncopyable>(
      "FrameBase", no_init).def("id", &FrameBase::id,
                                return_value_policy<copy_const_reference>()).def(
      "setId", &FrameBase::setId).def(
      "time", &FrameBase::time, return_value_policy<copy_const_reference>()).def(
      "setTime", &FrameBase::setTime).def("image", &getImage).def("setImage",
                                                                  &setImage).def(
      "geometryBase", geometryBase).def("keypointBase", keypointBase,
                                        return_internal_reference<>()).def(
      "numKeypoints", &FrameBase::numKeypoints).def(
      "computeReprojectionErrorUhp", computeReprojectionErrorUhp1).def(
      "computeReprojectionErrorUhp", computeReprojectionErrorUhp2).def(
      "computeReprojectionError4", computeReprojectionError41).def(
      "computeReprojectionError4", computeReprojectionError42).def(
      "computeReprojectionError3", computeReprojectionError31).def(
      "computeReprojectionError3", computeReprojectionError32).def(
      "getBackProjection", getBackProjection1).def("getBackProjection",
                                                   getBackProjection2).def(
      "getUncertainBackProjection", getUncertainBackProjection1).def(
      "getUncertainBackProjection", getUncertainBackProjection2).def(
      "keypointTime", keypointTime1).def("keypointTime", keypointTime2).def(
      "computeProjection3", &computeProjection3).def("computeProjection4",
                                                     &computeProjection4).def(
      "computeProjectionUhp", &computeProjectionUhp).def(
      "computeAllBackProjections", &FrameBase::computeAllBackProjections).def(
      "clearKeypoints", &FrameBase::clearKeypoints)
      //.def("",&FrameBase::)
      // virtual Time keypointTime(size_t i) const = 0;
      ;

  using namespace aslam::cameras;
  aslam::python::exportFrame<PinholeCameraGeometry>("PinholeFrame");
  aslam::python::exportFrame<DistortedPinholeCameraGeometry>(
      "DistortedPinholeFrame");
  aslam::python::exportFrame<EquidistantDistortedPinholeCameraGeometry>(
        "EquidistantDistortedPinholeFrame");
  aslam::python::exportFrame<FovDistortedPinholeCameraGeometry>(
          "FovDistortedPinholeFrame");

  aslam::python::exportFrame<PinholeRsCameraGeometry>("PinholeRsFrame");
  aslam::python::exportFrame<DistortedPinholeRsCameraGeometry>(
      "DistortedPinholeRsFrame");
  aslam::python::exportFrame<EquidistantDistortedPinholeRsCameraGeometry>(
      "EquidistantPinholeRsFrame");
  aslam::python::exportFrame<DistortedOmniRsCameraGeometry>("DistortedOmniRsFrame");

  //aslam::python::exportCovarianceReprojectionError<DistortedPinholeRsCameraGeometry>("DistortedPinholeRsFrameCovarianceReprojectionError");

  aslam::python::exportFrame<ExtendedUnifiedCameraGeometry>("ExtendedUnifiedFrame");

  aslam::python::exportFrame<DoubleSphereCameraGeometry>("DoubleSphereFrame");

  aslam::python::exportFrame<OmniCameraGeometry>("OmniFrame");
  aslam::python::exportFrame<DistortedOmniCameraGeometry>("DistortedOmniFrame");
  aslam::python::exportFrame<MaskedDistortedOmniCameraGeometry>(
      "MaskedDistortedOmniFrame");

  aslam::python::exportKeypoint<2>();
  aslam::python::exportKeypoint<3>();
  aslam::python::exportKeypoint<4>();

  class_ < KeypointIdentifier
      > ("KeypointIdentifier", init<>()).def(
          init<MultiFrameId, size_t, size_t>()).add_property(
          "frameId", &KeypointIdentifier::getFrameId,
          &KeypointIdentifier::setFrameId).def_readwrite(
          "keypointIndex", &KeypointIdentifier::keypointIndex).def_readwrite(
          "cameraIndex", &KeypointIdentifier::cameraIndex);
  // export the optimizable intrinsics errors
  //aslam::python::exportReprojectionIntrinsicsError<PinholeRsCameraDesignVariable, SurfDescriptor>("PinholeRsDVSurfFrameReprojectionIntrinsicsError");
  //aslam::python::exportReprojectionIntrinsicsError<PinholeRsCameraDesignVariable, BriskDescriptor>("PinholeRsDVBriskFrameReprojectionIntrinsicsError");
  //aslam::python::exportReprojectionIntrinsicsError<PinholeCameraDesignVariable, SurfDescriptor>("PinholeDVSurfFrameReprojectionIntrinsicsError");
  //aslam::python::exportReprojectionIntrinsicsError<PinholeCameraDesignVariable, BriskDescriptor>("PinholeDVBriskFrameReprojectionIntrinsicsError");

  class_<BackProjection, boost::shared_ptr<BackProjection> >("BackProjection",
                                                             init<>()).def(
      init<const Eigen::Vector3d &>("BackProjection( ray )")).def(
      init<const Eigen::Vector3d &, const Eigen::Vector3d &>(
          "BackProjection( ray, viewOrigin )")).def("getRay",
                                                    &BackProjection::getRay).def(
      "setRay", &BackProjection::setRay).def("getViewOrigin",
                                             &BackProjection::getViewOrigin).def(
      "setViewOrigin", &BackProjection::setViewOrigin);

}

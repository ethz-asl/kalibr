#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/MultiFrame.hpp>
#include <opencv2/core/eigen.hpp>
#include <aslam/FrameBuilder.hpp>
#include <sm/python/Id.hpp>
// These ones are in sm_boost
#include <boost/portable_binary_iarchive.hpp>
#include <boost/portable_binary_oarchive.hpp>
#include <sm/python/boost_serialization_pickle.hpp>

using namespace aslam;
using namespace boost::python;

boost::shared_ptr<NCameraSystem> (MultiFrame::*getCameraSystem1)() const = &MultiFrame::getCameraSystem;

template<typename A>
void saveMf(MultiFrame * mf, const std::string & filename) {
  std::ofstream ofs(filename.c_str(), std::ios::binary);
  SM_ASSERT_TRUE(std::runtime_error, ofs.good(),
                 "Unable to open file " << filename << " for writing");
  A oa(ofs);

  oa << ::boost::serialization::make_nvp("object", *mf);
}

template<typename A>
void loadMf(MultiFrame * mf, const std::string & filename) {
  std::ifstream ifs(filename.c_str(), std::ios::binary);
  SM_ASSERT_TRUE(std::runtime_error, ifs.good(),
                 "Unable to open file " << filename << " for reading");
  A ia(ifs);

  ia >> ::boost::serialization::make_nvp("object", *mf);
}

boost::python::tuple computeProjection3(MultiFrame * fb, size_t cameraIndex,
                                        const Eigen::Vector3d & p) {
  Eigen::VectorXd y;
  bool success = fb->computeProjection3(cameraIndex, p, y);
  return boost::python::make_tuple(success, y);
}

boost::python::tuple computeProjection4(MultiFrame * fb, size_t cameraIndex,
                                        const Eigen::Vector4d & p) {
  Eigen::VectorXd y;
  bool success = fb->computeProjection4(cameraIndex, p, y);
  return boost::python::make_tuple(success, y);
}

boost::python::tuple computeProjectionUhp(
    MultiFrame * fb, size_t cameraIndex,
    const sm::kinematics::UncertainHomogeneousPoint & p) {
  Eigen::VectorXd y;
  bool success = fb->computeProjectionUhp(cameraIndex, p, y);
  return boost::python::make_tuple(success, y);
}

boost::python::tuple getKeypointY(MultiFrame* ptr, size_t i) {
  return boost::python::make_tuple(ptr->keypoint(i).vsY()(0),
                                   ptr->keypoint(i).vsY()(1));
}

void setLandmark1(MultiFrame* mf, const KeypointIdentifier& kid, sm::kinematics::UncertainHomogeneousPoint l)
{
    mf->setLandmark(kid, l);
}

void setLandmark2(MultiFrame* mf, const KeypointIdentifier& kid, sm::kinematics::UncertainHomogeneousPoint l, bool enableLandmark)
{
    mf->setLandmark(kid, l, enableLandmark);
}


void exportMultiFrame() {

  using namespace aslam;
  using namespace boost::python;

  sm::python::Id_python_converter < MultiFrameId > ::register_converter();

  bool (MultiFrame::*computeReprojectionErrorUhp1)(
      size_t i,
      const sm::kinematics::UncertainHomogeneousPoint & p,
      double & outReprojectionError) const = &MultiFrame::computeReprojectionErrorUhp;

  bool (MultiFrame::*computeReprojectionErrorUhp2)(
      const KeypointIdentifier &,
      const sm::kinematics::UncertainHomogeneousPoint & p,
      double & outReprojectionError) const = &MultiFrame::computeReprojectionErrorUhp;

  bool (MultiFrame::*computeReprojectionError31)(
      size_t i,
      const Eigen::Vector3d & p,
      double & outReprojectionError) const = &MultiFrame::computeReprojectionError3;

  bool (MultiFrame::*computeReprojectionError41)(
      size_t i,
      const Eigen::Vector4d & p,
      double & outReprojectionError) const = &MultiFrame::computeReprojectionError4;

  bool (MultiFrame::*computeReprojectionError32)(
      const KeypointIdentifier &,
      const Eigen::Vector3d & p,
      double & outReprojectionError) const = &MultiFrame::computeReprojectionError3;

  bool (MultiFrame::*computeReprojectionError42)(
      const KeypointIdentifier &,
      const Eigen::Vector4d & p,
      double & outReprojectionError) const = &MultiFrame::computeReprojectionError4;

  Time (MultiFrame::*keypointTime1)(size_t i) const = &MultiFrame::keypointTime;
  Time (MultiFrame::*keypointTime2)(
      const KeypointIdentifier &) const = &MultiFrame::keypointTime;

  /// \brief get the back projection for keypoint i
  void (MultiFrame::*getBackProjection1)(
      size_t i,
      BackProjection & outBackProjection) const = &MultiFrame::getBackProjection;

  /// \brief get the back projection for keypoint kid
  void (MultiFrame::*getBackProjection2)(
      const KeypointIdentifier & kid,
      BackProjection & outBackProjection) const = &MultiFrame::getBackProjection;

  /// \brief get the back projection for keypoint i
  void (MultiFrame::*getUncertainBackProjection1)(
      size_t i,
      UncertainBackProjection & outBackProjection) const = &MultiFrame::getUncertainBackProjection;

  /// \brief get the back projection for keypoint kid
  void (MultiFrame::*getUncertainBackProjection2)(
      const KeypointIdentifier & kid,
      UncertainBackProjection & outBackProjection) const = &MultiFrame::getUncertainBackProjection;

  const sm::kinematics::Transformation & (MultiFrame::*T_v_c1)(
      const KeypointIdentifier & kid) const = &MultiFrame::T_v_c;

  /// \brief get the extrinsic calibration for camera j
  const sm::kinematics::Transformation & (MultiFrame::*T_v_c2)(
      size_t j) const = &MultiFrame::T_v_c;

  /// \brief get the extrinsic calibration of the camera that owns this keypoint
  //const sm::kinematics::Transformation & (MultiFrame::*T_c_v1)(const KeypointIdentifier & kid) const = &MultiFrame::T_c_v;

  /// \brief get the extrinsic calibration for camera j
  //const sm::kinematics::Transformation & (MultiFrame::*T_c_v2)(size_t j) const = &MultiFrame::T_c_v;

  boost::shared_ptr<FrameBase> (MultiFrame::*getFrame)(
      size_t f) = &MultiFrame::getFrame;

  KeypointBase & (MultiFrame::*keypoint1)(size_t k) = &MultiFrame::keypoint;
  KeypointBase & (MultiFrame::*keypoint2)(
      const KeypointIdentifier & k) = &MultiFrame::keypoint;


  class_<MultiFrame, boost::shared_ptr<MultiFrame> >("MultiFrame", init<>()).def(
      init<boost::shared_ptr<NCameraSystem> >()).def(
      "id", &MultiFrame::id, return_value_policy<copy_const_reference>()).def(
      "setId", &MultiFrame::setId).def(
      "time", &MultiFrame::time, return_value_policy<copy_const_reference>())
      .def("setTime", &MultiFrame::setTime).def("keypointTime", keypointTime1)
      .def("keypointTime", keypointTime2).def("keypointIdentifier",
                                              &MultiFrame::keypointIdentifier)
  // .def("computeReprojectionError3", &MultiFrame::computeReprojectionError3)
  // .def("computeReprojectionError4", &MultiFrame::computeReprojectionError4)
  // .def("computeReprojectionErrorUhp", &MultiFrame::computeReprojectionErrorUhp) 
  // .def("computeReprojectionError3", &MultiFrame::computeReprojectionError3)
  // .def("computeReprojectionError4", &MultiFrame::computeReprojectionError4)
  // .def("computeReprojectionErrorUhp", &MultiFrame::computeReprojectionErrorUhp)
      .def("computeReprojectionErrorUhp", computeReprojectionErrorUhp1).def(
      "computeReprojectionErrorUhp", computeReprojectionErrorUhp2).def(
      "computeReprojectionError4", computeReprojectionError41).def(
      "computeReprojectionError4", computeReprojectionError42).def(
      "computeReprojectionError3", computeReprojectionError31).def(
      "computeReprojectionError3", computeReprojectionError32)

  // .def("getBackProjection", &MultiFrame::getBackProjection)
  // .def("getBackProjection", &MultiFrame::getBackProjection)
  // .def("getUncertainBackProjection", &MultiFrame::getUncertainBackProjection)
  // .def("getUncertainBackProjection", &MultiFrame::getUncertainBackProjection)
      .def("getBackProjection", getBackProjection1).def("getBackProjection",
                                                        getBackProjection2).def(
      "getUncertainBackProjection", getUncertainBackProjection1).def(
      "getUncertainBackProjection", getUncertainBackProjection2)

  .def("computeProjection3", &computeProjection3).def("computeProjection4",
                                                      &computeProjection4).def(
      "computeProjectionUhp", &computeProjectionUhp)

  .def("numKeypoints", &MultiFrame::numKeypoints).def("getKeypointY",
                                                      &getKeypointY).def(
      "getFrame", getFrame)
  //.def("getFrame", &MultiFrame::getFrame)
      .def("setFrame", &MultiFrame::setFrame).def("numFrames",
                                                  &MultiFrame::numFrames).def(
      "T_v_c", T_v_c1, return_value_policy<copy_const_reference>()).def(
      "T_v_c", T_v_c2, return_value_policy<copy_const_reference>())
  //.def("T_c_v", T_c_v1, return_value_policy<copy_const_reference>())
  //.def("T_c_v", T_c_v2, return_value_policy<copy_const_reference>())
      .def("geometry", &MultiFrame::geometry).def("numCameras",
                                                  &MultiFrame::numCameras).def(
      "getOverlap", &MultiFrame::getOverlap, return_internal_reference<>()).def(
      "getCameraSystem", getCameraSystem1).def(
      "setCameraSystem", &MultiFrame::setCameraSystem).def(
      "saveArchive", &MultiFrame::saveArchive).def(
      "loadArchive", &MultiFrame::loadArchive).def(
      "saveTxt", &saveMf<boost::archive::text_oarchive>).def(
      "loadTxt", &loadMf<boost::archive::text_iarchive>).def(
      "saveXml", &saveMf<boost::archive::xml_oarchive>).def(
      "loadXml", &loadMf<boost::archive::xml_iarchive>).def(
      "saveBa", &saveMf<boost::archive::binary_oarchive>).def(
      "loadBa", &loadMf<boost::archive::binary_iarchive>).def(
      "savePba", &saveMf<boost::archive::portable_binary_oarchive>).def(
      "loadPba", &loadMf<boost::archive::portable_binary_iarchive>)

  //.def("pose", &MultiFrame::pose,return_value_policy<copy_const_reference>())
  //.def("setPose", &MultiFrame::setPose)
  //.def("setUPose", &MultiFrame::setUPose)

      .def("setLandmark", setLandmark1)
      .def("setLandmark", setLandmark2)

    .def(
      "setLandmarkId", &MultiFrame::setLandmarkId).def(
      "keypoint", keypoint1, return_internal_reference<>()).def(
      "keypoint", keypoint2, return_internal_reference<>())

    // clear all landmarks
      .def("clearAllLandmarks", &MultiFrame::clearAllLandmarks)
        ;

}

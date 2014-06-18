#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/CameraSystemBase.hpp>
#include <sm/python/Id.hpp>
#include <aslam/NCameraSystem.hpp>

void exportCameraSystem() {
  using namespace boost::python;
  using namespace aslam;

  sm::python::Id_python_converter < CameraSystemId > ::register_converter();
  class_<CameraSystemBase, boost::shared_ptr<CameraSystemBase>,
      boost::noncopyable>("CameraSystemBase", no_init)
  // /// \brief get the pose of camera i with respect to the vehicle frame
  //     virtual const sm::kinematics::Transformation & T_v_c ( size_t cameraIndex ) const = 0;
      .def("T_v_c", &CameraSystemBase::T_v_c,
           return_value_policy<copy_const_reference>())
  //     /// \brief get the pose of vehicle frame with respect to the camera i
  //     virtual const sm::kinematics::Transformation & T_c_v ( size_t cameraIndex ) const = 0;
  //.def("T_c_v", &CameraSystemBase::T_c_v, return_value_policy<copy_const_reference>())
  //     /// \brief get the geometry object for camera i
  //     virtual boost::shared_ptr<aslam::cameras::CameraGeometryBase> geometry( size_t cameraIndex ) const = 0;
      .def("geometry", &CameraSystemBase::geometry)
  //     /// \brief how many cameras does this system have?
  //     virtual size_t numCameras() const = 0;
      .def("numCameras", &CameraSystemBase::numCameras)

  //     /// \brief build a frame for camera i from this image.
  //     virtual boost::shared_ptr<FrameBase> buildFrame( size_t cameraIndex, const cv::Mat & image ) const = 0;
  //.def("buildFrame", &CameraSystemBase::buildFrame)            

  //     /// \brief Get the overlap mask (or null if there is no overlap)
  //     virtual const aslam::cameras::ImageMask * getOverlap(size_t cameraIndexSeenBy, size_t cameraIndex) const = 0;
      .def("getOverlap", &CameraSystemBase::getOverlap,
           return_internal_reference<>())
  //     virtual bool hasOverlap(size_t cameraIndexSeenBy, size_t cameraIndex) const = 0;
      .def("hasOverlap", &CameraSystemBase::hasOverlap).def(
      "id", &CameraSystemBase::id).def("setId", &CameraSystemBase::setId);

  class_<NCameraSystem, boost::shared_ptr<NCameraSystem>, boost::noncopyable,
      bases<CameraSystemBase> >("NCameraSystem", init<>()).def(
      init<const sm::PropertyTree &>()).def("addCamera",
                                            &NCameraSystem::addCamera)

                                            ;

}

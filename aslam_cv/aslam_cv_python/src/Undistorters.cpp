#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/UndistorterBase.hpp>
#include <aslam/OmniUndistorter.hpp>
#include <aslam/PinholeUndistorter.hpp>

boost::shared_ptr<aslam::FrameBase> buildFrame(
    aslam::UndistorterBase * undistorter,
    const Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> & from) {
  cv::Mat to;
  eigen2cv(from, to);
  return undistorter->buildFrame(to);
}

void exportUndistorters() {
  using namespace aslam;
  using namespace aslam::cameras;
  using namespace boost::python;

  enum_ < interpolation::InterpolationType> ("InterpolationType")
      .value("NearestNeighbor", interpolation::NearestNeighbor)
      .value("Linear", interpolation::Linear)
      .value("Area", interpolation::Area)
      .value("Cubic", interpolation::Cubic)
      .value("Lanczos4", interpolation::Lanczos4);

  class_<UndistorterBase, boost::shared_ptr<UndistorterBase>, boost::noncopyable>(
      "UndistorterBase", no_init).def("buildFrame", &buildFrame).def(
      "idealGeometry", &UndistorterBase::idealGeometryBase).def(
      "distortedGeometry", &UndistorterBase::distortedGeometryBase);

  def("createUndistorter",
      &UndistorterBase::createUndistorter,
      "createUndistorter(sm::PropertyTree undistorterConfig, sm::PropertyTree cameraConfig)");

}

#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/ImageSynchronizer.hpp>

void exportImageSynchronizer() {
  using namespace aslam;
  using namespace boost::python;

  class_<ImageSynchronizer, boost::shared_ptr<ImageSynchronizer>,
      boost::noncopyable>("ImageSynchronizer", init<sm::PropertyTree>())
      .def(init<size_t, double, double, double, bool>("ImageSynchronizer( numCameras, timestampToleranceSeconds, epochDurationSeconds, frameRateSeconds, doTimestampCorrection )"))
      .def(init<size_t, double>("ImageSynchronizer( numCameras, timestampToleranceSeconds )"))
      .def("addImage", &ImageSynchronizer::addEigenImage)
      ;

}

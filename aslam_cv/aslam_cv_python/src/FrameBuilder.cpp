#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/cameras.hpp>
#include <aslam/FrameBuilder.hpp>
#include <aslam/Frame.hpp>

void exportFrameBuilder() {
  using namespace boost::python;
  using namespace aslam;
  using namespace aslam::cameras;

  def("createFrameBuilder", &FrameBuilder::createFrameBuilder);

  class_<FrameBuilder, boost::shared_ptr<FrameBuilder>, boost::noncopyable>(
      "FrameBuilder", no_init).def("updateConfiguration",
                                   &FrameBuilder::updateConfiguration).def(
      "buildFrame", &FrameBuilder::buildFrame);
}

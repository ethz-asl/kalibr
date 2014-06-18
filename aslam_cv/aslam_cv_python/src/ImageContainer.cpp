#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/ImageContainer.hpp>

using namespace boost::python;
using namespace aslam;

void exportImageContainer() {
  class_<ImageContainer, boost::shared_ptr<ImageContainer> >(
      "ImageContainer", init<size_t, Time>())
      .def("numImages", &ImageContainer::numImages)
      .def("isImageReceived", &ImageContainer::isImageReceived)
      .def("get", &ImageContainer::getEigenImage)
      .def("set", &ImageContainer::setEigenImage)
      .def("rawTimestamp", &ImageContainer::rawHardwareTimestamp)
      .def("hardwareTimestamp", &ImageContainer::hardwareTimestamp)
      .def("softwareTimestamp", &ImageContainer::originalSoftwareTimestamp)
      .def("timestamp", &ImageContainer::timestamp)
      .def("areAllImagesReceived", &ImageContainer::areAllImagesReceived);
}

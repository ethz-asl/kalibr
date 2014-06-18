#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <sm/python/Id.hpp>

#include <aslam/Landmark.hpp>

using namespace aslam;

void exportLandmark() {
  using namespace boost::python;

  class_<Landmark, boost::shared_ptr<Landmark> >("Landmark").def(
      "descriptor", &Landmark::descriptorPtr).def("setDescriptor",
                                                  &Landmark::setDescriptor).def(
      "point", &Landmark::point, return_value_policy<return_by_value>()).def(
      "setPoint", &Landmark::setPoint).def(
      "landmarkId", &Landmark::landmarkId,
      return_value_policy<copy_const_reference>()).def("setLandmarkId",
                                                       &Landmark::setLandmarkId)
      .def("frameId", &Landmark::frameId,
           return_value_policy<copy_const_reference>()).def(
      "setFrameId", &Landmark::setFrameId).def("toEuclidean",
                                               &Landmark::toEuclidean).def(
      "toHomogeneous", &Landmark::toHomogeneous);
}

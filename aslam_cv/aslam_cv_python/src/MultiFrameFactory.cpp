#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/MultiFrameFactory.hpp>

using namespace aslam;
using namespace boost::python;

void exportMultiFrameFactory() {

  using namespace aslam;
  using namespace boost::python;

  class_<MultiFrameFactory, boost::shared_ptr<MultiFrameFactory> >("MultiFrameFactory", init<>())
    .def(init<int>())
    .def("setSeed", &MultiFrameFactory::setSeed)
    .def("createRandomMultiFrame", &MultiFrameFactory::createRandomMultiFrame)
    .def("createSimpleBriskMultiFrame", &MultiFrameFactory::createSimpleBriskMultiFrame)
        ;

}

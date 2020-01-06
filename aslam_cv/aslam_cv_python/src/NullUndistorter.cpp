#include <aslam/NullUndistorter.hpp>
#include <aslam/python/exportNullUndistorter.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

using namespace boost::python;
using namespace aslam;

void exportNullUndistorter() {
  aslam::cameras::exportNullUndistorter<cameras::DoubleSphereCameraGeometry>("DSNullUndistorter");
}

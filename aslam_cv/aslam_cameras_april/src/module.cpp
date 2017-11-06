// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetAssymetricAprilgrid.hpp>

#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/python/boost_serialization_pickle.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

BOOST_PYTHON_MODULE(libaslam_cameras_april_python)
{
  using namespace boost::python;
  using namespace aslam::cameras;

  class_<GridCalibrationTargetAprilgrid::AprilgridOptions>("AprilgridOptions", init<>())
    .def_readwrite("doSubpixRefinement", &GridCalibrationTargetAprilgrid::AprilgridOptions::doSubpixRefinement)
    .def_readwrite("showExtractionVideo", &GridCalibrationTargetAprilgrid::AprilgridOptions::showExtractionVideo)
    .def_readwrite("minTagsForValidObs", &GridCalibrationTargetAprilgrid::AprilgridOptions::minTagsForValidObs)
    .def_readwrite("minBorderDistance", &GridCalibrationTargetAprilgrid::AprilgridOptions::minBorderDistance)
    .def_readwrite("maxSubpixDisplacement2", &GridCalibrationTargetAprilgrid::AprilgridOptions::maxSubpixDisplacement2)
    .def_readwrite("blackTagBorder", &GridCalibrationTargetAprilgrid::AprilgridOptions::blackTagBorder)
    .def_pickle(sm::python::pickle_suite<GridCalibrationTargetAprilgrid::AprilgridOptions>());

  class_<GridCalibrationTargetAprilgrid, bases<GridCalibrationTargetBase>,
      boost::shared_ptr<GridCalibrationTargetAprilgrid>, boost::noncopyable>(
      "GridCalibrationTargetAprilgrid",
      init<size_t, size_t, double, double, GridCalibrationTargetAprilgrid::AprilgridOptions>(
          "GridCalibrationTargetAprilgrid(size_t tagRows, size_t tagCols, double tagSize, double tagSpacing, AprilgridOptions options)"))
      .def(init<size_t, size_t, double, double>(
          "GridCalibrationTargetAprilgrid(size_t tagRows, size_t tagCols, double tagSize, double tagSpacing)"))
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def_pickle(sm::python::pickle_suite<GridCalibrationTargetAprilgrid>());

  class_<GridCalibrationTargetAssymetricAprilgrid::TargetPoint>("TargetPoint", init<>())
    .def_readwrite("x", &GridCalibrationTargetAssymetricAprilgrid::TargetPoint::x)
    .def_readwrite("y", &GridCalibrationTargetAssymetricAprilgrid::TargetPoint::y)
    .def_readwrite("size", &GridCalibrationTargetAssymetricAprilgrid::TargetPoint::size)
    .def_readwrite("row", &GridCalibrationTargetAssymetricAprilgrid::TargetPoint::row)
    .def_readwrite("col", &GridCalibrationTargetAssymetricAprilgrid::TargetPoint::col)
    .def_readwrite("id", &GridCalibrationTargetAssymetricAprilgrid::TargetPoint::id)
    .def_pickle(sm::python::pickle_suite<GridCalibrationTargetAssymetricAprilgrid::TargetPoint>());
 

  class_<GridCalibrationTargetAssymetricAprilgrid, bases<GridCalibrationTargetBase>,
      boost::shared_ptr<GridCalibrationTargetAssymetricAprilgrid>, boost::noncopyable>(
      "GridCalibrationTargetAssymetricAprilgrid",
      init<std::vector<GridCalibrationTargetAssymetricAprilgrid::TargetPoint>,GridCalibrationTargetAprilgrid::AprilgridOptions>(
          "GridCalibrationTargetAssymetricAprilgrid(std::vector<TargetPoint> targetPoints,AprilgridOptions options)"))
      .def(init<std::vector<GridCalibrationTargetAssymetricAprilgrid::TargetPoint>>(
          "GridCalibrationTargetAssymetricAprilgrid(std::vector<TargetPoint> targetPoints)"))
      .def(init<boost::python::list,GridCalibrationTargetAprilgrid::AprilgridOptions>(
          "GridCalibrationTargetAssymetricAprilgrid(boost::python::list vector_list, GridCalibrationTargetAprilgrid::AprilgridOptions options)"))
      .def(init<boost::python::list>(
          "GridCalibrationTargetAssymetricAprilgrid(boost::python::list vector_list)"))
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def_pickle(sm::python::pickle_suite<GridCalibrationTargetAssymetricAprilgrid>());
}

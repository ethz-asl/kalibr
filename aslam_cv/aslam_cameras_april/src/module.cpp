// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/python/boost_serialization_pickle.hpp>

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
}

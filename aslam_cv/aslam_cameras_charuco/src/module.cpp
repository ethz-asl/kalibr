// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <aslam/cameras/GridCalibrationTargetCharuco.hpp>
#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/python/boost_serialization_pickle.hpp>

BOOST_PYTHON_MODULE(libaslam_cameras_charuco_python)
{
  using namespace boost::python;
  using namespace aslam::cameras;

  class_<GridCalibrationTargetCharuco::CharucoOptions>("CharucoOptions", init<>())
    .def_readwrite("doSubpixRefinement", &GridCalibrationTargetCharuco::CharucoOptions::doSubpixRefinement)
    .def_readwrite("showExtractionVideo", &GridCalibrationTargetCharuco::CharucoOptions::showExtractionVideo)
    .def_readwrite("minTagsForValidObs", &GridCalibrationTargetCharuco::CharucoOptions::minTagsForValidObs)
    .def_readwrite("minBorderDistance", &GridCalibrationTargetCharuco::CharucoOptions::minBorderDistance)
    .def_readwrite("maxSubpixDisplacement2", &GridCalibrationTargetCharuco::CharucoOptions::maxSubpixDisplacement2)
    .def_readwrite("blackTagBorder", &GridCalibrationTargetCharuco::CharucoOptions::blackTagBorder)
    .def_pickle(sm::python::pickle_suite<GridCalibrationTargetCharuco::CharucoOptions>());

  class_<GridCalibrationTargetCharuco, bases<GridCalibrationTargetBase>,
      boost::shared_ptr<GridCalibrationTargetCharuco>, boost::noncopyable>(
      "GridCalibrationTargetCharuco",
      init<size_t, size_t, double, double, const std::string&, size_t, size_t, GridCalibrationTargetCharuco::CharucoOptions>(
          "GridCalibrationTargetCharuco(size_t tagRows, size_t tagCols, double tagSize, double tagSpacing, const std::string& dictName, size_t markerSize, size_t nMarkers, CharucoOptions options)"))
      .def(init<size_t, size_t, double, double, const std::string&, size_t, size_t>(
          "GridCalibrationTargetCharuco(size_t tagRows, size_t tagCols, double tagSize, double tagSpacing, const std::string& dictName, size_t markerSize, size_t nMarkers)"))
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def_pickle(sm::python::pickle_suite<GridCalibrationTargetCharuco>());
}

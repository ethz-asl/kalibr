// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>

void exportCameraGeometries();
void exportTimeAndDuration();
void exportFrontend();
//void exportCameraSystemClasses();
//void exportMatchingAlgorithms();
//void exportImageContainer();
void exportGridCalibration();
//void exportLandmark();
void exportUndistorters();
//void exportNCameras();
void exportPinholeUndistorter();
void exportOmniUndistorter();


// The title of this library must match exactly
BOOST_PYTHON_MODULE(libaslam_cv_python)
{
  // fill this in with boost::python export code
  exportCameraGeometries();
  exportTimeAndDuration();
  exportFrontend();
//  exportCameraSystemClasses();
//  exportMatchingAlgorithms();
//  exportImageContainer();
  exportGridCalibration();
//  exportLandmark();
  exportUndistorters();
//  exportNCameras();
  exportPinholeUndistorter();
  exportOmniUndistorter();
}

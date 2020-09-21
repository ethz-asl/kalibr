#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <aslam/cameras/GridCalibrationTargetGeneral.hpp>

namespace aslam {
namespace cameras {

/// \brief Construct a calibration target
GridCalibrationTargetGeneral::GridCalibrationTargetGeneral(size_t rows, size_t cols)
    : GridCalibrationTargetBase(rows, cols) {

  // allocate memory for the grid points
  _points.resize(size(), 3);

  // for (unsigned int r = 0; r < rows; r++)
  //   for (unsigned int c = 0; c < cols; c++)
  //     _points.row(gridCoordinatesToPoint(r, c)) = Eigen::Matrix<double, 1, 3>(gridpoints[gridCoordinatesToPoint(r, c)][0], 
  //                                                                             gridpoints[gridCoordinatesToPoint(r, c)][1], 0.0);
  // _points = gridpoints;
  //initialize a normal grid
  // createGridPoints();

  //start the output window if requested
  initialize();
}

/// \brief initialize the object
void GridCalibrationTargetGeneral::initialize()
{
  // if (_options.showExtractionVideo) {
  //   cv::namedWindow("Circlegrid corners", CV_WINDOW_AUTOSIZE);
  //   cvStartWindowThread();
  // }
}

void GridCalibrationTargetGeneral::setPoints(const Eigen::MatrixXd &gridpoints){
    for (unsigned int r = 0; r < _rows; r++)
      for (unsigned int c = 0; c < _cols; c++)
        _points.row(gridCoordinatesToPoint(r, c)) = Eigen::Matrix<double, 1, 3>(gridpoints(gridCoordinatesToPoint(r, c),0), 
                                                                                gridpoints(gridCoordinatesToPoint(r, c),1), 0.0);
}


/// \brief initialize a checkerboard grid (cols*rows = (cols)*(rows) internal grid points)
void GridCalibrationTargetGeneral::createGridPoints() {
  // for (unsigned int r = 0; r < _rows; r++)
  //   for (unsigned int c = 0; c < _cols; c++)
  //     _points.row(gridCoordinatesToPoint(r, c)) = Eigen::Matrix<double, 1, 3>(
  //         _spacing * r, _spacing * c, 0.0)

  // TODO - Read the gridpoints extracted from the a file here(Gridpoints are essentially worldpoints)
}

/// \brief extract the calibration target points from an image and write to an observation
bool GridCalibrationTargetGeneral::computeObservation(const cv::Mat & image,
           Eigen::MatrixXd & outImagePoints, std::vector<bool> &outCornerObserved) const {


  // extract the circle grid corners
  // cv::Size patternSize(cols(), rows());
  // cv::Mat centers(size(), 2, CV_64FC1);

  // bool success = false;
  // if(_options.useAsymmetricCirclegrid)
  //   success = cv::findCirclesGrid( image, patternSize, centers, cv::CALIB_CB_ASYMMETRIC_GRID );
  // else
  //   success = cv::findCirclesGrid( image, patternSize, centers );

  // Read the corners from a file and if the corner was valid or here

  //draw corners
  // if (_options.showExtractionVideo) {
  //   //image with refined (blue) and raw corners (red)
  //   cv::Mat imageCopy1 = image.clone();
  //   cv::cvtColor(imageCopy1, imageCopy1, CV_GRAY2RGB);
  //   cv::drawChessboardCorners(imageCopy1, cv::Size(rows(), cols()), centers, true);

  //   // write error msg
  //   if (!success)
  //     cv::putText(imageCopy1, "Detection failed! (frame not used)",
  //                 cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
  //                 CV_RGB(255,0,0), 3, 8, false);

  //   cv::imshow("Circlegrid corners", imageCopy1);  // OpenCV call
  //   cv::waitKey(1);
  // }

  //exit here if there is an error
  // if (!success)
  //   return success;

  // //set all points as observed (circlegrid is only usable in that case)
  // outCornerObserved = allGood;

  // //convert to eigen for output
  // outImagePoints.resize(size(), 2);
  // for (unsigned int i = 0; i < size(); i++)
  //   outImagePoints.row(i) = Eigen::Matrix<double, 1, 2>(
  //       centers.row(i).at<float>(0), centers.row(i).at<float>(1));

  // return success;
  return true;
}

}  // namespace cameras
}  // namespace aslam

//export explicit instantions for all included archives
#include <sm/boost/serialization.hpp>
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetGeneral);

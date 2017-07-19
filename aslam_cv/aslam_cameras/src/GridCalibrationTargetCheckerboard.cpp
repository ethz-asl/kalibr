#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <sm/eigen/serialization.hpp>

namespace aslam {
namespace cameras {

/// \brief Construct a calibration target
///        rows:   number of internal corners (8x8 chessboard would be 7x7 internal corners)
///        cols:   number of internal corners
///        rowSpacingMeters: spacing in y-direction [m]
///        colSpacingMeters: spacing in x-direction [m]
///
///   point ordering in _points: (e.g. 2x2 grid)
///          *-------*-------*-------*
///          | BLACK | WHITE | BLACK |
///          *------(3)-----(4)------*
///          | WHITE | BLACK | WHITE |
///          *------(1)-----(2)------*
///    y     | BLACK | WHITE | BLACK |
///   ^      *-------*-------*-------*
///   |-->x
GridCalibrationTargetCheckerboard::GridCalibrationTargetCheckerboard(
    size_t rows, size_t cols, double rowSpacingMeters, double colSpacingMeters,
    const CheckerboardOptions &options)
    : GridCalibrationTargetBase(rows, cols),
      _rowSpacingMeters(rowSpacingMeters),
      _colSpacingMeters(colSpacingMeters),
      _options(options) {
  SM_ASSERT_GT(Exception, rowSpacingMeters, 0.0,
               "rowSpacingMeters has to be positive");
  SM_ASSERT_GT(Exception, colSpacingMeters, 0.0,
               "colSpacingMeters has to be positive");

  // allocate memory for the grid points
  _points.resize(size(), 3);

  //initialize a normal grid (checkerboard and circlegrids)
  createGridPoints();

  //start the output window if requested
  initialize();
}

/// \brief initialize the object
void GridCalibrationTargetCheckerboard::initialize()
{
  if (_options.showExtractionVideo) {
    cv::namedWindow("Checkerboard corners", CV_WINDOW_AUTOSIZE);
    cvStartWindowThread();
  }
}

/// \brief initialize a checkerboard grid (cols*rows = (cols)*(rows) internal grid points)
///   point ordering: (e.g. 2x2 grid)
///          *-------*-------*-------*
///          | BLACK | WHITE | BLACK |
///          *------(3)-----(4)------*
///          | WHITE | BLACK | WHITE |
///          *------(1)-----(2)------*
///    y     | BLACK | WHITE | BLACK |
///   ^      *-------*-------*-------*
///   |-->x
///
void GridCalibrationTargetCheckerboard::createGridPoints() {
  for (unsigned int r = 0; r < _rows; r++)
    for (unsigned int c = 0; c < _cols; c++)
      _points.row(gridCoordinatesToPoint(r, c)) = Eigen::Matrix<double, 1, 3>(
          _rowSpacingMeters * r, _colSpacingMeters * c, 0.0);
}

/// \brief extract the calibration target points from an image and write to an observation
bool GridCalibrationTargetCheckerboard::computeObservation(const cv::Mat & image,
           Eigen::MatrixXd & outImagePoints, std::vector<bool> &outCornerObserved) const {

  // set the open cv flags
  int flags = 0;
  if (_options.performFastCheck)
    flags += cv::CALIB_CB_FAST_CHECK;
  if (_options.useAdaptiveThreshold)
    flags += cv::CALIB_CB_ADAPTIVE_THRESH;
  if ( _options.normalizeImage)
    flags += cv::CALIB_CB_NORMALIZE_IMAGE;
  if (_options.filterQuads)
    flags += cv::CALIB_CB_FILTER_QUADS;

  // extract the checkerboard corners
  cv::Size patternSize(cols(), rows());
  cv::Mat centers(size(), 2, CV_64FC1);
  bool success = cv::findChessboardCorners(image, patternSize, centers, flags);

  // do optional subpixel refinement
  if (_options.doSubpixelRefinement && success) {
    cv::cornerSubPix(
        image, centers, cv::Size(_options.windowWidth, _options.windowWidth), cv::Size(-1, -1),
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  }

  //draw corners
  if (_options.showExtractionVideo) {
    //image with refined (blue) and raw corners (red)
    cv::Mat imageCopy1 = image.clone();
    cv::cvtColor(imageCopy1, imageCopy1, CV_GRAY2RGB);
    cv::drawChessboardCorners(imageCopy1, cv::Size(rows(), cols()), centers,
                              true);

    // write error msg
    if (!success)
      cv::putText(imageCopy1, "Detection failed! (frame not used)",
                  cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
                  CV_RGB(255,0,0), 3, 8, false);

    cv::imshow("Checkerboard corners", imageCopy1);  // OpenCV call
    cv::waitKey(1);
  }

  //exit here if there is an error
  if (!success)
    return success;

  //set all points as observed (checkerboard is only usable in that case)
  std::vector<bool> allGood(size(), true);
  outCornerObserved = allGood;

  //convert to eigen for output
  outImagePoints.resize(size(), 2);
  for (unsigned int i = 0; i < size(); i++)
    outImagePoints.row(i) = Eigen::Matrix<double, 1, 2>(
        centers.row(i).at<float>(0), centers.row(i).at<float>(1));

  return success;
}
}  // namespace cameras
}  // namespace aslam

//export explicit instantions for all included archives
#include <sm/boost/serialization.hpp>
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetCheckerboard);
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetCheckerboard::CheckerboardOptions);

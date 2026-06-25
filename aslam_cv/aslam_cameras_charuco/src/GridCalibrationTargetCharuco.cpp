#include <vector>
#include <algorithm>
#include <numeric>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <boost/make_shared.hpp>
#include <sm/assert_macros.hpp>
#include <sm/logging.hpp>
#include <aslam/cameras/GridCalibrationTargetCharuco.hpp>

namespace aslam {
namespace cameras {

/// \brief Construct a Charuco calibration target
///        tagRows:    number of chessboard squares in y-direction (= OpenCV squaresY)
///        tagCols:    number of chessboard squares in x-direction (= OpenCV squaresX)
///        tagSize:    ArUco marker side length [m] (= OpenCV markerLength)
///        tagSpacing: ratio such that chessboard square length = tagSize * (1 + tagSpacing)
///                    i.e. tagSpacing = squareLength/markerLength - 1
///        dictName:   Name of the Charuco dictionary (one of PREDEFINED_DICTIONARY_NAME without DICT_ prefix).
///        markerSize: size of the marker in bits (e.g. 4 for 4x4_50 markers) (only used if dictName is empty).
///        nMarkers:   number of markers in the dictionary (e.g. 50 for 4x4_50 markers) (only used if dictName is empty).
GridCalibrationTargetCharuco::GridCalibrationTargetCharuco(
    size_t tagRows, size_t tagCols, double tagSize, double tagSpacing, const std::string& dictName,
    size_t markerSize, size_t nMarkers, const CharucoOptions &options)
    : GridCalibrationTargetBase(tagRows - 1, tagCols - 1),
      _tagSize(tagSize),
      _tagSpacing(tagSpacing),
      _dictName(dictName),
      _markerSize(markerSize),
      _nMarkers(nMarkers),
      _options(options) {
  SM_ASSERT_GT(Exception, tagSize, 0.0, "tagSize has to be positive");
  SM_ASSERT_GT(Exception, tagSpacing, 0.0, "tagSpacing has to be positive");

  // allocate memory for the grid points
  _points.resize(size(), 3);

  //start the output window if requested
  initialize();
}

//protected ctor for serialization
GridCalibrationTargetCharuco::GridCalibrationTargetCharuco()
{}

cv::Ptr<cv::aruco::Dictionary> createDictionary(const std::string& dictName, size_t markerSize, size_t nMarkers)
{
  if (dictName == "4X4_50")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
  if (dictName == "4X4_100")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);
  if (dictName == "4X4_250")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_250);
  if (dictName == "4X4_1000")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_1000);
  if (dictName == "5X5_50")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_50);
  if (dictName == "5X5_100")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_100);
  if (dictName == "5X5_250")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_250);
  if (dictName == "5X5_1000")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_1000);
  if (dictName == "6X6_50")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);
  if (dictName == "6X6_100")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_100);
  if (dictName == "6X6_250")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);
  if (dictName == "6X6_1000")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_1000);
  if (dictName == "7X7_50")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_50);
  if (dictName == "7X7_100")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_100);
  if (dictName == "7X7_250")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_250);
  if (dictName == "7X7_1000")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_1000);
  if (dictName == "ARUCO_ORIGINAL")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);
  if (dictName == "APRILTAG_16h5")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_16h5);
  if (dictName == "APRILTAG_25h9")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_25h9);
  if (dictName == "APRILTAG_36h10")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h10);
  if (dictName == "APRILTAG_36h11")
    return cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11);

  return cv::aruco::generateCustomDictionary(nMarkers, markerSize);
}

/// \brief initialize the object
void GridCalibrationTargetCharuco::initialize()
{
  //initialize a normal grid (checkerboard and circlegrids)
  createGridPoints();

  if (_options.showExtractionVideo) {
    cv::namedWindow("Charuco: Marker detections", cv::WINDOW_NORMAL);
    cv::resizeWindow("Charuco: Marker detections", 640, 480);
    cv::namedWindow("Charuco: Marker corners", cv::WINDOW_NORMAL);
    cv::resizeWindow("Charuco: Marker corners", 640, 480);
    cv::namedWindow("Charuco: Corner detections", cv::WINDOW_NORMAL);
    cv::resizeWindow("Charuco: Corner detections", 640, 480);
  }

  //create the tag detector
  _dict = createDictionary(_dictName, _markerSize, _nMarkers);
  const auto squareLength = _tagSize * (1.0 + _tagSpacing);
  const auto markerLength = _tagSize;
  _board = cv::aruco::CharucoBoard::create(_cols + 1, _rows + 1, squareLength, markerLength, _dict);
  _params = cv::aruco::DetectorParameters::create();
  if (_options.doSubpixRefinement)
    _params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
  _params->minDistanceToBorder = _options.minBorderDistance;
  _params->markerBorderBits = _options.blackTagBorder;
}

/// \brief initialize a charuco grid
void GridCalibrationTargetCharuco::createGridPoints() {
  for (unsigned r = 0; r < _rows; r++) {
    for (unsigned c = 0; c < _cols; c++) {
      Eigen::Matrix<double, 1, 3> point;

      point(0) = (c + 1) * (1 + _tagSpacing) * _tagSize;
      point(1) = (r + 1) * (1 + _tagSpacing) * _tagSize;
      point(2) = 0.0;

      _points.row(static_cast<Eigen::Index>(gridCoordinatesToPoint(r, c))) = point;
    }
  }
}

/// \brief extract the calibration target points from an image and write to an observation
bool GridCalibrationTargetCharuco::computeObservation(
    const cv::Mat & image, Eigen::MatrixXd & outImagePoints,
    std::vector<bool> &outCornerObserved) const {

  bool success = true;

  // detect the tags
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f> > markerCorners;
  cv::aruco::detectMarkers(image, _board->dictionary, markerCorners, markerIds, _params);

  //did we find enough tags?
  if (markerCorners.size() < _options.minTagsForValidObs) {
    success = false;

    //immediate exit if we dont need to show video for debugging...
    //if video is shown, exit after drawing video...
    if (!_options.showExtractionVideo)
      return success;
  }

  // Create a list of indices
  std::vector<size_t> idx(markerIds.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(idx.begin(), idx.end(),
    [&markerIds](size_t i1, size_t i2) {return markerIds[i1] < markerIds[i2];});

  // check for duplicate tagIds (--> if found: wild Aruco in image not belonging to calibration target)
  // (only if we have more than 1 tag...)
  if (markerIds.size() > 1) {
    for (unsigned i = 0; i < markerIds.size() - 1; i++)
      if (markerIds[idx[i]] == markerIds[idx[i + 1]]) {
        //show the duplicate tags in the image
        cv::destroyAllWindows();
        cv::namedWindow("Wild Aruco detected. Hide them!");
        cv::startWindowThread();

        cv::Mat imageCopy = image.clone();
        cv::cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2RGB);

        //mark all duplicate tags in image
        for (size_t j = 0; j < markerIds.size() - 1; j++) {
          if (markerIds[idx[j]] == markerIds[idx[j + 1]]) {
            std::vector<std::vector<cv::Point2f>> corners = {
              markerCorners[idx[j]],
              markerCorners[idx[j + 1]],
            };
            std::vector<int> ids = {
              markerIds[idx[j]],
              markerIds[idx[j + 1]],
            };
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
          }
        }

        cv::putText(imageCopy, "Duplicate Aruco detected. Hide them.",
                    cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    CV_RGB(255,0,0), 2, 8, false);
        cv::putText(imageCopy, "Press enter to exit...", cv::Point(50, 80),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255,0,0), 2, 8, false);
        cv::imshow("Duplicate Aruco detected. Hide them", imageCopy);  // OpenCV call

        // and exit
        SM_FATAL_STREAM("\n[ERROR]: Found aruco not belonging to calibration board. Check the image for the tag and hide it.\n");

        cv::waitKey();
        exit(0);
      }
  }

  //insert the observed points into the correct location of the grid point array
  std::vector<cv::Point2f> charucoCorners;
  std::vector<int> charucoIds;
  if (markerIds.size() > 0) {
    cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, _board,
      charucoCorners, charucoIds);
  }

  if (_options.showExtractionVideo) {
    //image with refined (blue) and raw corners (red)
    cv::Mat imageCopy1 = image.clone();
    cv::cvtColor(imageCopy1, imageCopy1, cv::COLOR_GRAY2RGB);
    for (unsigned i = 0; i < markerCorners.size(); i++)
      for (unsigned j = 0; j < 4; j++) {
        cv::circle(
            imageCopy1,
            cv::Point2f(markerCorners[i][j].x, markerCorners[i][j].y),
              3, CV_RGB(0,0,255), 1);

        if (!success)
          cv::putText(imageCopy1, "Detection failed! (frame not used)",
                      cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                      CV_RGB(255,0,0), 3, 8, false);
      }

    cv::imshow("Charuco: Marker corners", imageCopy1);  // OpenCV call
    cv::waitKey(1);

    /* copy image for modification */
    cv::Mat imageCopy2 = image.clone();
    cv::cvtColor(imageCopy2, imageCopy2, cv::COLOR_GRAY2RGB);
    /* highlight detected tags in image */
    cv::aruco::drawDetectedMarkers(imageCopy2, markerCorners, markerIds);

    if (!success)
      cv::putText(imageCopy2, "Detection failed! (frame not used)",
                  cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  CV_RGB(255,0,0), 3, 8, false);

    cv::imshow("Charuco: Marker detections", imageCopy2);  // OpenCV call
    cv::waitKey(1);

    /* copy image for modification */
    cv::Mat imageCopy3 = image.clone();
    cv::cvtColor(imageCopy3, imageCopy3, cv::COLOR_GRAY2RGB);
    /* highlight detected tags in image */
    cv::aruco::drawDetectedCornersCharuco(imageCopy3, charucoCorners, charucoIds);

    if (!success)
      cv::putText(imageCopy3, "Detection failed! (frame not used)",
                  cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  CV_RGB(255,0,0), 3, 8, false);

    cv::imshow("Charuco: Corner detections", imageCopy3);  // OpenCV call
    cv::waitKey(1);

    //if success is false exit here (delayed exit if _options.showExtractionVideo=true for debugging)
    if (!success)
      return success;
  }

  outCornerObserved.resize(size(), false);
  outImagePoints.resize(size(), 2);

  for (size_t i = 0; i < charucoIds.size(); ++i)
  {
    const auto cornerId = static_cast<size_t>(charucoIds[i]);
    if (cornerId >= _board->chessboardCorners.size())
      continue;
    outImagePoints(static_cast<Eigen::Index>(cornerId), 0) = charucoCorners[i].x;
    outImagePoints(static_cast<Eigen::Index>(cornerId), 1) = charucoCorners[i].y;
    outCornerObserved[cornerId] = true;
  }

  //succesful observation
  return success;
}

}  // namespace cameras
}  // namespace aslam

//export explicit instantions for all included archives
#include <sm/boost/serialization.hpp>
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetCharuco);
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetCharuco::CharucoOptions);

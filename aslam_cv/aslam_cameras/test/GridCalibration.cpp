// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <sm/boost/serialization.hpp>
#include <aslam/targets.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sm/kinematics/Transformation.hpp>

TEST(GridCalibrationTestSuite, testSerialization)
{
  using namespace aslam::cameras;
  sm::logging::setLevel(sm::logging::levels::Debug);
  try {
    // create a target:
    GridCalibrationTargetCheckerboard::CheckerboardOptions checkeroptions;
    GridCalibrationTargetCheckerboard::Ptr checker( new GridCalibrationTargetCheckerboard(8, 9, 1, 1, checkeroptions) );
    sm::boost_serialization::save_xml(checker, "checkertarget", "test1.bin");
    sm::boost_serialization::save_xml(checkeroptions, "checkeroptions", "test2.bin");

    GridCalibrationTargetCheckerboard::Ptr checker2;
    GridCalibrationTargetCheckerboard::CheckerboardOptions checkeroptions2;
    sm::boost_serialization::load_xml(checker2, "checkertarget", "test1.bin");
    sm::boost_serialization::load_xml(checkeroptions2, "checkeroptions", "test2.bin");

    // create a target:
    GridCalibrationTargetCirclegrid::CirclegridOptions circleoptions;
    GridCalibrationTargetCirclegrid::Ptr circle( new GridCalibrationTargetCirclegrid(8, 9, 1, circleoptions) );
    sm::boost_serialization::save_xml(circle, "circlegrid", "test1.bin");
    sm::boost_serialization::save_xml(circleoptions, "circleoptions", "test2.bin");

    GridCalibrationTargetCirclegrid::Ptr circle2;
    GridCalibrationTargetCirclegrid::CirclegridOptions circleoptions2;
    sm::boost_serialization::load_xml(circle2, "circlegrid", "test1.bin");
    sm::boost_serialization::load_xml(circleoptions2, "circleoptions", "test2.bin");

    // create a camera:
    boost::shared_ptr<DistortedPinholeCameraGeometry> geometry( new DistortedPinholeCameraGeometry() );
    sm::boost_serialization::save_xml(geometry, "cam", "cam.bin");

    // test GridDetector
    GridDetector::GridDetectorOptions detector_options;
    GridDetector::GridDetectorOptions detector_options2;
    sm::boost_serialization::save_xml(detector_options, "detops", "test2.bin");
    sm::boost_serialization::load_xml(detector_options2, "detops", "test2.bin");
    GridDetector detector(geometry, checker, detector_options);

    // test Observation
    cv::Mat image;
    image = cv::imread("testImageCheckerboard.jpg", 0);
    if (image.data == NULL) {
      std::cout << "Checkerboard test image not found" << std::endl;
    }
    ASSERT_TRUE(image.data != NULL);

    // initialise the camera geometry:
    ASSERT_TRUE( detector.initCameraGeometryFromObservation(image) );

    //extract the target
    GridCalibrationTargetObservation obs;
    GridCalibrationTargetObservation obs2;
    ASSERT_TRUE( detector.findTarget(image, obs) );
    sm::boost_serialization::save_xml(obs, "obs", "test.bin");
    sm::boost_serialization::load_xml(obs2, "obs", "test.bin");
    ASSERT_TRUE( detector.findTarget(obs.image(), obs2) );

    SCOPED_TRACE("");
  }
  catch(const std::exception &e) {
    FAIL() << e.what();
  }
}

TEST(GridCalibrationTestSuite, testCalibrationTargetCheckerboard)
{
  using namespace aslam::cameras;
  sm::logging::setLevel(sm::logging::levels::Debug);
  try {
    int rows = 8;
    int cols = 9;
    double rowSpacingMeters = 0.05;
    double colSpacingMeter = 0.05;

    // create a target:
    GridCalibrationTargetCheckerboard::CheckerboardOptions target_options;
    target_options.doSubpixelRefinement = true;
    GridCalibrationTargetCheckerboard::Ptr target( new GridCalibrationTargetCheckerboard(rows, cols, rowSpacingMeters, colSpacingMeter, target_options) );

    // create a camera:
    boost::shared_ptr<DistortedPinholeRsCameraGeometry> geometry( new DistortedPinholeRsCameraGeometry() );

    // create a detector:
    GridDetector::GridDetectorOptions detector_options;
    GridDetector detector(geometry, target, detector_options);

    // load the test image and extract the target:
    cv::Mat image;
    image = cv::imread("testImageCheckerboard.jpg", 0);// force grayscale
    if (image.data == NULL) {
      std::cout << "Checkerboard test image not found" << std::endl;
    }
    ASSERT_TRUE(image.data != NULL);

    // initialise the camera geometry:
    ASSERT_TRUE( detector.initCameraGeometryFromObservation(image) );

    //extract the target
    GridCalibrationTargetObservation obs;
    ASSERT_TRUE( detector.findTarget(image, obs) );

    // try to get the intrinsics
    Eigen::Matrix3d cam = geometry->projection().getCameraMatrix();
    SM_DEBUG_STREAM("Camera matrix:\n" << cam << std::endl);

    // test the transformation:
    sm::kinematics::Transformation transformation;
    ASSERT_TRUE(geometry->projection().estimateTransformation(obs, transformation));

    SM_DEBUG_STREAM("Transformation" << std::endl << transformation.T() << std::endl);

    SCOPED_TRACE("");
  }
  catch(const std::exception &e) {
    FAIL() << e.what();
  }
}

TEST(GridCalibrationTestSuite, testCalibrationTargetCircleGrid)
{
  using namespace aslam::cameras;
  sm::logging::setLevel(sm::logging::levels::Debug);
  try {
    int rows = 5;
    int cols = 7;
    double spacingMeters = 0.01;

    // create a target:
    GridCalibrationTargetCirclegrid::CirclegridOptions target_options;
    GridCalibrationTargetCirclegrid::Ptr target( new GridCalibrationTargetCirclegrid(rows, cols, spacingMeters, target_options) );

    // create a camera:
    boost::shared_ptr<DistortedPinholeRsCameraGeometry> geometry( new DistortedPinholeRsCameraGeometry() );

    // create a detector:
    GridDetector::GridDetectorOptions detector_options;
    GridDetector detector(geometry, target, detector_options);

    // load the test image and extract the target:
    cv::Mat image;
    image = cv::imread("testImageCircleGrid.jpg", 0);// force grayscale
    if (image.data == NULL) {
      std::cout << "Circlegrid test image not found" << std::endl;
    }
    ASSERT_TRUE(image.data != NULL);

    //extract the target
    GridCalibrationTargetObservation obs;
    ASSERT_TRUE( detector.findTargetNoTransformation(image, obs) );

    SCOPED_TRACE("");
  }
  catch(const std::exception &e) {
    FAIL() << e.what();
  }
}

TEST(GridCalibrationTestSuite, testNonSquareCheckerboardIntrinsicsInit)
{
  using namespace aslam::cameras;
  sm::logging::setLevel(sm::logging::levels::Debug);
  try {
    // Test 1: rectangular target with cols > rows (e.g. 6 rows x 11 cols)
    {
      const size_t rows = 6;
      const size_t cols = 11;
      GridCalibrationTargetCheckerboard::CheckerboardOptions options;
      GridCalibrationTargetCheckerboard::Ptr target(
          new GridCalibrationTargetCheckerboard(rows, cols, 0.05, 0.05, options));

      GridCalibrationTargetObservation obs(target);
      cv::Mat img(480, 640, CV_8UC1, cv::Scalar(0));
      obs.setImage(img);

      for (size_t r = 0; r < rows; ++r) {
        double cx = 150.0 + r * 30.0;
        double cy = 240.0;
        double radius = 120.0;
        for (size_t c = 0; c < cols; ++c) {
          double angle = static_cast<double>(c) / (cols - 1) * (M_PI / 2.0);
          Eigen::Vector2d pt(cx + radius * std::cos(angle), cy + radius * std::sin(angle));
          obs.updateImagePoint(target->gridCoordinatesToPoint(r, c), pt);
        }
      }

      std::vector<GridCalibrationTargetObservation> observations;
      observations.push_back(obs);
      PinholeProjection<NoDistortion> pinhole;

      ASSERT_TRUE(pinhole.initializeIntrinsics(observations));
      EXPECT_GT(pinhole.fu(), 0.0);
      EXPECT_GT(pinhole.fv(), 0.0);
      EXPECT_TRUE(std::isfinite(pinhole.fu()));
      EXPECT_TRUE(std::isfinite(pinhole.fv()));
    }

    // Test 2: rectangular target with rows > cols (e.g. 11 rows x 6 cols)
    {
      const size_t rows = 11;
      const size_t cols = 6;
      GridCalibrationTargetCheckerboard::CheckerboardOptions options;
      GridCalibrationTargetCheckerboard::Ptr target(
          new GridCalibrationTargetCheckerboard(rows, cols, 0.05, 0.05, options));

      GridCalibrationTargetObservation obs(target);
      cv::Mat img(480, 640, CV_8UC1, cv::Scalar(0));
      obs.setImage(img);

      for (size_t r = 0; r < rows; ++r) {
        double cx = 150.0 + r * 15.0;
        double cy = 240.0;
        double radius = 120.0;
        for (size_t c = 0; c < cols; ++c) {
          double angle = static_cast<double>(c) / (cols - 1) * (M_PI / 2.0);
          Eigen::Vector2d pt(cx + radius * std::cos(angle), cy + radius * std::sin(angle));
          obs.updateImagePoint(target->gridCoordinatesToPoint(r, c), pt);
        }
      }

      std::vector<GridCalibrationTargetObservation> observations;
      observations.push_back(obs);
      PinholeProjection<NoDistortion> pinhole;

      ASSERT_TRUE(pinhole.initializeIntrinsics(observations));
      EXPECT_GT(pinhole.fu(), 0.0);
      EXPECT_GT(pinhole.fv(), 0.0);
      EXPECT_TRUE(std::isfinite(pinhole.fu()));
      EXPECT_TRUE(std::isfinite(pinhole.fv()));
    }

    SCOPED_TRACE("");
  }
  catch (const std::exception & e) {
    FAIL() << e.what();
  }
}

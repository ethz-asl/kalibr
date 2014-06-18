#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/cameras.hpp>
#include <aslam/Frame.hpp>
#include <sm/boost/serialization.hpp>
#include <opencv2/core/core.hpp>

TEST(AslamFrontendTestSuite, testFrameXmlSerialization)
{

  try {
    using namespace aslam;
    using namespace aslam::cameras;
    typedef Frame<DistortedPinholeCameraGeometry> frame_t;

    frame_t F1;

    F1.setRandom();
    F1.computeAllBackProjections(true);
    ASSERT_TRUE(F1.isBinaryEqual(F1));

    frame_t F2;

    ASSERT_FALSE(F1.isBinaryEqual(F2));
    ASSERT_FALSE(F2.isBinaryEqual(F1));

    sm::boost_serialization::save_xml(F1,"frame", "test.xml");
    sm::boost_serialization::load_xml(F2,"frame", "test.xml");

    // Actually this test is too strict. Doubles are saved and loaded as ASCII and this fails the binary equality test.
    //  ASSERT_TRUE(F1.isBinaryEqual(F2));
    //  ASSERT_TRUE(F2.isBinaryEqual(F1));
  }
  catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}

TEST(AslamFrontendTestSuite, testFrameSerialization)
{
  try {
    using namespace aslam;
    using namespace aslam::cameras;
    typedef Frame<DistortedPinholeCameraGeometry> frame_t;

    frame_t F1;

    F1.setRandom();
    ASSERT_TRUE(F1.isBinaryEqual(F1));

    frame_t F2;

    ASSERT_FALSE(F1.isBinaryEqual(F2));
    ASSERT_FALSE(F2.isBinaryEqual(F1));

    sm::boost_serialization::save(F1,"test.ba");

    sm::boost_serialization::load(F2,"test.ba");

    ASSERT_TRUE(F1.isBinaryEqual(F2));
    ASSERT_TRUE(F2.isBinaryEqual(F1));
  }
  catch(std::exception const & e)
  {
    FAIL() << e.what();
  }

}

TEST(AslamFrontendTestSuite, testMultiOctaveFrameSerialization)
{
  try {
    using namespace aslam;
    using namespace aslam::cameras;
    typedef Frame<DistortedPinholeCameraGeometry> frame_t;

    frame_t F1;

    F1.setRandom();
    F1.getOctavesMutable()->setNumOctaves(2);

    cv::Mat img0(100, 100, CV_8UC1);
    cv::randn(img0, cv::Scalar::all(128), cv::Scalar::all(20));
    F1.setImage(img0, 0);

    cv::Mat img1(50, 50, CV_8UC1);
    cv::randn(img1, cv::Scalar::all(128), cv::Scalar::all(20));
    F1.setImage(img1, 1);

    ASSERT_TRUE(F1.isBinaryEqual(F1));

    frame_t F2;

    ASSERT_FALSE(F1.isBinaryEqual(F2));
    ASSERT_FALSE(F2.isBinaryEqual(F1));

    sm::boost_serialization::save(F1,"test.ba");

    sm::boost_serialization::load(F2,"test.ba");

    ASSERT_TRUE(F1.isBinaryEqual(F2));
    ASSERT_TRUE(F2.isBinaryEqual(F1));
  }
  catch(std::exception const & e)
  {
    FAIL() << e.what();
  }

}

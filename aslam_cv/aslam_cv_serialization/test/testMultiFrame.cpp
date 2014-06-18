#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <aslam/MultiFrame.hpp>
#include <aslam/NCameraSystem.hpp>
#include <sm/boost/serialization.hpp>
#include <aslam/cameras.hpp>

#include <sm/kinematics/transformations.hpp>
#include <sm/kinematics/UncertainTransformation.hpp>
#include <sm/string_routines.hpp>
#include <aslam/feature_converters.hpp>
#include <aslam/SurfDescriptor.hpp>
#include <boost/serialization/export.hpp>
#include <boost/make_shared.hpp>
#include <aslam/FrameBaseSerialization.hpp>

typedef aslam::NCameraSystem MaskedDistortedCameraSystem;
typedef aslam::Frame<aslam::cameras::MaskedDistortedOmniCameraGeometry> MDOCGFrame;

boost::shared_ptr<aslam::cameras::MaskedDistortedOmniCameraGeometry> createTestCamera() {
  boost::shared_ptr<aslam::cameras::MaskedDistortedOmniCameraGeometry> camera(
      new aslam::cameras::MaskedDistortedOmniCameraGeometry(
          aslam::cameras::OmniProjection<
              aslam::cameras::RadialTangentialDistortion>(1.0,    // xi
              400.0,  // fu (or gamma x)
              400.0,  // fv (or gamma y)
              640.0,  // cu (image center x)
              480.0,  // cv (image center y)
              1280.0,  // resolution u
              960.0,  // resolution v
              aslam::cameras::RadialTangentialDistortion(0.001,  // k1
                  0.001,  // k2
                  0.001,  // p1
                  0.001  // p2
                  ))));

  camera->setMask(aslam::cameras::ImageMask(cv::Mat(0, 0, CV_8UC1), 1.0));
  // If you have a mask:
  return camera;
}

boost::shared_ptr<MaskedDistortedCameraSystem> createTestCameraSystem() {
  // Save some typing.
  typedef boost::shared_ptr<aslam::cameras::MaskedDistortedOmniCameraGeometry> cam_t;
  // Build the extrinsics
  // FRONT_CAMERA = 0,
  cam_t front = createTestCamera();
  boost::shared_ptr<sm::kinematics::Transformation> T_odometry_front(
      new sm::kinematics::Transformation);
  T_odometry_front->setRandom();
  // LEFT_CAMERA = 1,
  cam_t left = createTestCamera();
  boost::shared_ptr<sm::kinematics::Transformation> T_odometry_left(
      new sm::kinematics::Transformation);
  T_odometry_left->setRandom();
  // BACK_CAMERA = 2,
  cam_t back = createTestCamera();
  boost::shared_ptr<sm::kinematics::Transformation> T_odometry_back(
      new sm::kinematics::Transformation);
  T_odometry_back->setRandom();
  // RIGHT_CAMERA = 3
  cam_t right = createTestCamera();
  boost::shared_ptr<sm::kinematics::Transformation> T_odometry_right(
      new sm::kinematics::Transformation);
  T_odometry_right->setRandom();

  aslam::NCameraSystem::TransformationVector vT;
  aslam::NCameraSystem::CameraGeometryVector vG;

  vT.push_back((T_odometry_front));
  vG.push_back(front);
  vT.push_back((T_odometry_left));
  vG.push_back(left);
  vT.push_back((T_odometry_back));
  vG.push_back(back);
  vT.push_back((T_odometry_right));
  vG.push_back(right);

  boost::shared_ptr<MaskedDistortedCameraSystem> cameraSystem(
      new MaskedDistortedCameraSystem(vT, vG, false));

  return cameraSystem;
}

boost::shared_ptr<
    aslam::Frame<aslam::cameras::MaskedDistortedOmniCameraGeometry> > buildTestFrame(
    boost::shared_ptr<
        aslam::Frame<aslam::cameras::MaskedDistortedOmniCameraGeometry> > & frontFrame) {
  assert(frontFrame);

  frontFrame->setId(aslam::FrameId(2));
  frontFrame->setTime(aslam::Time(rand(), rand()));

  // Add keypoints. see aslam_frames/include/aslam/Keypoint.hpp
  // if you have OpenCV SURF keypoints, please use:
  //
  // void convertSurfKeypoints(const std::vector<cv::KeyPoint>& ocvKeypoints, 
  //                           const std::vector<float> & ocvDescriptors, 
  //                           FrameBuilder::keypoint_list_t & outKeypoints);
  // 
  // defined in aslam_features2d/include/aslam/feature_converters.hpp
  // But note, then you still have to set the landmark/landmark ID
  const int N = 10;
  for (int i = 0; i < N; ++i) {
    // For each keypoint, here are all the things we need filled in:
    aslam::Keypoint<2> kp;

    // 1. The measurement
    kp.setMeasurement(Eigen::Vector2d::Random());

    int octave = 0;
    // 2. The measurement covariance.
    // from aslam_features2d/include/aslam/features_converters.hpp
    // here we base it on the SURF octave where the keypoint was found.
    double recip_sigma = aslam::octaveToInverseUncertainty(octave);
    kp.setInverseMeasurementCovariance(
        Eigen::Matrix2d::Identity(2, 2) * recip_sigma);

    // 3. the landmark
    // This point is 4x1 and has a 4x4 covariance. To set using a Euclidean point, just fill in 
    // [ x y z 1 ]^T and the top left corner of the covariance matrix (set the rest of the
    // matrix to zero)
    sm::kinematics::UncertainHomogeneousPoint point;
    point.setRandom();
    kp.setLandmark(point);

    // 4. the landmark id
    kp.setLandmarkId(aslam::LandmarkId(4));

    // 5. the SURF descriptor.
    std::vector<float> surfDescriptor;
    surfDescriptor.resize(64);
    bool laplacianBit = 1;
    float size = 2;
    float angle = 0;
    float response = 1;

    // The Surf descriptor will copy the values
    // and the keypoint will take ownership of the pointer.
    kp.setDescriptorRawPtr(
        new aslam::SurfDescriptor(&surfDescriptor[0], laplacianBit, size,
                                  octave, angle, response));

    /// add the keypoint to the frame
    /// see aslam_frames/include/aslam/Frame.hpp
    frontFrame->addKeypoint(kp);
  }

  // Set the raw image in the frame.
  cv::Mat frontImage(0, 0, CV_8UC1);
  frontFrame->setImage(frontImage);
  return frontFrame;
}

boost::shared_ptr<aslam::MultiFrame> buildTestMultiFrame() {
  //Create a CameraSystem with stub values
  boost::shared_ptr<MaskedDistortedCameraSystem> cameraSystem;
  cameraSystem = createTestCameraSystem();

  // These unique ids should match those on the wiki.
  cameraSystem->setId(aslam::CameraSystemId(1));

  // Create a multi-camera frame and set the camera system.
  // aslam_camera_system/include/aslam/MultiFrame.hpp
  boost::shared_ptr<aslam::MultiFrame> frame(
      new aslam::MultiFrame(cameraSystem));
  int sec = 0;
  int nsec = 0;
  frame->setTime(aslam::Time(sec, nsec));

  // see aslam_frames/include/aslam/Frame.hpp
  for (int c = 0; c < 4; ++c) {
    boost::shared_ptr<
        aslam::Frame<aslam::cameras::MaskedDistortedOmniCameraGeometry> > f;
    f.reset(
        new aslam::Frame<aslam::cameras::MaskedDistortedOmniCameraGeometry>);
    buildTestFrame(f);
    f->setGeometry(
        cameraSystem->geometryAs
            < aslam::cameras::MaskedDistortedOmniCameraGeometry > (c));
    frame->setFrame(c, f);
  }
  return frame;
}

TEST(AslamCameraSystemTestSuite, testCameraSystemSerialization)
{
  try
  {
    using namespace aslam;

    //Create a CameraSystem with stub values
    boost::shared_ptr<MaskedDistortedCameraSystem> c1;
    c1 = createTestCameraSystem();
    // These unique ids should match those on the wiki.
    c1->setId(aslam::CameraSystemId(1));
    ASSERT_TRUE(SM_CHECKSAME(c1, c1));

    boost::shared_ptr<MaskedDistortedCameraSystem> c2(new MaskedDistortedCameraSystem);

    ASSERT_FALSE(SM_CHECKSAME(c1, c2, false));

    ASSERT_FALSE(SM_CHECKSAME(c2, c1, false));

    sm::boost_serialization::save(*c1, "test.ba");

    sm::boost_serialization::load(*c2, "test.ba");

    ASSERT_TRUE(SM_CHECKSAME(c1, c2));
    ASSERT_TRUE(SM_CHECKSAME(c2, c1));
  }
  catch(const std::exception& e)
  {
    FAIL() << e.what();
  }
}

TEST(AslamCameraSystemTestSuite, testFrameSerialization)
{
  //boost::serialization::void_cast_register< aslam::SurfDescriptor, aslam::DescriptorBase>(static_cast<aslam::SurfDescriptor *>(NULL), static_cast< aslam::DescriptorBase * >(NULL) );
  aslam::linkCvSerialization();

  try
  {
    using namespace aslam;

    boost::shared_ptr<aslam::Frame<aslam::cameras::MaskedDistortedOmniCameraGeometry> > f1;
    f1.reset( new aslam::Frame<aslam::cameras::MaskedDistortedOmniCameraGeometry> );
    buildTestFrame( f1 );

    boost::shared_ptr<aslam::Frame<aslam::cameras::MaskedDistortedOmniCameraGeometry> > f2;

    //std::cout << "Serializing.\n";
    std::string filename = "frame.ba";
    sm::boost_serialization::save(f1, filename);
    sm::boost_serialization::load(f2, filename);

    //std::cout << "Asserting equal.\n";
    ASSERT_TRUE(SM_CHECKSAME(f1, f2));
    ASSERT_TRUE(SM_CHECKSAME(f2, f1));
  }
  catch(const std::exception& e)
  {
    FAIL() << e.what();
  }
}

TEST(AslamCameraSystemTestSuite, testMultiFrameSerialization)
{
  boost::serialization::void_cast_register< aslam::SurfDescriptor, aslam::DescriptorBase>(static_cast<aslam::SurfDescriptor *>(NULL), static_cast< aslam::DescriptorBase * >(NULL) );

  try
  {
    using namespace aslam;

    boost::shared_ptr<MultiFrame> f1;
    //f1.setRandom();   //TODO: Implement.
    f1 = buildTestMultiFrame();

    MultiFrameId fId(0);
    f1->setId(fId);

    //std::cout << "Asserting self-equal.\n";
    ASSERT_TRUE(SM_CHECKSAME(f1, f1));

    boost::shared_ptr<MultiFrame> f2(new MultiFrame);

    //std::cout << "Asserting non-equal.\n";
    ASSERT_FALSE(SM_CHECKSAME(f1, f2, false));
    ASSERT_FALSE(SM_CHECKSAME(f2, f1, false));

    std::string filename = "mframe.ba";

    sm::boost_serialization::save(f1, filename);
    sm::boost_serialization::load(f2, filename);

    ASSERT_TRUE(SM_CHECKSAME(f1, f2));
    ASSERT_TRUE(SM_CHECKSAME(f2, f1));
  }
  catch(const std::exception& e)
  {
    FAIL() << e.what();
  }
}

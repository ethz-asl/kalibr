#include <sm/eigen/gtest.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/Frame.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/random.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer.hpp>
//#include <aslam/FrameTypedefs.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/cameras.hpp>

class HpErr : public aslam::backend::ErrorTermFs<4> {
 public:
  aslam::backend::HomogeneousPoint _p;
  typedef aslam::backend::ErrorTermFs<4> parent_t;
  HpErr(const Eigen::Vector4d & p)
      : _p(p) {
    _p.setActive(true);
    _p.setBlockIndex(0);
    parent_t::setDesignVariables(&_p);
    Eigen::MatrixXd invR = parent_t::invR();
    invR.setIdentity();
    parent_t::setInvR(invR);
  }
  virtual ~HpErr() {
  }

  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation() {
    parent_t::setError(_p.toHomogeneous());

    return parent_t::error().dot(parent_t::invR() * parent_t::error());
  }

  /// \brief evaluate the jacobian

  virtual void evaluateJacobiansImplementation(
      aslam::backend::JacobianContainer & J) {
    _p.evaluateJacobians(J);
  }

};

TEST(ReprojectionErrorTestSuite, testSimpleError)
{
  try
  {
    using namespace aslam::backend;
    HpErr e(Eigen::Vector4d::Random());
    JacobianContainer estJ(4);
    e.evaluateJacobiansFiniteDifference(estJ);

    JacobianContainer J(4);
    e.evaluateJacobians(J);

    SCOPED_TRACE("");
    sm::eigen::assertNear(J.asDenseMatrix(), estJ.asDenseMatrix(), 1e-6, SM_SOURCE_FILE_POS, "Checking the jacobian vs. finite differences");

  }
  catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}

///*
TEST(ReprojectionErrorTestSuite, testReprojectionError)
{
  try {

    using namespace aslam;
    using namespace aslam::backend;
    using namespace aslam::cameras;
    typedef Frame<OmniCameraGeometry> frame_t;
    typedef frame_t::camera_geometry_t camera_geometry_t;
    typedef frame_t::keypoint_t keypoint_t;
    typedef ReprojectionError<camera_geometry_t> error_t;

    boost::shared_ptr<camera_geometry_t> geometry( new camera_geometry_t( camera_geometry_t::getTestGeometry() ) );
    // create the camera design variable:
    CameraDesignVariable<camera_geometry_t> cameraDv(geometry);

    frame_t frame;
    frame.setGeometry(geometry);

    Eigen::Matrix2d invR;
    invR.setIdentity();

    /// N keypoints per frame
    const int N = 1;

    for(int n = 0; n < N; n++)
    {
      keypoint_t k;
      Eigen::Vector4d p = sm::kinematics::toHomogeneous(geometry->createRandomVisiblePoint());
      boost::shared_ptr<aslam::backend::HomogeneousPoint> pt( new aslam::backend::HomogeneousPoint(p) );
      pt->setActive(true);
      pt->setBlockIndex(0);
      HomogeneousExpression hep(pt);

      Eigen::Vector2d kp;
      geometry->homogeneousToKeypoint(hep.toHomogeneous(), kp);
      k.setMeasurement(kp);
      k.setInverseMeasurementCovariance(invR);
      frame.addKeypoint(k);

      error_t e(&frame, n, hep, cameraDv);

      //ASSERT_NEAR(e.evaluateError(), 0.0, 1e-14);

      // Get the jacobians by finite difference.
      JacobianContainer estJ(2);
      e.evaluateJacobiansFiniteDifference(estJ);

      JacobianContainer J(2);
      e.evaluateJacobians(J);

      SCOPED_TRACE("");
      sm::eigen::assertNear(J.asDenseMatrix(), estJ.asDenseMatrix(), 1e-6, SM_SOURCE_FILE_POS, "Checking the jacobian vs. finite differences");

    }
  }
  catch(const std::exception & e)
  {
    FAIL() << e.what();
  }

}

using namespace aslam;
using namespace aslam::backend;

TEST(ReprojectionErrorTestSuite, testEstimator)
{
  try
  {

    using namespace aslam;
    using namespace aslam::backend;
    using namespace aslam::cameras;
    typedef Frame<OmniCameraGeometry> frame_t;
    typedef frame_t::camera_geometry_t camera_geometry_t;
    typedef frame_t::keypoint_t keypoint_t;
    typedef ReprojectionError<camera_geometry_t> error_t;

    boost::shared_ptr<camera_geometry_t> geometry( new camera_geometry_t( camera_geometry_t::getTestGeometry() ) );

    // create the camera design variable:
    CameraDesignVariable<camera_geometry_t> cameraDv(geometry);

    // N keypoints per frame
    const int N = 200;
    // K frames
    const int K = 5;
    // noise on keypoints
    const double sigma = 1.0;
    // noise on initial guess
    const double perturbQuat = 0.01;
    const double perturbTrans = 0.01;

    Eigen::Matrix2d invR;
    invR.setIdentity();

    // Step 1. Generate the trajectory
    boost::ptr_vector<sm::kinematics::Transformation> T_w_ck;
    boost::ptr_vector<aslam::backend::EuclideanPoint> t_w_ck_w;
    boost::ptr_vector<aslam::backend::RotationQuaternion> q_w_ck;
    boost::ptr_vector<frame_t> frame_k;
    boost::ptr_vector<aslam::backend::HomogeneousPoint> landmarks;

    boost::shared_ptr<OptimizationProblem> problem_ptr( new OptimizationProblem );
    OptimizationProblem & problem = *problem_ptr;

    // add the camera stuff to the problem
    problem.addDesignVariable(cameraDv.projectionDesignVariable());
    problem.addDesignVariable(cameraDv.distortionDesignVariable());
    problem.addDesignVariable(cameraDv.shutterDesignVariable());

    // start at identity
    sm::kinematics::Transformation T_w_c;
    sm::kinematics::Transformation T_ck_ckp1;
    // Start at identity.    
    for(int k = 0; k < K; ++k)
    {
      T_w_ck.push_back( new sm::kinematics::Transformation( T_w_c) );
      t_w_ck_w.push_back( new aslam::backend::EuclideanPoint( T_w_c.t() ) );
      q_w_ck.push_back( new aslam::backend::RotationQuaternion( T_w_c.q() ) );

      T_ck_ckp1.setRandom( 0.5, sm::kinematics::deg2rad(3.0) );
      T_w_c = T_w_c * T_ck_ckp1;

      frame_k.push_back(new frame_t);
      frame_k[k].setGeometry( geometry );

      // Setting these poses to active means they will be estimated.
      q_w_ck[k].setActive(true);
      t_w_ck_w[k].setActive(true);

      // Add the design variables to the problem In this case, we pass in raw pointers
      // and tell the optimization problem that it doesn't own these variables.
      // In this case, it is the users responsibility to ensure that these pointers don't
      // go out of scope while the problem exists.
      problem.addDesignVariable(&q_w_ck[k], false);
      problem.addDesignVariable(&t_w_ck_w[k], false);
    }

    // Step 2. Create some visible landmarks.

    for(int k = 0; k < K; ++k)
    {
      for(int n = 0; n < N; ++n)
      {
        // create a point.
        Eigen::Vector4d p_ck = sm::kinematics::toHomogeneous(frame_k[k].geometry().createRandomVisiblePoint(sm::random::randLU(1.0,10.0)));

        // the aslam::backend::HomogeneousPoint class is a design variable.
        landmarks.push_back( new aslam::backend::HomogeneousPoint(T_w_ck[k] * p_ck) );
        problem.addDesignVariable( &landmarks.back(), false );
      }
    }

    std::set<int> activeLandmarks;

    // Step 3. Project all of the landmarks into the images to create some keypoint measurements.
    for(size_t l = 0; l < landmarks.size(); ++l)
    {
      int seenCount = 0;
      for(int k = 0; k < K; k++)
      {

        sm::kinematics::Transformation T_ck_w = T_w_ck[k].inverse();
        // Transform the landmark into the current camera frame.
        Eigen::Vector4d p_ck = T_ck_w * landmarks[l].toHomogeneous();
        // Project it into the image using this frame's camera geometry.
        Eigen::Vector2d y;
        frame_k[k].geometry().homogeneousToKeypoint(p_ck, y);

        // Check if the projection is valid and the point is in front of the camera.
        if( frame_k[k].geometry().isValid(y) && sm::kinematics::fromHomogeneous(p_ck)[2] > 0.0)
        {
          ++seenCount;
        }
      }

      if(seenCount > 1)
      {
        // If we see this landmark in 2+ frames
        // add it to the list of landmarks in the estimation problem.
        activeLandmarks.insert(l);
        // Keeping 10 landmarks fixed will make sure this is an observable system. This is a hack.
        if(activeLandmarks.size() > 10)
        {
          // Past 10 landmarks we will estimate the position
          landmarks[l].setActive(true);
          // setting this to marginalized will enable the Schur complement trick for these variables.
          landmarks[l].setMarginalized(true);
        }

        // Now we add all of the keypoint measurements
        LandmarkId lid(l);
        for(int k = 0; k < K; k++)
        {
          // Create a transformation expression from the design variables.
          TransformationExpression T_w_ck( q_w_ck[k].toExpression(), t_w_ck_w[k].toExpression() );

          // Invert this expression to get a new expression that takes points from the world frame to the camera frame.
          TransformationExpression T_ck_w = T_w_ck.inverse();

          // Multiplying the above expression with a landmark HomogeneousPointExpression makes a new HomogeneousPointExpression
          // representing the landmark expressed in the camera frame.
          HomogeneousExpression p_ck = T_ck_w * landmarks[l].toExpression();

          // Project this into the image.
          Eigen::Vector2d y;
          frame_k[k].geometry().homogeneousToKeypoint(p_ck.toHomogeneous(), y);

          // Again, check if this is a valid projection and in front of the camera.
          if( frame_k[k].geometry().isValid(y) && sm::kinematics::fromHomogeneous(p_ck.toHomogeneous())[2] > 0.0 )
          {
            // Now we can add an error term...First add the measurement to the frame.
            keypoint_t kp;
            kp.setMeasurement(y + Eigen::Vector2d(sm::random::randn() * sigma, sm::random::randn() * sigma ));
            kp.setInverseMeasurementCovariance(invR);
            kp.setLandmarkId(lid);
            frame_k[k].addKeypoint(kp);

            // Then add an error term into the optimization problem.
            boost::shared_ptr<error_t> error(new error_t(&frame_k[k], frame_k[k].numKeypoints() - 1, p_ck, cameraDv));
            problem.addErrorTerm(error);
          }
        }
      }  // end if this landmark was seen by more than one pose.	
    }  // end for each landmark

    // Perturb the initial guesses a little bit;
    for(size_t i = 0; i < q_w_ck.size(); i++)
    {
      Eigen::Vector3d up = Eigen::Vector3d::Random() * perturbQuat;
      q_w_ck[i].update(&up[0],3);
    }
    for(size_t i = 0; i < t_w_ck_w.size(); i++)
    {
      Eigen::Vector3d up = Eigen::Vector3d::Random() * perturbTrans;
      t_w_ck_w[i].update(&up[0],3);
    }

    // Okay, now we have a bunch of design variables in "q_w_ck", "t_w_ck_w", and "landmarks".
    // And a bunch of reprojection errors in "errors".
    // These are packed in to the variable "problem"
    OptimizerOptions options;
    options.verbose = true;
    options.linearSolver = "cholmod";
    options.levenbergMarquardtLambdaInit = 10;
    options.doSchurComplement = true;
    options.doLevenbergMarquardt = true;
    Optimizer optimizer(options);
    optimizer.setProblem( problem_ptr );
    optimizer.options().verbose = true;
    optimizer.optimize();

  }
  catch(const std::exception & e)
  {
    FAIL() << e.what();
  }
}

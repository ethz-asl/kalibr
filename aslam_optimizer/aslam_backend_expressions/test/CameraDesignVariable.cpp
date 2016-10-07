#include <sm/eigen/gtest.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/cameras.hpp>
#include <aslam/backend/JacobianContainer.hpp>


TEST(AslamBackendTestSuite, testCameraDesignVariable) {

	using namespace aslam::cameras;
	using namespace aslam::backend;
	typedef DistortedPinholeCameraGeometry cam_t;
	// create test geometry
	cam_t c = cam_t::getTestGeometry();
	boost::shared_ptr<cam_t> camera(&c, sm::null_deleter());
	// create design variable:
	CameraDesignVariable<cam_t> camDv(camera);

	// active all except shutter
	camDv.setActive(true,true,false);
	camDv.setTestBlockIndices();

    // test distortion and distored jacobian:
    Eigen::Vector3d p = camera->createRandomVisiblePoint();
    Eigen::Vector4d ph = Eigen::Vector4d::Ones();
    ph.segment(0,3) = p;
    Eigen::Vector2d k = camera->createRandomKeypoint();

    Eigen::Vector2d k1;
    camera->euclideanToKeypoint(p, k1);

    JacobianContainer Jc(2);
    camDv.evaluateJacobians(Jc, ph);

  //  std::cout << "Jc" << std::endl << Jc.asDenseMatrix() << std::endl;


    Eigen::MatrixXd estJi;
    camera->euclideanToKeypointIntrinsicsJacobianFiniteDifference(p, estJi);
    Eigen::MatrixXd estJd;
    camera->euclideanToKeypointDistortionJacobianFiniteDifference(p, estJd);


    Eigen::MatrixXd estJ(estJi.rows(), estJi.cols() + estJd.cols());
    estJ.block(0,0,estJi.rows(), estJi.cols()) = estJi;
    estJ.block(0,estJi.cols(), estJd.rows(), estJd.cols()) = estJd;

  //  std::cout << "estJi" << std::endl << estJi << std::endl;
  //  std::cout << "estJd" << std::endl << estJd << std::endl;
  //  std::cout << "estJ" << std::endl << estJ << std::endl;
    sm::eigen::assertNear(Jc.asDenseMatrix(),-estJ, 1e-5, SM_SOURCE_FILE_POS);


}




#include <kalibr_errorterms/AccelerometerError.hpp>
#include <kalibr_errorterms/GyroscopeError.hpp>
#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>

// GyroscopeError(const Eigen::Vector3d & measurement, const Eigen::Matrix3d & invR, const aslam::backend::EuclideanExpression & angularVelocity, const aslam::backend::EuclideanExpression & bias );

// AccelerometerError(const Eigen::Vector3d & measurement, const Eigen::Matrix3d invR,
//                    aslam::backend::RotationExpression C_b_w, aslam::backend::EuclideanExpression acceleration_w,
//                    aslam::backend::EuclideanExpression bias, aslam::backend::EuclideanExpression g_w);

TEST(ImuCameraTests, testAccelerometer) {
	// AccelerometerError(const Eigen::Vector3d & measurement, const Eigen::Matrix3d invR,
	//                    aslam::backend::RotationExpression C_b_w, aslam::backend::EuclideanExpression acceleration_w,
	//                    aslam::backend::EuclideanExpression bias, aslam::backend::EuclideanExpression g_w);
	using namespace aslam::backend;
	using namespace kalibr_errorterms;

	RotationQuaternion q_b_w(sm::kinematics::quatRandom());
	EuclideanPoint a_w(Eigen::Vector3d::Random());
	EuclideanPoint bias(Eigen::Vector3d::Random());
	EuclideanPoint gravity(Eigen::Vector3d::Random());

	AccelerometerError aerr(Eigen::Vector3d::Random(),
			Eigen::Matrix3d::Identity(), q_b_w.toExpression(),
			a_w.toExpression(), bias.toExpression(),
			gravity.toExpression());

	ErrorTermTestHarness<3> harness(&aerr);
	harness.testAll(1e-5);
}

TEST(ImuCameraTests, testGyroscope) {
	// GyroscopeError(const Eigen::Vector3d & measurement, const Eigen::Matrix3d & invR, const aslam::backend::EuclideanExpression & angularVelocity, const aslam::backend::EuclideanExpression & bias );

	using namespace aslam::backend;
	using namespace kalibr_errorterms;

	EuclideanPoint a_w(Eigen::Vector3d::Random());
	EuclideanPoint bias(Eigen::Vector3d::Random());

	GyroscopeError gerr(Eigen::Vector3d::Random(), Eigen::Matrix3d::Identity(),
			a_w.toExpression(), bias.toExpression());

	ErrorTermTestHarness<3> harness(&gerr);

	harness.testAll(1e-5);
}

TEST(ImuCameraTests, testEccentricGyroscope) {
	using namespace aslam::backend;
	using namespace kalibr_errorterms;

	MatrixBasic M(Eigen::Matrix3d::Random());
	MatrixBasic Ma(Eigen::Matrix3d::Random());
	RotationQuaternion q_b_w(sm::kinematics::quatRandom());
	EuclideanPoint acceleration_w(Eigen::Vector3d::Random());
	EuclideanPoint angularVelocity_b(Eigen::Vector3d::Random());
	EuclideanPoint angularAcceleration_b(Eigen::Vector3d::Random());
	RotationQuaternion q_i_b(sm::kinematics::quatRandom());
	EuclideanPoint r_b(Eigen::Vector3d::Random());
	EuclideanPoint bias(Eigen::Vector3d::Random());
	EuclideanPoint g_w(Eigen::Vector3d::Random());

	GyroscopeErrorEccentric gerr(Eigen::Vector3d::Random(), Eigen::Matrix3d::Identity(),
			M.toExpression(), Ma.toExpression(), q_b_w.toExpression(), acceleration_w.toExpression(),
			angularVelocity_b.toExpression(), angularAcceleration_b.toExpression(), q_i_b.toExpression(),
			r_b.toExpression(), bias.toExpression(), g_w.toExpression());

	ErrorTermTestHarness<3> harness(&gerr);

	harness.testAll(1e-5);
}

TEST(ImuCameraTests, testEccentricAccelerometer) {
	using namespace aslam::backend;
	using namespace kalibr_errorterms;

	MatrixBasic M(Eigen::Matrix3d::Random());
	RotationQuaternion q_b_w(sm::kinematics::quatRandom());
	EuclideanPoint acceleration_w(Eigen::Vector3d::Random());
	EuclideanPoint angularVelocity_b(Eigen::Vector3d::Random());
	EuclideanPoint angularAcceleration_b(Eigen::Vector3d::Random());
	RotationQuaternion q_i_b(sm::kinematics::quatRandom());
	EuclideanPoint rx_b(Eigen::Vector3d::Random());
	EuclideanPoint ry_b(Eigen::Vector3d::Random());
	EuclideanPoint rz_b(Eigen::Vector3d::Random());
	EuclideanPoint bias(Eigen::Vector3d::Random());
	EuclideanPoint g_w(Eigen::Vector3d::Random());


	AccelerometerErrorEccentric aerr(Eigen::Vector3d::Random(), Eigen::Matrix3d::Identity(),
			M.toExpression(), q_b_w.toExpression(), acceleration_w.toExpression(),
			angularVelocity_b.toExpression(), angularAcceleration_b.toExpression(), q_i_b.toExpression(),
			rx_b.toExpression(), ry_b.toExpression(), rz_b.toExpression(), bias.toExpression(),
			g_w.toExpression());


	ErrorTermTestHarness<3> harness(&aerr);

	harness.testAll(1e-5);
}

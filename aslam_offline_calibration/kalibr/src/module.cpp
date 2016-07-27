// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <kalibr_errorterms/EuclideanError.hpp>
#include <kalibr_errorterms/GyroscopeError.hpp>
#include <kalibr_errorterms/AccelerometerError.hpp>

// The title of this library must match exactly
BOOST_PYTHON_MODULE(libkalibr_errorterms_python)
{
	using namespace boost::python;
	using namespace kalibr_errorterms;
	using namespace aslam::backend;

//    class_<EuclideanError, boost::shared_ptr<EuclideanError>, boost::noncopyable, bases< ErrorTerm > >("EuclideanError", no_init);
    class_<EuclideanError, boost::shared_ptr<EuclideanError>, boost::noncopyable, bases< ErrorTerm > >("EuclideanError", init<const Eigen::Vector3d & , const Eigen::Matrix3d & ,
     const aslam::backend::EuclideanExpression &>("EuclideanError(measurement, invR, predicted_measurement)"))
    .def("getMeasurement", &EuclideanError::getMeasurement)
    .def("getPredictedMeasurement", &EuclideanError::getPredictedMeasurement);

	// fill this in with boost::python export code
	class_<GyroscopeError, boost::shared_ptr<GyroscopeError>, bases< EuclideanError > >
	("GyroscopeError", init<const Eigen::Vector3d & , const Eigen::Matrix3d & ,
			const aslam::backend::EuclideanExpression &, const aslam::backend::EuclideanExpression & >
	("GyroscopeError(measurement, invR, angularVelocity, bias)"));

	class_<AccelerometerError, boost::shared_ptr<AccelerometerError>, bases< EuclideanError > >
	("AccelerometerError", init<const Eigen::Vector3d &, const Eigen::Matrix3d &,
			const aslam::backend::RotationExpression &, const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &, const aslam::backend::EuclideanExpression &>
	("AccelerometerError(measurement, invR, C_b_w,  acceleration_w, bias, g_w)"));

	class_<GyroscopeNoBiasError, boost::shared_ptr<GyroscopeNoBiasError>, bases< ErrorTerm > >
	("GyroscopeNoBiasError", init<const Eigen::Vector3d & , const Eigen::Matrix3d & ,
			const aslam::backend::EuclideanExpression & >
	("GyroscopeError(measurement, invR, angularVelocity)"));

	class_<AccelerometerErrorEccentric, boost::shared_ptr<AccelerometerErrorEccentric>,
	bases<EuclideanError> >
	("AccelerometerErrorEccentric", init<
			const Eigen::Vector3d &,
			const Eigen::Matrix3d &,
			const aslam::backend::MatrixExpression &,
			const aslam::backend::RotationExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::RotationExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &>
	("AccelerometerErrorEccentric(measurement, invR, M, C_b_w, acceleration_w, angularVelocity_b, angularAcceleration_b,)"
			"C_i_b, rx_b, ry_b, rz_b, bias, g_w"));

	class_<GyroscopeErrorEccentric, boost::shared_ptr<GyroscopeErrorEccentric>,
	bases<EuclideanError> >
	("GyroscopeErrorEccentric", init<
			const Eigen::Vector3d &,
			const Eigen::Matrix3d &,
			const aslam::backend::MatrixExpression &,
			const aslam::backend::MatrixExpression &,
			const aslam::backend::RotationExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::RotationExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &,
			const aslam::backend::EuclideanExpression &>
	("GyroscopeErrorEccentric(measurement, invR, M, Ma, C_b_w, acceleration_w, angularVelocity_b, angularAcceleration_b,)"
			"C_i_b, r_b, bias, g_w"));

}

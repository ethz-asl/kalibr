#include <aslam/backend/ErrorTerm.hpp>
#include <laser_errorterms/LaserError.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

BOOST_PYTHON_MODULE(liblaser_errorterms_python)
{
	using namespace aslam::backend;
	using namespace boost::python;
	using namespace laser_errorterms;

	class_<LaserError, boost::shared_ptr<LaserError>, bases< ErrorTerm > >("LaserError", init<const double & , const Eigen::Matrix<double,1,1> &, const Eigen::Vector3d &,
			const aslam::backend::TransformationExpression &, const aslam::backend::EuclideanExpression &,
			const aslam::backend::ScalarExpression &, const aslam::backend::ScalarExpression & >("LaserError(measurement, invR, direction, T, normal, offset, bias)"));

	class_<ScalarError, boost::shared_ptr<ScalarError>, bases< ErrorTerm > >("ScalarError", init<const double & , const Eigen::Matrix<double,1,1> &,
			const aslam::backend::ScalarExpression & >("ScalarError(measurement, invR, predictedMeasurement)"));
}

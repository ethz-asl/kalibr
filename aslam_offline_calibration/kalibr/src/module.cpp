// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <kalibr_errorterms/GyroscopeError.hpp>
#include <kalibr_errorterms/AccelerometerError.hpp>

// The title of this library must match exactly
BOOST_PYTHON_MODULE(libkalibr_errorterms_python)
{
    using namespace boost::python;
    using namespace kalibr_errorterms;
    using namespace aslam::backend;

    // fill this in with boost::python export code
    class_<GyroscopeError, boost::shared_ptr<GyroscopeError>, bases< ErrorTerm > >
    ("GyroscopeError", init<const Eigen::Vector3d & , const Eigen::Matrix3d & ,
     const aslam::backend::EuclideanExpression &, const aslam::backend::EuclideanExpression & >
    ("GyroscopeError(measurement, invR, angularVelocity, bias)"))
    .def("getMeasurement", &GyroscopeError::getMeasurement)
    .def("getPredictedMeasurement", &GyroscopeError::getPredictedMeasurement);

    class_<AccelerometerError, boost::shared_ptr<AccelerometerError>, bases<ErrorTerm> >
    ("AccelerometerError", init<const Eigen::Vector3d &, const Eigen::Matrix3d,
     aslam::backend::RotationExpression , aslam::backend::EuclideanExpression,
     aslam::backend::EuclideanExpression, aslam::backend::EuclideanExpression>
    ("AccelerometerError(measurement, invR, C_b_w,  acceleration_w, bias, g_w)"))
    .def("getMeasurement", &AccelerometerError::getMeasurement)
    .def("getPredictedMeasurement", &AccelerometerError::getPredictedMeasurement);

    class_<GyroscopeNoBiasError, boost::shared_ptr<GyroscopeNoBiasError>, bases< ErrorTerm > >
    ("GyroscopeNoBiasError", init<const Eigen::Vector3d & , const Eigen::Matrix3d & ,
     const aslam::backend::EuclideanExpression & >
    ("GyroscopeError(measurement, invR, angularVelocity)"));

}

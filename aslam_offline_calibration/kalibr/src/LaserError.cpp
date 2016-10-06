#include <laser_errorterms/LaserError.hpp>

namespace laser_errorterms
{

ScalarError::ScalarError(const double & measurement, const Eigen::Matrix<double,1,1> & invR, const aslam::backend::ScalarExpression & predictedMeasurement) :
        				_measurement(measurement), _predictedMeasurement(predictedMeasurement)
{
	setInvR( invR );

	aslam::backend::DesignVariable::set_t dvs;
	_predictedMeasurement.getDesignVariables(dvs);
	setDesignVariablesIterator(dvs.begin(), dvs.end());
	//Perform an initial error evaluation so that reasonable a priori errors can be retrieved.
	evaluateError();
}

ScalarError::~ScalarError() {}

double ScalarError::evaluateErrorImplementation()
{
	_evaluatedErrorTerm(0,0) = _predictedMeasurement.toScalar() - _measurement;
	setError(_evaluatedErrorTerm);

	return evaluateChiSquaredError();
}

void ScalarError::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & _jacobians)
{
	_predictedMeasurement.evaluateJacobians(_jacobians);
}


LaserError::LaserError(const double & measurement, const Eigen::Matrix<double,1,1> & invR, const Eigen::Vector3d & p, const aslam::backend::TransformationExpression & rt, const aslam::backend::EuclideanExpression & normal, const aslam::backend::ScalarExpression & offset, const aslam::backend::ScalarExpression & bias) :
        				_measurement(measurement), _p(p), _rt(rt), _normal(normal), _offset(offset), _bias(bias), _predictedMeasurement(0.0)
{
	setInvR( invR );

	_predictedMeasurement = ((_normal.dot(_rt.toEuclideanExpression()) - _offset))/(_normal.dot(_rt.toRotationExpression()*_p)) + _bias;

	//TODO(jrn) This seems troublesome! Fix that in the evaluation and Jacobian instead!
	if (_predictedMeasurement.toScalar() < 0)
	{
		_predictedMeasurement = (_offset - (_normal.dot(_rt.toEuclideanExpression())))/(_normal.dot(_rt.toRotationExpression()*_p)) + _bias;
	}

	aslam::backend::DesignVariable::set_t dvs;
	_predictedMeasurement.getDesignVariables(dvs);
	setDesignVariablesIterator(dvs.begin(), dvs.end());
}

LaserError::~LaserError()
{

}

double LaserError::evaluateErrorImplementation()
{
	_evaluatedErrorTerm(0,0) = _predictedMeasurement.toScalar() - _measurement;
	setError(_evaluatedErrorTerm);

	return evaluateChiSquaredError();
}

void LaserError::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & _jacobians)
{
	_predictedMeasurement.evaluateJacobians(_jacobians);
}


} // namespace laser_errorterms

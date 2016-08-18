
#include <aslam/backend/MatrixBasic.hpp>

#include <sm/kinematics/rotations.hpp>
#include <aslam/Exceptions.hpp>


namespace aslam {
namespace backend {

MatrixBasic::MatrixBasic(const Eigen::Matrix3d & A) : _A(A), _p_A(A)
, _B(Eigen::MatrixXd::Identity(9,9)) {
}

MatrixBasic::MatrixBasic(const Eigen::Matrix3d & A,
		const Eigen::Matrix3i & UpdatePattern) : _A(A), _p_A(A) {
	_B = Eigen::MatrixXd::Zero(9,9);
	size_t non_zero_elements = 0;
	for (size_t row=0; row<UpdatePattern.rows(); ++row) {
		for(size_t col=0; col<UpdatePattern.cols(); ++col) {
			if(UpdatePattern(row, col) != 0) {
				_B(col * UpdatePattern.rows() + row, non_zero_elements) = 1.0;
				++non_zero_elements;
			}
		}
	}
	_B.conservativeResize(Eigen::NoChange, non_zero_elements);
}

MatrixBasic::~MatrixBasic(){}


/// \brief Revert the last state update.
void MatrixBasic::revertUpdateImplementation()
{
	_A = _p_A;
}

/// \brief Update the design variable.
void MatrixBasic::updateImplementation(const double * dp, int size)
{
	Eigen::VectorXd d = Eigen::Map<const Eigen::VectorXd>(dp, _B.cols());
	Eigen::Matrix<double, 9, 1> dV = _B*d;
	_p_A = _A;
	_A += Eigen::Map<Eigen::Matrix<double, 3, 3>>(dV.data());
}

int MatrixBasic::minimalDimensionsImplementation() const
{
	return _B.cols(); // number of unknowns in the matrix
}

// ##
Eigen::Matrix3d MatrixBasic::toMatrix3x3Implementation()
{
	return _A;
}

///## will not be used
void MatrixBasic::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
{
	if (minimalDimensionsImplementation() != 0)
	{
		outJacobians.add( const_cast<MatrixBasic *>(this), Eigen::Matrix3d::Identity());
	}
}


void MatrixBasic::evaluateJacobiansImplementation(JacobianContainer & outJacobians,
		const Eigen::MatrixXd & applyChainRule) const
{
	if (minimalDimensionsImplementation() != 0)
	{
		outJacobians.add( const_cast<MatrixBasic*>(this), applyChainRule *_B );
	}
}

MatrixExpression MatrixBasic::toExpression()
{
	return MatrixExpression(this);
}

void MatrixBasic::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
{
	if (minimalDimensionsImplementation() != 0)
	{
		designVariables.insert(const_cast<MatrixBasic*>(this));
	}
}

void MatrixBasic::getParametersImplementation(
		Eigen::MatrixXd& value) const {
	value = _A;
}

void MatrixBasic::setParametersImplementation(
		const Eigen::MatrixXd& value) {
	_p_A = _A;
	_A = value;
}

} // namespace backend
} // namespace aslam


#ifndef ASLAM_BACKEND_DV_MATRIX_TRAFO_HPP
#define ASLAM_BACKEND_DV_MATRIX_TRAFO_HPP


#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>
#include "MatrixExpression.hpp"
#include "MatrixExpressionNode.hpp"

namespace aslam {
namespace backend {

class MatrixBasic : public MatrixExpressionNode, public DesignVariable
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MatrixBasic(const Eigen::Matrix3d & A);
	MatrixBasic(const Eigen::Matrix3d & A, bool add_dv);
	MatrixBasic(const Eigen::Matrix3d & A, const Eigen::Matrix3i & UpdatePattern);

	virtual ~MatrixBasic();

	/// \brief Revert the last state update.
	virtual void revertUpdateImplementation();

	/// \brief Update the design variable.
	virtual void updateImplementation(const double * dp, int size);

	/// \brief the size of an update step
	virtual int minimalDimensionsImplementation() const;

	MatrixExpression toExpression();
private:
	virtual Eigen::Matrix3d toMatrix3x3Implementation();
	virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
	virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
	virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

	/// Returns the content of the design variable
	virtual void getParametersImplementation(Eigen::MatrixXd& value) const;

	/// Sets the content of the design variable
	virtual void setParametersImplementation(const Eigen::MatrixXd& value);

	Eigen::Matrix3d _A;
	Eigen::Matrix3d _p_A;
	Eigen::Matrix<double, 9, Eigen::Dynamic> _B; //9xN (N<=9) update matrix derived from updatePattern
};

} // namespace backend
} // namespace aslam

#endif /* ASLAM_BACKEND_DV_MATRIX_TRAFO_HPP */

#ifndef ASLAM_BACKEND_DV_MATRIX_TRAFO_HPP
#define ASLAM_BACKEND_DV_MATRIX_TRAFO_HPP


#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>
#include "MatrixExpression.hpp"
#include "MatrixExpressionNode.hpp"

namespace aslam {
  namespace backend {
    
    class MatrixTransformation : public MatrixExpressionNode, public DesignVariable
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        MatrixTransformation(const Eigen::Matrix3d & A);
        MatrixTransformation(const Eigen::Matrix3d & A, const Eigen::Matrix3d & UpdatePattern);

      virtual ~MatrixTransformation();

      /// \brief Revert the last state update.
      virtual void revertUpdateImplementation();

      /// \brief Update the design variable.
      virtual void updateImplementation(const double * dp, int size);

      /// \brief the size of an update step
      virtual int minimalDimensionsImplementation() const;

      MatrixExpression toExpression();
    private:
      virtual Eigen::Matrix3d toFullMatrixImplementation();
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      /// Returns the content of the design variable
      virtual void getParametersImplementation(Eigen::MatrixXd& value) const;

      /// Sets the content of the design variable
      virtual void setParametersImplementation(const Eigen::MatrixXd& value);

      /// Computes the minimal distance in tangent space between the current value of the DV and xHat
      virtual void minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const;

      /// Computes the minimal distance in tangent space between the current value of the DV and xHat and the jacobian
      virtual void minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const;


      Eigen::Matrix3d _A;
      Eigen::Matrix3d _A_a;
      Eigen::Matrix3d _UpdatePattern;		// 3x3 Matrix containing 1 at the position that should be estimated and 0 for constants
      int _UpdateDimension;					// Number of elements in the matrix, which are estimated by the calibration
      
    };

  } // namespace backend
} // namespace aslam

#endif /* ASLAM_BACKEND_DV_MATRIX_TRAFO_HPP */

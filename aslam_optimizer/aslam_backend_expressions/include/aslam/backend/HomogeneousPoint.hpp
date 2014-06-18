#ifndef ASLAM_BACKEND_HOMOGENEOUS_POINT_HPP
#define ASLAM_BACKEND_HOMOGENEOUS_POINT_HPP

#include "HomogeneousExpressionNode.hpp"
#include "HomogeneousExpression.hpp"

#include <aslam/backend/DesignVariable.hpp>

namespace aslam {
  namespace backend {
    
    class HomogeneousPoint : public HomogeneousExpressionNode, public DesignVariable
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      HomogeneousPoint(const Eigen::Vector4d & p);
      virtual ~HomogeneousPoint();

      /// \brief Revert the last state update.
      virtual void revertUpdateImplementation();

      /// \brief Update the design variable.
      virtual void updateImplementation(const double * dp, int size);

      /// \brief the size of an update step
      virtual int minimalDimensionsImplementation() const;

      HomogeneousExpression toExpression();
    private:
      virtual Eigen::Vector4d toHomogeneousImplementation() const;

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

    private:
      /// \brief The current value of the design variable.
      Eigen::Vector4d _p;

      /// \brief The previous version of the design variable.
      Eigen::Vector4d _p_p;


    };
    
  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_HOMOGENEOUS_POINT_HPP */

#ifndef ASLAM_BACKEND_TV_QUAT_HPP
#define ASLAM_BACKEND_TV_QUAT_HPP

#include "TransformationExpressionNode.hpp"
#include <Eigen/Core>
#include "RotationExpression.hpp"
#include "EuclideanExpression.hpp"

namespace aslam {
  namespace backend {
    
    class TransformationBasic : public TransformationExpressionNode
    {
    public:
      typedef Eigen::Matrix<double, 6, 6> Matrix6d;
      TransformationBasic(RotationExpression C_0_1, EuclideanExpression t_0_1_0);
      virtual ~TransformationBasic();

      TransformationExpression toExpression();

    private:
      virtual Eigen::Matrix4d toTransformationMatrixImplementation();
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;
      
    private:
      boost::shared_ptr<RotationExpressionNode> _rotation;
      boost::shared_ptr<EuclideanExpressionNode>  _translation;
    };

  } // namespace backend
} // namespace aslam

#endif /* ASLAM_BACKEND_DV_QUAT_HPP */

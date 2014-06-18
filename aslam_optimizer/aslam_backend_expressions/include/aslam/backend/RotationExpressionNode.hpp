#ifndef ASLAM_BACKEND_DV_ROTATION_HPP
#define ASLAM_BACKEND_DV_ROTATION_HPP


#include <aslam/backend/JacobianContainer.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <boost/shared_ptr.hpp>
#include <set>
#include <aslam/backend/TransformationExpressionNode.hpp>

namespace aslam {
  namespace backend {
    
    /**
     * \class RotationExpressionNode
     * \brief The superclass of all classes representing rotations.
     */
    class RotationExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      RotationExpressionNode();
      virtual ~RotationExpressionNode();

      /// \brief Evaluate the rotation matrix.
      Eigen::Matrix3d toRotationMatrix() const;
      
      /// \brief Evaluate the Jacobians
      void evaluateJacobians(JacobianContainer & outJacobians) const;
    
      /// \brief Evaluate the Jacobians and apply the chain rule.
      void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      void getDesignVariables(DesignVariable::set_t & designVariables) const;
    protected:        
      // These functions must be implemented by child classes.
      virtual Eigen::Matrix3d toRotationMatrixImplementation() const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const = 0;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const = 0;

    };


    /**
     * \class RotationExpressionNodeMultiply
     *
     * \brief A class representing the multiplication of two rotation matrices.
     * 
     */
    class RotationExpressionNodeMultiply : public RotationExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      RotationExpressionNodeMultiply(boost::shared_ptr<RotationExpressionNode> lhs, 
				     boost::shared_ptr<RotationExpressionNode> rhs);
      virtual ~RotationExpressionNodeMultiply();

    private:
      virtual Eigen::Matrix3d toRotationMatrixImplementation() const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<RotationExpressionNode> _lhs;
      mutable Eigen::Matrix3d _C_lhs;
      boost::shared_ptr<RotationExpressionNode> _rhs;
      mutable Eigen::Matrix3d _C_rhs;
    };


    /**
     * \class RotationExpressionNodeInverse
     * 
     * \brief A class representing the inverse of a rotation matrix.
     *
     */
    class RotationExpressionNodeInverse : public RotationExpressionNode
    {
    public:
      RotationExpressionNodeInverse(boost::shared_ptr<RotationExpressionNode> dvRotation);

      virtual ~RotationExpressionNodeInverse();

    private:
      virtual Eigen::Matrix3d toRotationMatrixImplementation() const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<RotationExpressionNode> _dvRotation;
      mutable Eigen::Matrix3d _C;
    };

    class RotationExpressionNodeTransformation : public RotationExpressionNode
    {
    public:
      RotationExpressionNodeTransformation(boost::shared_ptr<TransformationExpressionNode> dvRotation);

      virtual ~RotationExpressionNodeTransformation();

    private:
      virtual Eigen::Matrix3d toRotationMatrixImplementation() const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<TransformationExpressionNode> _transformation;
    };

  

  } // namespace backend
} // namespace aslam



#endif /* ASLAM_BACKEND_DV_ROTATION_HPP */

#ifndef ASLAM_TRANSFORMATION_EXPRESSION_NODE
#define ASLAM_TRANSFORMATION_EXPRESSION_NODE

#include <Eigen/Core>
#include <aslam/backend/JacobianContainer.hpp>
#include <boost/shared_ptr.hpp>
#include <set>

namespace aslam {
  namespace backend {
    class EuclideanExpression;
    class RotationExpression;
    
    class TransformationExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef Eigen::Matrix<double,6,6> Matrix6d;
      typedef Eigen::Matrix<double,4,6> Matrix4x6d;

      TransformationExpressionNode();
      virtual ~TransformationExpressionNode();

      /// \brief Evaluate the rotation matrix.
      Eigen::Matrix4d toTransformationMatrix();
      
      /// \brief Evaluate the Jacobians
      void evaluateJacobians(JacobianContainer & outJacobians) const;   
      template <typename DERIVED>
      EIGEN_ALWAYS_INLINE void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixBase<DERIVED> & applyChainRule) const {
        evaluateJacobians(outJacobians.apply(applyChainRule));
      }
      void getDesignVariables(DesignVariable::set_t & designVariables) const;
    protected:
      // These functions must be implemented by child classes.
      virtual Eigen::Matrix4d toTransformationMatrixImplementation() = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const = 0;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const = 0;

      virtual RotationExpression toRotationExpression(const boost::shared_ptr<TransformationExpressionNode> & thisShared) const;
      virtual EuclideanExpression toEuclideanExpression(const boost::shared_ptr<TransformationExpressionNode> & thisShared) const;
      
      friend class TransformationExpression;
    };

    

    /**
     * \class TransformationExpressionNodeMultiply
     *
     * \brief A class representing the multiplication of two transformation matrices.
     * 
     */
    class TransformationExpressionNodeMultiply : public TransformationExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef Eigen::Matrix<double,6,6> Matrix6d;

      TransformationExpressionNodeMultiply(boost::shared_ptr<TransformationExpressionNode> lhs, 
				     boost::shared_ptr<TransformationExpressionNode> rhs);
      virtual ~TransformationExpressionNodeMultiply();

    private:
      virtual Eigen::Matrix4d toTransformationMatrixImplementation();
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<TransformationExpressionNode> _lhs;
      Eigen::Matrix4d _T_lhs;
      boost::shared_ptr<TransformationExpressionNode> _rhs;
      Eigen::Matrix4d _T_rhs;
    };


    /**
     * \class TransformationExpressionNodeInverse
     * 
     * \brief A class representing the inverse of a transformation matrix.
     *
     */
    class TransformationExpressionNodeInverse : public TransformationExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef Eigen::Matrix<double,6,6> Matrix6d;

      TransformationExpressionNodeInverse(boost::shared_ptr<TransformationExpressionNode> dvTransformation);

      virtual ~TransformationExpressionNodeInverse();

    private:
      virtual Eigen::Matrix4d toTransformationMatrixImplementation();
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<TransformationExpressionNode> _dvTransformation;
      Eigen::Matrix4d _T;
    };


    /**
     * \class TransformationExpressionNodeMultiply
     *
     * \brief A class representing the multiplication of two transformation matrices.
     * 
     */
    class TransformationExpressionNodeConstant : public TransformationExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TransformationExpressionNodeConstant(const Eigen::Matrix4d & T);
        virtual ~TransformationExpressionNodeConstant();

    private:
      virtual Eigen::Matrix4d toTransformationMatrixImplementation();
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;


      Eigen::Matrix4d _T;
    };



  } // namespace backend
} // namespace aslam


#endif /* ASLAM_TRANSFORMATION_EXPRESSION_NODE */

#ifndef ASLAM_BACKEND_HOMOGENEOUS_EXPRESSION_NODE_HPP
#define ASLAM_BACKEND_HOMOGENEOUS_EXPRESSION_NODE_HPP

#include <aslam/backend/JacobianContainer.hpp>
#include "TransformationExpressionNode.hpp"
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include "EuclideanExpression.hpp"


namespace aslam {
  namespace backend {
    class TransformationExpressionNode;
        
    /**
     * \class HomogeneousExpressionNode
     * \brief The superclass of all classes representing homogeneous points.
     */
    class HomogeneousExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      HomogeneousExpressionNode();
      virtual ~HomogeneousExpressionNode();

      /// \brief Evaluate the homogeneous matrix.
      Eigen::Vector4d toHomogeneous() const;
      
      /// \brief Evaluate the Jacobians
      void evaluateJacobians(JacobianContainer & outJacobians) const;
    
      /// \brief Evaluate the Jacobians and apply the chain rule.
      void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariables(DesignVariable::set_t & designVariables) const;
    protected:        
      // These functions must be implemented by child classes.
      virtual Eigen::Vector4d toHomogeneousImplementation() const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const = 0;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const = 0;

    };


    /**
     * \class HomogeneousExpressionNodeMultiply
     *
     * \brief A class representing the transformation of a homogeneous point
     * 
     */
    class HomogeneousExpressionNodeMultiply : public HomogeneousExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      HomogeneousExpressionNodeMultiply(boost::shared_ptr<TransformationExpressionNode> lhs, 
				     boost::shared_ptr<HomogeneousExpressionNode> rhs);
      virtual ~HomogeneousExpressionNodeMultiply();

    private:
      virtual Eigen::Vector4d toHomogeneousImplementation() const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<TransformationExpressionNode> _lhs;
      mutable Eigen::Matrix4d _T_lhs;
      boost::shared_ptr<HomogeneousExpressionNode> _rhs;
      mutable Eigen::Vector4d _p_rhs;
    };


    class HomogeneousExpressionNodeConstant : public HomogeneousExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      HomogeneousExpressionNodeConstant(const Eigen::Vector4d & p);
      virtual ~HomogeneousExpressionNodeConstant();

        void set(const Eigen::Vector4d & p){ _p = p; }
    private:
      virtual Eigen::Vector4d toHomogeneousImplementation() const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      Eigen::Vector4d _p;
    };

  class HomogeneousExpressionNodeEuclidean : public HomogeneousExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      HomogeneousExpressionNodeEuclidean( boost::shared_ptr<EuclideanExpressionNode> p);
      virtual ~HomogeneousExpressionNodeEuclidean();

    private:
      virtual Eigen::Vector4d toHomogeneousImplementation() const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<EuclideanExpressionNode> _p;
    };


  } // namespace backend
} // namespace aslam

#endif /* ASLAM_BACKEND_HOMOGENEOUS_EXPRESSION_NODE_HPP */

#ifndef ASLAM_BACKEND_SCALAR_EXPRESSION_NODE_HPP
#define ASLAM_BACKEND_SCALAR_EXPRESSION_NODE_HPP

#include <aslam/backend/JacobianContainer.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <aslam/backend/VectorExpressionNode.hpp>


namespace aslam {
  namespace backend {


    /**
     * \class ScalarExpressionNode
     * \brief The superclass of all classes representing scalar points.
     */
    class ScalarExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ScalarExpressionNode();
      virtual ~ScalarExpressionNode();

      /// \brief Evaluate the scalar matrix.
      double toScalar() const;

      /// \brief Evaluate the Jacobians
      void evaluateJacobians(JacobianContainer & outJacobians) const;

      /// \brief Evaluate the Jacobians and apply the chain rule.
      void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      void getDesignVariables(DesignVariable::set_t & designVariables) const;
    protected:
      // These functions must be implemented by child classes.
      virtual double toScalarImplementation() const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const = 0;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const = 0;
    };


      class ScalarExpressionNodeMultiply : public ScalarExpressionNode
      {
      public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          ScalarExpressionNodeMultiply(boost::shared_ptr<ScalarExpressionNode> lhs,
                                       boost::shared_ptr<ScalarExpressionNode> rhs);
          virtual ~ScalarExpressionNodeMultiply();
      protected:
          // These functions must be implemented by child classes.
          virtual double toScalarImplementation() const;
          virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
          virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
          virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

          boost::shared_ptr<ScalarExpressionNode> _lhs;
          boost::shared_ptr<ScalarExpressionNode> _rhs;

    };

      class ScalarExpressionNodeDivide : public ScalarExpressionNode
      {
      public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          ScalarExpressionNodeDivide(boost::shared_ptr<ScalarExpressionNode> lhs,
                                       boost::shared_ptr<ScalarExpressionNode> rhs);
          virtual ~ScalarExpressionNodeDivide();
      protected:
          // These functions must be implemented by child classes.
          virtual double toScalarImplementation() const;
          virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
          virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
          virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

          boost::shared_ptr<ScalarExpressionNode> _lhs;
          boost::shared_ptr<ScalarExpressionNode> _rhs;

    };

      class ScalarExpressionNodeAdd : public ScalarExpressionNode
      {
      public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          ScalarExpressionNodeAdd(boost::shared_ptr<ScalarExpressionNode> lhs,
                                  boost::shared_ptr<ScalarExpressionNode> rhs,
                                  double multiplyRhs = 1.0);
          virtual ~ScalarExpressionNodeAdd();

       protected:
          // These functions must be implemented by child classes.
          virtual double toScalarImplementation() const;
          virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
          virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
          virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

          boost::shared_ptr<ScalarExpressionNode> _lhs;
          boost::shared_ptr<ScalarExpressionNode> _rhs;
          double _multiplyRhs;
    };


      class ScalarExpressionNodeConstant  : public ScalarExpressionNode
      {
      public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          ScalarExpressionNodeConstant(double s);
          virtual ~ScalarExpressionNodeConstant();

      protected:
          // These functions must be implemented by child classes.
          virtual double toScalarImplementation() const{return _s;}
        virtual void evaluateJacobiansImplementation(JacobianContainer & /* outJacobians */) const{}
        virtual void evaluateJacobiansImplementation(JacobianContainer & /* outJacobians */, const Eigen::MatrixXd & /* applyChainRule */) const{}
        virtual void getDesignVariablesImplementation(DesignVariable::set_t & /* designVariables */) const{}

          double _s;
      };

      class ScalarExpressionNodeFromVectorExpression : public ScalarExpressionNode
      {
      public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          ScalarExpressionNodeFromVectorExpression(boost::shared_ptr<VectorExpressionNode<1> > lhs);
          virtual ~ScalarExpressionNodeFromVectorExpression();

       protected:
          // These functions must be implemented by child classes.
          virtual double toScalarImplementation() const;
          virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
          virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
          virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

          boost::shared_ptr<VectorExpressionNode<1> > _lhs;
    };


  } // namespace backend
} // namespace aslam

#endif /* ASLAM_BACKEND_EUCLIDEAN_EXPRESSION_NODE_HPP */

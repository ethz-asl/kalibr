#ifndef ASLAM_BACKEND_EUCLIDEAN_EXPRESSION_NODE_HPP
#define ASLAM_BACKEND_EUCLIDEAN_EXPRESSION_NODE_HPP

#include <aslam/backend/JacobianContainer.hpp>
#include "RotationExpressionNode.hpp"
#include "ScalarExpressionNode.hpp"
#include "TransformationExpressionNode.hpp"
#include "MatrixExpressionNode.hpp"
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <sm/kinematics/RotationalKinematics.hpp>

namespace aslam {
  namespace backend {
    template <int D> class VectorExpression;
    template <int D> class VectorExpressionNode;
  class HomogeneousExpressionNode;
    /**
     * \class EuclideanExpressionNode
     * \brief The superclass of all classes representing euclidean points.
     */
    class EuclideanExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      EuclideanExpressionNode();
      virtual ~EuclideanExpressionNode();

      /// \brief Evaluate the euclidean matrix.
      Eigen::Vector3d toEuclidean() const;
      
      /// \brief Evaluate the Jacobians
      void evaluateJacobians(JacobianContainer & outJacobians) const;
    
      /// \brief Evaluate the Jacobians and apply the chain rule.
      void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      void getDesignVariables(DesignVariable::set_t & designVariables) const;
    protected:        
      // These functions must be implemented by child classes.
      virtual Eigen::Vector3d toEuclideanImplementation() const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const = 0;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const = 0;
    };


    /**
     * \class EuclideanExpressionNodeMultiply
     *
     * \brief A class representing the multiplication of two euclidean matrices.
     * 
     */
    class EuclideanExpressionNodeMultiply : public EuclideanExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      EuclideanExpressionNodeMultiply(boost::shared_ptr<RotationExpressionNode> lhs, 
				     boost::shared_ptr<EuclideanExpressionNode> rhs);
      virtual ~EuclideanExpressionNodeMultiply();

    private:
      virtual Eigen::Vector3d toEuclideanImplementation() const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<RotationExpressionNode> _lhs;
      mutable Eigen::Matrix3d _C_lhs;
      boost::shared_ptr<EuclideanExpressionNode> _rhs;
      mutable Eigen::Vector3d _p_rhs;


    };

    // ## New Class for Multiplication with a MatrixExpression
    /**
      * \class EuclideanExpressionNodeMatrixMultiply
      *
      * \brief A class representing the multiplication of a Matrix with a euclidean Vector.
      *
      */
     class EuclideanExpressionNodeMatrixMultiply : public EuclideanExpressionNode
     {
     public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       EuclideanExpressionNodeMatrixMultiply(boost::shared_ptr<MatrixExpressionNode> lhs, boost::shared_ptr<EuclideanExpressionNode> rhs);
       virtual ~EuclideanExpressionNodeMatrixMultiply();

     private:
       virtual Eigen::Vector3d toEuclideanImplementation() const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
       virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

       boost::shared_ptr<MatrixExpressionNode> _lhs;
       mutable Eigen::Matrix3d _A_lhs;
       boost::shared_ptr<EuclideanExpressionNode> _rhs;
       mutable Eigen::Vector3d _p_rhs;

     };



    /**
      * \class EuclideanExpressionNodeCrossEuclidean
      *
      * \brief A class representing the cross product of two euclidean expressions.
      *
      */
     class EuclideanExpressionNodeCrossEuclidean : public EuclideanExpressionNode
     {
     public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       EuclideanExpressionNodeCrossEuclidean(boost::shared_ptr<EuclideanExpressionNode> lhs,
           boost::shared_ptr<EuclideanExpressionNode> rhs);
       virtual ~EuclideanExpressionNodeCrossEuclidean();

     private:
       virtual Eigen::Vector3d toEuclideanImplementation() const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
       virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

       boost::shared_ptr<EuclideanExpressionNode> _lhs;
       boost::shared_ptr<EuclideanExpressionNode> _rhs;
     };


     /**
       * \class EuclideanExpressionNodeAddEuclidean
       *
       * \brief A class representing the addition of two euclidean expressions.
       *
       */
      class EuclideanExpressionNodeAddEuclidean : public EuclideanExpressionNode
      {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EuclideanExpressionNodeAddEuclidean(boost::shared_ptr<EuclideanExpressionNode> lhs,
            boost::shared_ptr<EuclideanExpressionNode> rhs);
        virtual ~EuclideanExpressionNodeAddEuclidean();

      private:
        virtual Eigen::Vector3d toEuclideanImplementation() const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
        virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

        boost::shared_ptr<EuclideanExpressionNode> _lhs;
        boost::shared_ptr<EuclideanExpressionNode> _rhs;
      };


      /**
      * \class EuclideanExpressionNodeSubtractEuclidean
      *
      * \brief A class representing the subtraction of two Euclidean expressions.
      *
      */
     class EuclideanExpressionNodeSubtractEuclidean : public EuclideanExpressionNode
     {
     public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       EuclideanExpressionNodeSubtractEuclidean(boost::shared_ptr<EuclideanExpressionNode> lhs,
           boost::shared_ptr<EuclideanExpressionNode> rhs);
       virtual ~EuclideanExpressionNodeSubtractEuclidean();

     private:
       virtual Eigen::Vector3d toEuclideanImplementation() const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
       virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

       boost::shared_ptr<EuclideanExpressionNode> _lhs;
       boost::shared_ptr<EuclideanExpressionNode> _rhs;
     };

     /**
     * \class EuclideanExpressionNodeConstant
     *
     * \brief A class representing a constant Euclidean expressions.
     *
     */
     class EuclideanExpressionNodeConstant : public EuclideanExpressionNode
     {
     public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       EuclideanExpressionNodeConstant(const Eigen::Vector3d & p);
       virtual ~EuclideanExpressionNodeConstant();

         void set(const Eigen::Vector3d & p){ _p = p; }
     private:
         virtual Eigen::Vector3d toEuclideanImplementation() const;
         virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
         virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
         virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

         Eigen::Vector3d _p;
     };

    /**
      * \class EuclideanExpressionNodeSubtractVector
      *
      * \brief A class representing the subtraction of a vector from an Euclidean expression.
      *
      */
     class EuclideanExpressionNodeSubtractVector : public EuclideanExpressionNode
     {
     public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       EuclideanExpressionNodeSubtractVector(boost::shared_ptr<EuclideanExpressionNode> lhs,
              const Eigen::Vector3d & rhs);
       virtual ~EuclideanExpressionNodeSubtractVector();

     private:
       virtual Eigen::Vector3d toEuclideanImplementation() const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
       virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

       boost::shared_ptr<EuclideanExpressionNode> _lhs;
       Eigen::Vector3d _rhs;
     };


     /**
       * \class EuclideanExpressionNodeSubtractVector
       *
       * \brief A class representing the negated Euclidean expression.
       *
       */
      class EuclideanExpressionNodeNegated : public EuclideanExpressionNode
      {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EuclideanExpressionNodeNegated(boost::shared_ptr<EuclideanExpressionNode> operand);
        virtual ~EuclideanExpressionNodeNegated();

      private:
        virtual Eigen::Vector3d toEuclideanImplementation() const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
        virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

        boost::shared_ptr<EuclideanExpressionNode> _operand;
      };

     /**
       * \class EuclideanExpressionNodeScalarMultiply
       *
       * \brief A class representing the multiplication of a ScalarExpression with an Euclidean expression.
       *
       */
      class EuclideanExpressionNodeScalarMultiply : public EuclideanExpressionNode
      {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EuclideanExpressionNodeScalarMultiply(boost::shared_ptr<EuclideanExpressionNode> p, boost::shared_ptr<ScalarExpressionNode> s);
        virtual ~EuclideanExpressionNodeScalarMultiply();

      private:
        virtual Eigen::Vector3d toEuclideanImplementation() const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
        virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

        boost::shared_ptr<EuclideanExpressionNode> _p;
        boost::shared_ptr<ScalarExpressionNode> _s;
      };

     /**
       * \class VectorExpression2EuclideanExpressionAdapter
       *
       * \brief A class representing an adapted VectorExpression<3>.
       *
       */
      class VectorExpression2EuclideanExpressionAdapter : public EuclideanExpressionNode
      {
      public:
        VectorExpression2EuclideanExpressionAdapter(boost::shared_ptr<VectorExpressionNode<3> > vectorExpressionNode);
        virtual ~VectorExpression2EuclideanExpressionAdapter();

      private:
        virtual Eigen::Vector3d toEuclideanImplementation() const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
        virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

        boost::shared_ptr<VectorExpressionNode<3> > _vectorExpressionNode;
      };

      class EuclideanExpressionNodeTranslation : public EuclideanExpressionNode
      {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EuclideanExpressionNodeTranslation(boost::shared_ptr<TransformationExpressionNode> operand);
        virtual ~EuclideanExpressionNodeTranslation();

      private:
        virtual Eigen::Vector3d toEuclideanImplementation() const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
        virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

        boost::shared_ptr<TransformationExpressionNode> _operand;
      };


  class EuclideanExpressionNodeRotationParameters : public EuclideanExpressionNode
      {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EuclideanExpressionNodeRotationParameters(boost::shared_ptr<RotationExpressionNode> operand, sm::kinematics::RotationalKinematics::Ptr rk);
        virtual ~EuclideanExpressionNodeRotationParameters();

      private:
        virtual Eigen::Vector3d toEuclideanImplementation() const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
        virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

        boost::shared_ptr<RotationExpressionNode> _operand;
        sm::kinematics::RotationalKinematics::Ptr _rk;
      };


  class EuclideanExpressionNodeFromHomogeneous : public EuclideanExpressionNode
      {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EuclideanExpressionNodeFromHomogeneous(boost::shared_ptr<HomogeneousExpressionNode> root);
        virtual ~EuclideanExpressionNodeFromHomogeneous();

      private:
        virtual Eigen::Vector3d toEuclideanImplementation() const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
        virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
        virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

        boost::shared_ptr<HomogeneousExpressionNode> _root;
      };

    /**
      * \class EuclideanExpressionNodeElementwiseMultiplyEuclidean
      *
      * \brief A class representing the elementwise product of two euclidean expressions.
      *
      */
     class EuclideanExpressionNodeElementwiseMultiplyEuclidean : public EuclideanExpressionNode
     {
     public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       EuclideanExpressionNodeElementwiseMultiplyEuclidean(boost::shared_ptr<EuclideanExpressionNode> lhs,
           boost::shared_ptr<EuclideanExpressionNode> rhs);
       virtual ~EuclideanExpressionNodeElementwiseMultiplyEuclidean();

     private:
       virtual Eigen::Vector3d toEuclideanImplementation() const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
       virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
       virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

       boost::shared_ptr<EuclideanExpressionNode> _lhs;
       boost::shared_ptr<EuclideanExpressionNode> _rhs;
     };

  
  
  } // namespace backend
} // namespace aslam

#endif /* ASLAM_BACKEND_EUCLIDEAN_EXPRESSION_NODE_HPP */

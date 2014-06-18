#ifndef ASLAM_BACKEND_DV_MATRIX_HPP
#define ASLAM_BACKEND_DV_MATRIX_HPP


#include <aslam/backend/JacobianContainer.hpp>
#include <boost/shared_ptr.hpp>
#include <set>

namespace aslam {
  namespace backend {
    
    /**
     * \class MatrixExpressionNode
     * \brief The superclass of all classes representing transformations.
     */
    class MatrixExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      MatrixExpressionNode();
      virtual ~MatrixExpressionNode();

      /// \brief Evaluate the transformation matrix.
      Eigen::Matrix3d toFullMatrix();
      
      /// \brief Evaluate the Jacobians
      void evaluateJacobians(JacobianContainer & outJacobians) const;   
    
      /// \brief Evaluate the Jacobians and apply the chain rule.
      void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      void getDesignVariables(DesignVariable::set_t & designVariables) const;
    protected:        
      // These functions must be implemented by child classes.
      virtual Eigen::Matrix3d toFullMatrixImplementation() = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const = 0;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const = 0;

    };


    /**
     * \class MatrixExpressionNodeMultiply
     *
     * \brief A class representing the multiplication of two transformation matrices.
     * 
     */
    class MatrixExpressionNodeMultiply : public MatrixExpressionNode
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      MatrixExpressionNodeMultiply(boost::shared_ptr<MatrixExpressionNode> lhs,
				     boost::shared_ptr<MatrixExpressionNode> rhs);
      virtual ~MatrixExpressionNodeMultiply();

    private:
      virtual Eigen::Matrix3d toFullMatrixImplementation();
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<MatrixExpressionNode> _lhs;
      Eigen::Matrix3d _A_lhs;
      boost::shared_ptr<MatrixExpressionNode> _rhs;
      Eigen::Matrix3d _A_rhs;
    };


    /**
     * \class MatrixExpressionNodeInverse
     * 
     * \brief A class representing the inverse of a transformation matrix.
     *
     */
    class MatrixExpressionNodeInverse : public MatrixExpressionNode
    {
    public:
    	MatrixExpressionNodeInverse(boost::shared_ptr<MatrixExpressionNode> dvTrafo);

      virtual ~MatrixExpressionNodeInverse();

    private:
      virtual Eigen::Matrix3d toFullMatrixImplementation();
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<MatrixExpressionNode> _dvTrafo;
      Eigen::Matrix3d _A;
    };


  } // namespace backend
} // namespace aslam



#endif /* ASLAM_BACKEND_DV_MATRIX_HPP */

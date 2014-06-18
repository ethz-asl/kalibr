#include <aslam/backend/MatrixExpression.hpp>
#include <aslam/backend/MatrixExpressionNode.hpp>
#include <aslam/backend/EuclideanExpressionNode.hpp>
#include <sm/boost/null_deleter.hpp>

namespace aslam {
  namespace backend {

    

    /// \brief the base case is to initialize an expression from a design variable.
    MatrixExpression::MatrixExpression(MatrixExpressionNode * rotationDesignVariable)
    {
      _root.reset(rotationDesignVariable,sm::null_deleter());
      SM_ASSERT_TRUE(Exception, _root.get() != NULL, "It is illegal to initialized a rotation expression with a null node");
    }

    MatrixExpression::MatrixExpression()
    {
      // 0
    }
    

    MatrixExpression::~MatrixExpression()
    {
      // 0
    }

    MatrixExpression::MatrixExpression(boost::shared_ptr<MatrixExpressionNode> node) :
      _root(node)
    {
      SM_ASSERT_TRUE(Exception, _root.get() != NULL, "It is illegal to initialized a matrix transformation expression with a null node");
    }

    /// \brief Evaluate the full transformation matrix.
    Eigen::Matrix3d MatrixExpression::toFullMatrix()
    {
      return _root->toFullMatrix();
    }

    
    /// \brief return the expression that inverts the rotation.
    MatrixExpression MatrixExpression::inverse()
    {
      boost::shared_ptr<MatrixExpressionNode> newRoot( new MatrixExpressionNodeInverse(_root) );
      return MatrixExpression(newRoot);
    }

    
    /// \brief Evaluate the Jacobians
    void MatrixExpression::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      _root->evaluateJacobians(outJacobians);
    }

    
  MatrixExpression MatrixExpression::operator*(const MatrixExpression & /* p */)
    {
    	// \todo
    	        SM_THROW(aslam::NotImplementedException, "Not implemented yet")
    	      return MatrixExpression();
    }

    EuclideanExpression MatrixExpression::operator*(const EuclideanExpression & p)
    {
      boost::shared_ptr<EuclideanExpressionNode> newRoot( new EuclideanExpressionNodeMatrixMultiply(_root, p._root));	// ##
      return EuclideanExpression(newRoot);
      
    }

  HomogeneousExpression MatrixExpression::operator*(const HomogeneousExpression & /* p */)
    {
      // \todo
        SM_THROW(aslam::NotImplementedException, "Not implemented yet")
      return HomogeneousExpression();
    }

    
    TransformationExpression MatrixExpression::toTransformationExpression()
    {
      // \todo
        SM_THROW(aslam::NotImplementedException, "Not implemented yet")
      return TransformationExpression();
    }

    void MatrixExpression::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      return _root->getDesignVariables(designVariables);
    }
    
    

  } // namespace backend
} // namespace aslam

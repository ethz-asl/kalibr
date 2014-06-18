#include <aslam/backend/TransformationExpression.hpp>
#include <sm/boost/null_deleter.hpp>
#include <aslam/backend/TransformationExpressionNode.hpp>
#include <aslam/backend/HomogeneousExpressionNode.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/backend/EuclideanExpressionNode.hpp>

namespace aslam {
  namespace backend {
    TransformationExpression::TransformationExpression()
    {
      
    }
    
    TransformationExpression::TransformationExpression(TransformationExpressionNode * root) :
      _root(root, sm::null_deleter())
    {

    }
    TransformationExpression::TransformationExpression(boost::shared_ptr<TransformationExpressionNode> root) :
      _root(root)
    {

    }

      TransformationExpression::TransformationExpression(const Eigen::Matrix4d & T)
      {
          _root.reset(new TransformationExpressionNodeConstant(T));
      }

    TransformationExpression::TransformationExpression(const RotationExpression & rotation, const EuclideanExpression & translation)
    {
      _root.reset(new TransformationBasic(rotation, translation) );
    }


    TransformationExpression::~TransformationExpression()
    {
      
    }

    void TransformationExpression::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      _root->getDesignVariables(designVariables);
    }


    Eigen::Matrix4d TransformationExpression::toTransformationMatrix() const
    {
      return _root->toTransformationMatrix();
    }
  
  RotationExpression TransformationExpression::toRotationExpression() const {
    boost::shared_ptr< RotationExpressionNode > een( new RotationExpressionNodeTransformation( _root ) );
    return RotationExpression(een);
  }
  // HomogeneousExpression toHomogeneousExpression() const;
  EuclideanExpression TransformationExpression::toEuclideanExpression() const {
    boost::shared_ptr< EuclideanExpressionNode > een( new EuclideanExpressionNodeTranslation( _root ) );
    return EuclideanExpression(een);
  }


    void TransformationExpression::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      return _root->evaluateJacobians(outJacobians);
    }


    void TransformationExpression::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      return _root->evaluateJacobians(outJacobians, applyChainRule);
    }


    // EuclideanExpression operator*(const EuclideanExpression & rhs) const;

    HomogeneousExpression TransformationExpression::operator*(const HomogeneousExpression & rhs) const
    {
      boost::shared_ptr<HomogeneousExpressionNode> newRoot( new HomogeneousExpressionNodeMultiply(_root, rhs._root));
      return HomogeneousExpression(newRoot);
    }
    
    TransformationExpression TransformationExpression::operator*(const TransformationExpression & rhs) const
    {
      boost::shared_ptr<TransformationExpressionNode> newRoot( new TransformationExpressionNodeMultiply(_root, rhs._root));
      return TransformationExpression(newRoot);
    }

    TransformationExpression TransformationExpression::inverse() const
    {
      boost::shared_ptr<TransformationExpressionNode> newRoot( new TransformationExpressionNodeInverse(_root));
      return TransformationExpression(newRoot);
    }
  

  } // namespace backend
} // namespace aslam

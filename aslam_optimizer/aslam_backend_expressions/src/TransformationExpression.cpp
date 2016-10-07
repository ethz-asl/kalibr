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
      _root.reset((rotation.isEmpty() && translation.isEmpty()) ? nullptr : new TransformationBasic(rotation, translation) );
    }


    TransformationExpression::~TransformationExpression()
    {
    }

    void TransformationExpression::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      if(_root)
        _root->getDesignVariables(designVariables);
    }


    Eigen::Matrix4d TransformationExpression::toTransformationMatrix() const
    {
      if(_root){
        return _root->toTransformationMatrix();
      } else {
        return Eigen::Matrix4d::Identity();
      }
    }
  
  RotationExpression TransformationExpression::toRotationExpression() const {
    if(_root){
      return _root->toRotationExpression(_root);
    } else {
      return RotationExpression();
    }
  }
  // HomogeneousExpression toHomogeneousExpression() const;
  EuclideanExpression TransformationExpression::toEuclideanExpression() const {
    if(_root){
      return _root->toEuclideanExpression(_root);
    } else {
      return EuclideanExpression();
    }
  }


    void TransformationExpression::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      if(_root)
        _root->evaluateJacobians(outJacobians);
    }

    EuclideanExpression TransformationExpression::operator*(const EuclideanExpression & rhs) const{
      if(!_root) return rhs;
      return (*this * rhs.toHomogeneousExpression()).toEuclideanExpression();
    }
    HomogeneousExpression TransformationExpression::operator*(const HomogeneousExpression & rhs) const
    {
      if(!_root) return rhs;
      boost::shared_ptr<HomogeneousExpressionNode> newRoot( new HomogeneousExpressionNodeMultiply(_root, rhs._root));
      return HomogeneousExpression(newRoot);
    }
    
    TransformationExpression TransformationExpression::operator*(const TransformationExpression & rhs) const
    {
      if(!_root) return rhs;
      if(!rhs._root) return *this;
      boost::shared_ptr<TransformationExpressionNode> newRoot( new TransformationExpressionNodeMultiply(_root, rhs._root));
      return TransformationExpression(newRoot);
    }

    TransformationExpression TransformationExpression::inverse() const
    {
      if(!_root) return *this;
      boost::shared_ptr<TransformationExpressionNode> newRoot( new TransformationExpressionNodeInverse(_root));
      return TransformationExpression(newRoot);
    }

    sm::kinematics::Transformation TransformationExpression::toTransformation() const {
      return toTransformationMatrix();
    }

  } // namespace backend
}  // namespace aslam


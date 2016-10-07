#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/EuclideanExpressionNode.hpp>
#include <sm/boost/null_deleter.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/HomogeneousExpressionNode.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/VectorExpressionNode.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/ScalarExpressionNode.hpp>

namespace aslam {
  namespace backend {
    HomogeneousExpression EuclideanExpression::toHomogeneousExpression() const
    {
      assert(!isEmpty());
      boost::shared_ptr<HomogeneousExpressionNode> newRoot( new HomogeneousExpressionNodeEuclidean(_root) );
      return HomogeneousExpression(newRoot);
    }
    
    void EuclideanExpression::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      if(!isEmpty())
        _root->evaluateJacobians(outJacobians);
    }

    void EuclideanExpression::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      if(!isEmpty())
        _root->evaluateJacobians(outJacobians, applyChainRule);
    }

    EuclideanExpression EuclideanExpression::cross(const EuclideanExpression & p) const
    {
      if(p.isEmpty() || this->isEmpty())
        return EuclideanExpression();
      boost::shared_ptr<EuclideanExpressionNode> newRoot( new EuclideanExpressionNodeCrossEuclidean(_root, p._root));
      return EuclideanExpression(newRoot);
    }

    EuclideanExpression EuclideanExpression::elementwiseMultiply(const EuclideanExpression & p) const
    {
      boost::shared_ptr<EuclideanExpressionNode> newRoot( new EuclideanExpressionNodeElementwiseMultiplyEuclidean(_root, p._root));
      return EuclideanExpression(newRoot);
    }

    EuclideanExpression EuclideanExpression::operator+(const EuclideanExpression & p) const
    {
      if(p.isEmpty())
        return *this;
      if(this->isEmpty())
        return p;
      boost::shared_ptr<EuclideanExpressionNode> newRoot( new EuclideanExpressionNodeAddEuclidean(_root, p._root));
      return EuclideanExpression(newRoot);
    }

    EuclideanExpression EuclideanExpression::operator-(const EuclideanExpression & p) const
    {
      if(p.isEmpty())
        return *this;
      if(this->isEmpty())
        return p;
      boost::shared_ptr<EuclideanExpressionNode> newRoot( new EuclideanExpressionNodeSubtractEuclidean(_root, p._root));
      return EuclideanExpression(newRoot);
    }

    EuclideanExpression EuclideanExpression::operator-(const Eigen::Vector3d & p) const
    {
      if(this->isEmpty())
        return p;
      boost::shared_ptr<EuclideanExpressionNode> newRoot( new EuclideanExpressionNodeSubtractVector(_root, p));
      return EuclideanExpression(newRoot);
    }

    EuclideanExpression EuclideanExpression::operator-() const
    {
      if(this->isEmpty())
        return *this;
      boost::shared_ptr<EuclideanExpressionNode> newRoot( new EuclideanExpressionNodeNegated(_root));
      return EuclideanExpression(newRoot);
    }

    EuclideanExpression EuclideanExpression::operator*(const ScalarExpression& s) const
    {
      if(this->isEmpty() || s.isEmpty())
        return EuclideanExpression();
      boost::shared_ptr<EuclideanExpressionNode> newRoot(new EuclideanExpressionNodeScalarMultiply(_root, s._root));
      return EuclideanExpression(newRoot);
    }

  
  } // namespace backend
} // namespace aslam

#include <boost/make_shared.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/RotationExpressionNode.hpp>
#include <aslam/backend/EuclideanExpressionNode.hpp>
#include <sm/boost/null_deleter.hpp>

namespace aslam {
  namespace backend {


    /// \brief the base case is to initialize an expression from a design variable.
    RotationExpression::RotationExpression(RotationExpressionNode * rotationDesignVariable) 
    {
      _root.reset(rotationDesignVariable,sm::null_deleter());
      SM_ASSERT_TRUE(Exception, _root.get() != NULL, "It is illegal to initialize a rotation expression with a null pointer");
    }

    RotationExpression::~RotationExpression()
    {
    }

    RotationExpression::RotationExpression(boost::shared_ptr<RotationExpressionNode> node) :
      _root(node)
    {
    }

    RotationExpression::RotationExpression(const Eigen::Matrix3d & C) :
      _root(boost::make_shared<ConstantRotationExpressionNode>(C))
    {
    }

    /// \brief Evaluate the rotation matrix.
    Eigen::Matrix3d RotationExpression::toRotationMatrix() const
    {
      if(isEmpty()) return Eigen::Matrix3d::Identity();
      return _root->toRotationMatrix();
    }

    /// \brief return the expression that inverts the rotation.
    RotationExpression RotationExpression::inverse() const
    {
      if(isEmpty()) return *this;
      boost::shared_ptr<RotationExpressionNode> newRoot( new RotationExpressionNodeInverse(_root) );
      return RotationExpression(newRoot);
    }

    /// \brief Evaluate the Jacobians
    void RotationExpression::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      if(!isEmpty())
        _root->evaluateJacobians(outJacobians);
    }

    RotationExpression RotationExpression::operator*(const RotationExpression & p) const
    {
      if(p.isEmpty())
        return *this;
      if(this->isEmpty())
        return p;
      boost::shared_ptr<RotationExpressionNode> newRoot( new RotationExpressionNodeMultiply(_root, p._root));
      return RotationExpression(newRoot);
    }

    EuclideanExpression RotationExpression::operator*(const EuclideanExpression & p) const
    {
      if(p.isEmpty())
        return EuclideanExpression();
      if(this->isEmpty())
        return EuclideanExpression();
      boost::shared_ptr<EuclideanExpressionNode> newRoot( new EuclideanExpressionNodeMultiply(_root, p._root));
      return EuclideanExpression(newRoot);
      
    }

    HomogeneousExpression RotationExpression::operator*(const HomogeneousExpression & /* p */) const
    {
      // \todo
        SM_THROW(Exception, "Not implemented yet")
      return HomogeneousExpression();
    }

    TransformationExpression RotationExpression::toTransformationExpression() const
    {
      // \todo
        SM_THROW(Exception, "Not implemented yet")
      return TransformationExpression();
    }

    void RotationExpression::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      if(!isEmpty())
        _root->getDesignVariables(designVariables);
    }
    
    EuclideanExpression RotationExpression::toParameters(sm::kinematics::RotationalKinematics::Ptr rk) const {
      assert(!isEmpty());
      boost::shared_ptr<EuclideanExpressionNode> een( new EuclideanExpressionNodeRotationParameters(_root, rk));
      return EuclideanExpression(een);
    }

  } // namespace backend
} // namespace aslam

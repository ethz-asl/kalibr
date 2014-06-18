#include <aslam/backend/HomogeneousExpression.hpp>
#include <sm/boost/null_deleter.hpp>
#include <aslam/backend/HomogeneousExpressionNode.hpp>
#include <aslam/backend/EuclideanExpressionNode.hpp>

namespace aslam {
  namespace backend {
    HomogeneousExpression::HomogeneousExpression()
    {
      
    }
    HomogeneousExpression::~HomogeneousExpression()
    {
     
    }
    HomogeneousExpression::HomogeneousExpression(HomogeneousExpressionNode * designVariable) :
      _root(designVariable, sm::null_deleter())
    {

    }
    
    HomogeneousExpression::HomogeneousExpression(boost::shared_ptr<HomogeneousExpressionNode> designVariable) :
      _root(designVariable)
    {

    }

      HomogeneousExpression::HomogeneousExpression(const Eigen::Vector4d & p)
      {
          _root.reset(new HomogeneousExpressionNodeConstant(p));
      }

      HomogeneousExpression::HomogeneousExpression(const Eigen::Vector3d & p)
      {
          Eigen::Vector4d ph(p[0],p[1],p[2],1.0);
          _root.reset(new HomogeneousExpressionNodeConstant(ph));
      }


    Eigen::Vector4d HomogeneousExpression::toHomogeneous() const
    {
      return _root->toHomogeneous();
    }

    EuclideanExpression HomogeneousExpression::toEuclideanExpression() const
     {
       boost::shared_ptr<EuclideanExpressionNode> newRoot( new EuclideanExpressionNodeFromHomogeneous(_root));
       return EuclideanExpression(newRoot);
                                                           
    }

    void HomogeneousExpression::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      return _root->evaluateJacobians(outJacobians);
    }

    void HomogeneousExpression::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      return _root->evaluateJacobians(outJacobians, applyChainRule);
    }

    void HomogeneousExpression::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      _root->getDesignVariables(designVariables);
    }


  
  } // namespace backend
} // namespace aslam

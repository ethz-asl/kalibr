#include <aslam/backend/RotationExpressionNode.hpp>

namespace aslam {
  namespace backend {
    
    ////////////////////////////////////////////
    // RotationExpressionNode: The Super Class
    ////////////////////////////////////////////
    RotationExpressionNode::RotationExpressionNode(){}

    RotationExpressionNode::~RotationExpressionNode(){}

    Eigen::Matrix3d RotationExpressionNode::toRotationMatrix() const { return toRotationMatrixImplementation(); }

    void RotationExpressionNode::evaluateJacobians(JacobianContainer & outJacobians) const{
      evaluateJacobiansImplementation(outJacobians);
    }      

    void RotationExpressionNode::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const{
      evaluateJacobiansImplementation(outJacobians, applyChainRule);
    }
      
    void RotationExpressionNode::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      getDesignVariablesImplementation(designVariables);
    }

    /////////////////////////////////////////////////
    // RotationExpressionNodeMultiply: A container for C1 * C2
    /////////////////////////////////////////////////

    RotationExpressionNodeMultiply::RotationExpressionNodeMultiply(boost::shared_ptr<RotationExpressionNode> lhs, boost::shared_ptr<RotationExpressionNode> rhs):
      _lhs(lhs), _rhs(rhs)
    {
      _C_lhs = _lhs->toRotationMatrix();
      _C_rhs = _rhs->toRotationMatrix();
    }

    RotationExpressionNodeMultiply::~RotationExpressionNodeMultiply(){}

    Eigen::Matrix3d RotationExpressionNodeMultiply::toRotationMatrixImplementation() const
    {
      _C_lhs = _lhs->toRotationMatrix();
	_C_rhs = _rhs->toRotationMatrix();
	return  _C_lhs * _C_rhs;
      }

      void RotationExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
      {
	_rhs->evaluateJacobians(outJacobians,_C_lhs);
	_lhs->evaluateJacobians(outJacobians);
      }

      void RotationExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
      {	
	_rhs->evaluateJacobians(outJacobians, applyChainRule * _C_lhs);
	_lhs->evaluateJacobians(outJacobians, applyChainRule);
      }

    void RotationExpressionNodeMultiply::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }


    ////////////////////////////////////////////////////
    // RotationExpressionNodeInverse: A container for C^T
    ////////////////////////////////////////////////////
    
    RotationExpressionNodeInverse::RotationExpressionNodeInverse(boost::shared_ptr<RotationExpressionNode> dvRotation) : _dvRotation(dvRotation)
      {
	_C = _dvRotation->toRotationMatrix();
      }
    
    RotationExpressionNodeInverse::~RotationExpressionNodeInverse(){}

    Eigen::Matrix3d RotationExpressionNodeInverse::toRotationMatrixImplementation() const
    {
      _C = _dvRotation->toRotationMatrix();
      return  _C.transpose();
    }

    void RotationExpressionNodeInverse::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _dvRotation->evaluateJacobians(outJacobians, -_C.transpose());
    }

    void RotationExpressionNodeInverse::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _dvRotation->evaluateJacobians(outJacobians, -applyChainRule * _C.transpose());
    }

    void RotationExpressionNodeInverse::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _dvRotation->getDesignVariables(designVariables);
    }


  RotationExpressionNodeTransformation::RotationExpressionNodeTransformation(boost::shared_ptr<TransformationExpressionNode> transformation) :
      _transformation(transformation) {
    
  }


  
  RotationExpressionNodeTransformation::~RotationExpressionNodeTransformation() {

  }

  Eigen::Matrix3d RotationExpressionNodeTransformation::toRotationMatrixImplementation() const {
    return _transformation->toTransformationMatrix().topLeftCorner<3,3>();
  }

  void RotationExpressionNodeTransformation::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3,6);
    J.topRightCorner<3,3>() = Eigen::Matrix3d::Identity();
    _transformation->evaluateJacobians(outJacobians, J);
  }

  void RotationExpressionNodeTransformation::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3,6);
    J.topRightCorner<3,3>() = Eigen::Matrix3d::Identity();
    _transformation->evaluateJacobians(outJacobians, applyChainRule*J);

  }

  void RotationExpressionNodeTransformation::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
    _transformation->getDesignVariables(designVariables);
  }


  
  } // namespace backend
} // namespace aslam

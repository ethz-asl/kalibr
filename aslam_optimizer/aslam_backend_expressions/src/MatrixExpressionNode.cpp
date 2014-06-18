#include <aslam/backend/MatrixExpressionNode.hpp>

namespace aslam {
  namespace backend {
    
    ////////////////////////////////////////////
    // RotationExpressionNode: The Super Class
    ////////////////////////////////////////////
    MatrixExpressionNode::MatrixExpressionNode(){}

    MatrixExpressionNode::~MatrixExpressionNode(){}

    Eigen::Matrix3d MatrixExpressionNode::toFullMatrix(){ return toFullMatrixImplementation(); }

    void MatrixExpressionNode::evaluateJacobians(JacobianContainer & outJacobians) const{
      evaluateJacobiansImplementation(outJacobians);
    }      

    void MatrixExpressionNode::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const{
      evaluateJacobiansImplementation(outJacobians, applyChainRule);
    }
      
    void MatrixExpressionNode::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      getDesignVariablesImplementation(designVariables);
    }


    /////////////////////////////////////////////////
    // MatrixExpressionNodeMultiply: A container for A1 * A2
    /////////////////////////////////////////////////

    MatrixExpressionNodeMultiply::MatrixExpressionNodeMultiply(boost::shared_ptr<MatrixExpressionNode> lhs, boost::shared_ptr<MatrixExpressionNode> rhs):
      _lhs(lhs), _rhs(rhs)
    {
      _A_lhs = _lhs->toFullMatrix();
      _A_rhs = _rhs->toFullMatrix();
    }

    MatrixExpressionNodeMultiply::~MatrixExpressionNodeMultiply(){}

    Eigen::Matrix3d MatrixExpressionNodeMultiply::toFullMatrixImplementation()
    {
      _A_lhs = _lhs->toFullMatrix();
	  _A_rhs = _rhs->toFullMatrix();
	return  _A_lhs * _A_rhs;
      }

      void MatrixExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
      {	
	    _rhs->evaluateJacobians(outJacobians,_A_lhs);
	    _lhs->evaluateJacobians(outJacobians);
      }

      void MatrixExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
      {	
	    _rhs->evaluateJacobians(outJacobians, applyChainRule * _A_lhs);  		//##ck: adapt this
	    _lhs->evaluateJacobians(outJacobians, applyChainRule);
      }

    void MatrixExpressionNodeMultiply::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }


    ////////////////////////////////////////////////////
    // MatrixExpressionNodeInverse: A container for A^-1
    ////////////////////////////////////////////////////
    
    MatrixExpressionNodeInverse::MatrixExpressionNodeInverse(boost::shared_ptr<MatrixExpressionNode> dvTrafo) : _dvTrafo(dvTrafo)
      {
	_A = _dvTrafo->toFullMatrix();
      }
    
    MatrixExpressionNodeInverse::~MatrixExpressionNodeInverse(){}

    Eigen::Matrix3d MatrixExpressionNodeInverse::toFullMatrixImplementation()
    {
      _A = _dvTrafo->toFullMatrix();
      return  _A.transpose();			//## ToDo: Replace Transpose with inverse
    }

    void MatrixExpressionNodeInverse::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
    	_dvTrafo->evaluateJacobians(outJacobians, -_A.transpose());  //## ToDo: Replace Transpose with inverse
    }

    void MatrixExpressionNodeInverse::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
    	_dvTrafo->evaluateJacobians(outJacobians, -applyChainRule * _A.transpose());  //## ToDo: Replace Transpose with inverse
    }

    void MatrixExpressionNodeInverse::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
    	_dvTrafo->getDesignVariables(designVariables);
    }


  } // namespace backend
} // namespace aslam

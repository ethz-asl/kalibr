#include <aslam/backend/ScalarExpressionNode.hpp>
#include <sm/kinematics/rotations.hpp>

namespace aslam {
    namespace backend {
    
        ScalarExpressionNode::ScalarExpressionNode()
        {

        }

        ScalarExpressionNode::~ScalarExpressionNode()
        {
        
        }


        /// \brief Evaluate the scalar matrix.
        double ScalarExpressionNode::toScalar() const
        {
            return toScalarImplementation();
        }

      
        /// \brief Evaluate the Jacobians
        void ScalarExpressionNode::evaluateJacobians(JacobianContainer & outJacobians) const
        {
            evaluateJacobiansImplementation(outJacobians);
        }
   
    
        /// \brief Evaluate the Jacobians and apply the chain rule.
        void ScalarExpressionNode::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            SM_ASSERT_EQ_DBG(Exception, applyChainRule.cols(), 1, "The chain rule matrix must have one columns");
            evaluateJacobiansImplementation(outJacobians, applyChainRule);
        }

        void ScalarExpressionNode::getDesignVariables(DesignVariable::set_t & designVariables) const
        {
            getDesignVariablesImplementation(designVariables);
        }


          
        ScalarExpressionNodeMultiply::ScalarExpressionNodeMultiply(boost::shared_ptr<ScalarExpressionNode> lhs,
                                                                   boost::shared_ptr<ScalarExpressionNode> rhs) :
            _lhs(lhs), _rhs(rhs)
        {

        }

        ScalarExpressionNodeMultiply::~ScalarExpressionNodeMultiply()
        {

        }
        double ScalarExpressionNodeMultiply::toScalarImplementation() const
        {
            return _lhs->toScalar() * _rhs->toScalar();
        }

        void ScalarExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd L(1,1), R(1,1);
            L(0,0) = _lhs->toScalar();
            R(0,0) = _rhs->toScalar();
            _lhs->evaluateJacobians(outJacobians, R);
            _rhs->evaluateJacobians(outJacobians, L);
        }

        void ScalarExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            Eigen::MatrixXd L(1,1), R(1,1);
            L(0,0) = _lhs->toScalar();
            R(0,0) = _rhs->toScalar();
            _lhs->evaluateJacobians(outJacobians, applyChainRule * R);
            _rhs->evaluateJacobians(outJacobians, applyChainRule * L);
        }

        void ScalarExpressionNodeMultiply::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
            _rhs->getDesignVariables(designVariables);
        }

        ScalarExpressionNodeDivide::ScalarExpressionNodeDivide(boost::shared_ptr<ScalarExpressionNode> lhs,
                                                                   boost::shared_ptr<ScalarExpressionNode> rhs) :
            _lhs(lhs), _rhs(rhs)
        {

        }

        ScalarExpressionNodeDivide::~ScalarExpressionNodeDivide()
        {

        }
        double ScalarExpressionNodeDivide::toScalarImplementation() const
        {
            return _lhs->toScalar() / _rhs->toScalar();
        }

        void ScalarExpressionNodeDivide::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd L(1,1), R(1,1);
            L(0,0) = 1 / _rhs->toScalar();
            R(0,0) = -_lhs->toScalar() / (_rhs->toScalar()*_rhs->toScalar());
            _lhs->evaluateJacobians(outJacobians, L);
            _rhs->evaluateJacobians(outJacobians, R);
        }

        void ScalarExpressionNodeDivide::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            Eigen::MatrixXd L(1,1), R(1,1);
            L(0,0) = 1 /_rhs->toScalar();
            R(0,0) = -_lhs->toScalar() / (_rhs->toScalar()*_rhs->toScalar());
            _lhs->evaluateJacobians(outJacobians, applyChainRule * L);
            _rhs->evaluateJacobians(outJacobians, applyChainRule * R);
        }

        void ScalarExpressionNodeDivide::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
            _rhs->getDesignVariables(designVariables);
        }


        ScalarExpressionNodeAdd::ScalarExpressionNodeAdd(boost::shared_ptr<ScalarExpressionNode> lhs,
                                                         boost::shared_ptr<ScalarExpressionNode> rhs,
                                                         double multiplyRhs) :
            _lhs(lhs), _rhs(rhs), _multiplyRhs(multiplyRhs)
        {
            
        }

        ScalarExpressionNodeAdd::~ScalarExpressionNodeAdd()
        {

        }

        double ScalarExpressionNodeAdd::toScalarImplementation() const
        {
            return _lhs->toScalar() + _multiplyRhs * _rhs->toScalar();
        }

        void ScalarExpressionNodeAdd::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd R(1,1);
            R(0,0) = _multiplyRhs;
            _lhs->evaluateJacobians(outJacobians);
            _rhs->evaluateJacobians(outJacobians, R);
        }

        void ScalarExpressionNodeAdd::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            _lhs->evaluateJacobians(outJacobians, applyChainRule);
            _rhs->evaluateJacobians(outJacobians, applyChainRule * _multiplyRhs);
        }

        void ScalarExpressionNodeAdd::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
            _rhs->getDesignVariables(designVariables);
        }

          
        ScalarExpressionNodeConstant::ScalarExpressionNodeConstant(double s) : _s(s)
        {
            
        }

        ScalarExpressionNodeConstant::~ScalarExpressionNodeConstant()
        {

        }

        ScalarExpressionNodeFromVectorExpression::ScalarExpressionNodeFromVectorExpression(boost::shared_ptr<VectorExpressionNode<1> > lhs) :
            _lhs(lhs)
        {

        }

        ScalarExpressionNodeFromVectorExpression::~ScalarExpressionNodeFromVectorExpression()
        {

        }

        double ScalarExpressionNodeFromVectorExpression::toScalarImplementation() const
        {
            return _lhs->evaluate()(0);
        }

        void ScalarExpressionNodeFromVectorExpression::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
        {
            _lhs->evaluateJacobians(outJacobians);
        }

        void ScalarExpressionNodeFromVectorExpression::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            _lhs->evaluateJacobians(outJacobians, applyChainRule);
        }

        void ScalarExpressionNodeFromVectorExpression::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
        }

          




    } // namespace backend
} // namespace aslam

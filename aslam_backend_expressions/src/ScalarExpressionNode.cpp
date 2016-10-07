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

        void ScalarExpressionNodeAdd::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
            _rhs->getDesignVariables(designVariables);
        }


        ScalarExpressionNodeSqrt::ScalarExpressionNodeSqrt(boost::shared_ptr<ScalarExpressionNode> lhs) :
            _lhs(lhs)
        {

        }

        ScalarExpressionNodeSqrt::~ScalarExpressionNodeSqrt()
        {

        }

        void ScalarExpressionNodeSqrt::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
        }

        ScalarExpressionNodeLog::ScalarExpressionNodeLog(boost::shared_ptr<ScalarExpressionNode> lhs) :
            _lhs(lhs)
        {

        }

        ScalarExpressionNodeLog::~ScalarExpressionNodeLog()
        {

        }

        void ScalarExpressionNodeLog::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
        }


        ScalarExpressionNodeExp::ScalarExpressionNodeExp(boost::shared_ptr<ScalarExpressionNode> lhs) :
            _lhs(lhs)
        {

        }

        ScalarExpressionNodeExp::~ScalarExpressionNodeExp()
        {

        }

        void ScalarExpressionNodeExp::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
        }


        ScalarExpressionNodeAtan::ScalarExpressionNodeAtan(boost::shared_ptr<ScalarExpressionNode> lhs) :
            _lhs(lhs)
        {

        }

        ScalarExpressionNodeAtan::~ScalarExpressionNodeAtan()
        {

        }

        void ScalarExpressionNodeAtan::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
        }

        ScalarExpressionNodeTanh::ScalarExpressionNodeTanh(boost::shared_ptr<ScalarExpressionNode> lhs) :
            _lhs(lhs)
        {

        }

        ScalarExpressionNodeTanh::~ScalarExpressionNodeTanh()
        {

        }

        void ScalarExpressionNodeTanh::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
        }

        ScalarExpressionNodeAtan2::ScalarExpressionNodeAtan2(boost::shared_ptr<ScalarExpressionNode> lhs, boost::shared_ptr<ScalarExpressionNode> rhs) :
            _lhs(lhs),
            _rhs(rhs)
        {

        }

        ScalarExpressionNodeAtan2::~ScalarExpressionNodeAtan2()
        {

        }

        void ScalarExpressionNodeAtan2::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
            _rhs->getDesignVariables(designVariables);
        }

        ScalarExpressionNodeAcos::ScalarExpressionNodeAcos(boost::shared_ptr<ScalarExpressionNode> lhs) :
            _lhs(lhs)
        {

        }

        ScalarExpressionNodeAcos::~ScalarExpressionNodeAcos()
        {

        }

        void ScalarExpressionNodeAcos::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _lhs->getDesignVariables(designVariables);
        }


        ScalarExpressionNodeAcosSquared::ScalarExpressionNodeAcosSquared(boost::shared_ptr<ScalarExpressionNode> lhs) :
            _lhs(lhs)
        {

        }

        ScalarExpressionNodeAcosSquared::~ScalarExpressionNodeAcosSquared()
        {

        }

        void ScalarExpressionNodeAcosSquared::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
          _lhs->getDesignVariables(designVariables);
        }

        ScalarExpressionNodeInverseSigmoid::ScalarExpressionNodeInverseSigmoid(boost::shared_ptr<ScalarExpressionNode> lhs,
                                                                 const double height,
                                                                 const double scale,
                                                                 const double shift)
          : _lhs(lhs), _height(height), _scale(scale), _shift(shift)
        {

        }

        ScalarExpressionNodeInverseSigmoid::~ScalarExpressionNodeInverseSigmoid()
        {

        }

        void ScalarExpressionNodeInverseSigmoid::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
          _lhs->getDesignVariables(designVariables);
        }


        ScalarExpressionNodePower::ScalarExpressionNodePower(boost::shared_ptr<ScalarExpressionNode> lhs, const int k)
          :_lhs(lhs), _power(k)
        {

        }

        ScalarExpressionNodePower::~ScalarExpressionNodePower()
        {

        }

        void ScalarExpressionNodePower::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
          _lhs->getDesignVariables(designVariables);
        }


        ScalarExpressionPiecewiseExpression::ScalarExpressionPiecewiseExpression(boost::shared_ptr<ScalarExpressionNode> e1, boost::shared_ptr<ScalarExpressionNode> e2, std::function<bool()> useFirst)
          : _e1(e1), _e2(e2), _useFirst(useFirst)
        {

        }

        ScalarExpressionPiecewiseExpression::~ScalarExpressionPiecewiseExpression()
        {

        }

        void ScalarExpressionPiecewiseExpression::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
          if (_useFirst()) {
            return _e1->getDesignVariables(designVariables);
          } else {
            return _e2->getDesignVariables(designVariables);
          }
        }

        ScalarExpressionNodeConstant::ScalarExpressionNodeConstant(double s) : _s(s)
        {
        }

        ScalarExpressionNodeConstant::~ScalarExpressionNodeConstant()
        {
        }

        ScalarExpressionNodeNegated::ScalarExpressionNodeNegated(boost::shared_ptr<ScalarExpressionNode> rhs) :
            _rhs(rhs)
        {
        }

        ScalarExpressionNodeNegated::~ScalarExpressionNodeNegated()
        {
        }

        void ScalarExpressionNodeNegated::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            _rhs->getDesignVariables(designVariables);
        }
    } // namespace backend
} // namespace aslam

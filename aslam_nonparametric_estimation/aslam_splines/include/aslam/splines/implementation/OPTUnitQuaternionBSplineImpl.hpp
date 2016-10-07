#ifndef OPTUNITQUATERNIONBSPLINEIMPL_HPP_
#define OPTUNITQUATERNIONBSPLINEIMPL_HPP_

#include "aslam/backend/JacobianContainer.hpp"

namespace bsplines {

#define _TEMPLATE template <typename TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy, typename TModifiedDerivedConf>
#define _CLASS DiffManifoldBSpline<aslam::splines::DesignVariableSegmentBSplineConf<UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TModifiedDerivedConf>, aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> >

_TEMPLATE
template <typename FactoryData_>
typename _CLASS::angular_derivative_expression_t _CLASS::ExpressionFactory<FactoryData_>::getAngularVelocityExpression() const {
	typedef aslam::backend::VectorExpressionNode<3> node_t;

	class ExpressionNode : public node_t {
	public:
		ExpressionNode(const DataSharedPtr & dataPtr) : _dataPtr(dataPtr){}
	private:
		typedef typename node_t::vector_t vector_t;
		const DataSharedPtr _dataPtr;
		inline void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd * applyChainRule) const {
			const int dimension=_dataPtr->getSpline().getDimension(), pointSize = dimension, splineOrder = _dataPtr->getSpline().getSplineOrder();
			typename _CLASS::angular_jacobian_t J(pointSize, dimension * splineOrder);
			auto & eval = _dataPtr->getEvaluator();
			eval.evalAngularVelocityJacobian(J);

			int col = 0;
			for(SegmentConstIterator i = eval.begin(), end = eval.end(); i != end; ++i)
			{
				internal::AddJac<typename _CLASS::angular_jacobian_t, Dimension, Dimension>::addJac(J, col, &i->getDesignVariable(), outJacobians, applyChainRule, pointSize, dimension);
				col+=dimension;
			}
			if(_dataPtr->hasTimeExpression()){
				auto evalJac = eval.evalAngularAcceleration();
				_dataPtr->getTimeExpression().evaluateJacobians(outJacobians, applyChainRule ? Eigen::MatrixXd(*applyChainRule * evalJac) : Eigen::MatrixXd(evalJac));
			}
		}

	protected:
		virtual vector_t evaluateImplementation() const {
			return _dataPtr->getEvaluator().evalAngularVelocity();
		}

		virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const {
			evaluateJacobiansImplementation(outJacobians, NULL);
		}

		virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
			evaluateJacobiansImplementation(outJacobians, &applyChainRule);
		}

		virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const {
			_dataPtr->getDesignVariables(designVariables);
		}
	};
	return angular_derivative_expression_t(boost::shared_ptr<node_t>(static_cast<node_t *> (new ExpressionNode(this->getDataPtr()))));
}

_TEMPLATE
template <typename FactoryData_>
typename _CLASS::angular_derivative_expression_t _CLASS::ExpressionFactory<FactoryData_>::getAngularAccelerationExpression() const {
	typedef aslam::backend::VectorExpressionNode<3> node_t;

	class ExpressionNode : public node_t {
	private:
		typedef typename node_t::vector_t vector_t;
		_CLASS::ExpressionFactory<FactoryData_>::DataSharedPtr _dataPtr;
	public:
		ExpressionNode(const DataSharedPtr & dataPtr) : _dataPtr(dataPtr){}
		virtual ~ExpressionNode(){};
	private:
		inline void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd * applyChainRule) const {
			const int dimension=_dataPtr->getSpline().getDimension(), pointSize = dimension, splineOrder = _dataPtr->getSpline().getSplineOrder();
			typename _CLASS::angular_jacobian_t J(pointSize, dimension * splineOrder);
			auto & eval = _dataPtr->getEvaluator();

			eval.evalAngularAccelerationJacobian(J);
			int col = 0;
			for(SegmentConstIterator i = eval.begin(), end = eval.end(); i != end; ++i)
			{
				internal::AddJac<typename _CLASS::angular_jacobian_t, Dimension, Dimension>::addJac(J, col, &i->getDesignVariable(), outJacobians, applyChainRule, pointSize, dimension);
				col+=dimension;
			}
			if(_dataPtr->hasTimeExpression()){
				auto evalJac = eval.template evalAngularDerivative<3>();
				_dataPtr->getTimeExpression().evaluateJacobians(outJacobians, applyChainRule ? Eigen::MatrixXd(*applyChainRule * evalJac) : Eigen::MatrixXd(evalJac));
			}
		}

	protected:
		virtual vector_t evaluateImplementation() const {
			return _dataPtr->getEvaluator().evalAngularAcceleration();
		}

		virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const {
			evaluateJacobiansImplementation(outJacobians, NULL);
		}

		virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
			evaluateJacobiansImplementation(outJacobians, &applyChainRule);
		}

		virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const {
			_dataPtr->getDesignVariables(designVariables);
		}
	};
	return angular_derivative_expression_t(boost::shared_ptr<node_t>(static_cast<node_t *> (new ExpressionNode(this->getDataPtr()))));
}

#undef _CLASS
#undef _TEMPLATE

} // namespace bsplines

#endif /* OPTUNITQUATERNIONBSPLINEIMPL_HPP_ */

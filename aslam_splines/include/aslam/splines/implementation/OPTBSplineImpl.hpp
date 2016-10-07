/*
 * OPTBSplineImpl.hpp
 *
 *  Created on: 05.08.2012
 *      Author: hannes
 */

#ifndef OPTBSPLINEIMPL_HPP_
#define OPTBSPLINEIMPL_HPP_

#include <type_traits>
#include "aslam/backend/JacobianContainer.hpp"
#include "boost/typeof/typeof.hpp"
#include "bsplines/DiffManifoldBSpline.hpp"
#include "bsplines/manifolds/LieGroup.hpp"

namespace bsplines {

#define _TEMPLATE template<typename TModifiedConf, typename TModifiedDerivedConf>
#define _CLASS DiffManifoldBSpline<aslam::splines::DesignVariableSegmentBSplineConf<TModifiedConf, TModifiedDerivedConf>, aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> >

_TEMPLATE
void _CLASS::init() {
	parent_t::init();
	updateDesignVariablesVector();
}

_TEMPLATE
void _CLASS::updateDesignVariablesVector() {
	_designVariables.clear();
	for(SegmentIterator i = this->firstRelevantSegment(), end = this->end(); i != end; i++)
	{
		_designVariables.push_back((dv_t* )& i->getDesignVariable());
	}
}

_TEMPLATE
size_t _CLASS::numDesignVariables() const {
	return _designVariables.size();
}

_TEMPLATE
typename _CLASS::dv_t * _CLASS::designVariable(
		size_t i) {
	SM_ASSERT_LT(aslam::Exception, i,
			_designVariables.size(), "Index out of bounds");
	return _designVariables[i];
}

_TEMPLATE
const typename _CLASS::dv_t * _CLASS::designVariable(size_t i) const {
  SM_ASSERT_LT(aslam::Exception, i, _designVariables.size(), "Index out of bounds");
  return _designVariables[i];
}

_TEMPLATE
const std::vector<typename _CLASS::dv_t *> & _CLASS::getDesignVariables() {
	return _designVariables;
}

template <typename T>
struct TimeExpEvaluator {
	static T eval(T t) { return t; }
};

template <typename Scalar_, std::uintmax_t Divider>
struct TimeExpEvaluator<aslam::backend::FixedPointNumber<Scalar_, Divider> > {
	static Scalar_ eval(aslam::backend::FixedPointNumber<Scalar_, Divider> t) { return t.getNumerator(); }
};

_TEMPLATE
template<int IMaxDerivativeOrder>
auto _CLASS::TimeExpressionFactoryData<IMaxDerivativeOrder>::getEvaluator() const -> const eval_t & {
	time_t t = TimeExpEvaluator<typename TimeExpression::Scalar>::eval(_timeExp.evaluate());
	SM_ASSERT_GE(std::runtime_error, t, _lowerBound, "The evaluation time expression evaluated to a value below its lower bound");
	SM_ASSERT_LE(std::runtime_error, t, _upperBound, "The evaluation time expression evaluated to a value above its upper bound");
	if(evalPtr) {
		if(evalPtr->getKnot() != t) {
			delete evalPtr;
			evalPtr = nullptr;
		}
	}
	if(!evalPtr){
		evalPtr = new eval_t(_spline, t);
	}
	return *evalPtr;
}


_TEMPLATE
template<int IMaxDerivativeOrder>
void _CLASS::TimeExpressionFactoryData<IMaxDerivativeOrder>::getDesignVariables(aslam::backend::DesignVariable::set_t & designVariables) const {
	for(auto & i: *this) { designVariables.insert(const_cast<aslam::backend::DesignVariable *>(&i.getDesignVariable())); }; _timeExp.getDesignVariables(designVariables);
}

_TEMPLATE
template<int IMaxDerivativeOrder>
void _CLASS::TimeExpressionFactoryData<IMaxDerivativeOrder>::makeSureForEveryDesignVariableAJacobianGetsAdded(aslam::backend::JacobianContainer & outJacobians, int rows) const {
	for(SegmentConstIterator i = this->begin(), end = this->end(); i != end; ++i)
	{
		outJacobians.add(const_cast<aslam::backend::DesignVariable *>(&i->getDesignVariable()), Eigen::MatrixXd::Zero(rows, getSpline().getDimension()));
	}
}

namespace internal {
	template <typename TDiffManifoldBSplineConfiguration, typename TDiffManifoldBSplineConfigurationDerived>
	inline void SegmentData< ::aslam::splines::DesignVariableSegmentBSplineConf<TDiffManifoldBSplineConfiguration, TDiffManifoldBSplineConfigurationDerived> >::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const {
		UpdateTraits::minimalDifference(_manifold, xHat, this->getControlVertex(), outDifference);
	}

	template <typename TDiffManifoldBSplineConfiguration, typename TDiffManifoldBSplineConfigurationDerived>
	inline void SegmentData< ::aslam::splines::DesignVariableSegmentBSplineConf<TDiffManifoldBSplineConfiguration, TDiffManifoldBSplineConfigurationDerived> >::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const {
		UpdateTraits::minimalDifferenceAndJacobian(_manifold, xHat, this->getControlVertex(), outDifference, outJacobian);
	}

	template <typename TDerived>
	inline void addJacImpl(const aslam::backend::DesignVariable * designVariable, aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd * applyChainRule, const Eigen::MatrixBase<TDerived> & block){
		if(applyChainRule){
			outJacobians.add(const_cast<aslam::backend::DesignVariable *>(designVariable), (*applyChainRule) * block);
		}
		else{
			outJacobians.add(const_cast<aslam::backend::DesignVariable *>(designVariable), block);
		}
	}

	template <typename TJacobian, int IPointSize, int IDimension, bool Dynamic = TJacobian::SizeAtCompileTime == Eigen::Dynamic>
	struct AddJac {
			static inline void addJac(TJacobian & J, const int col, const aslam::backend::DesignVariable * designVariable, aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd * applyChainRule, int /* pointSize */, int /* dimension */){
			addJacImpl(designVariable, outJacobians, applyChainRule, J.template block<IPointSize, IDimension>(0,col));
		}
	};

	template <typename TJacobian, int IPointSize, int IDimension>
	struct AddJac<TJacobian, IPointSize, IDimension, true> {
		static inline void addJac(TJacobian & J, const int col, const aslam::backend::DesignVariable * designVariable, aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd * applyChainRule, int pointSize, int dimension){
			addJacImpl(designVariable, outJacobians, applyChainRule, J.block(0,col, pointSize, dimension));
		}
	};
}

_TEMPLATE
template <typename FactoryData_>
typename _CLASS::expression_t _CLASS::ExpressionFactory<FactoryData_>::getValueExpression(const int derivativeOrder) const {
	typedef aslam::backend::VectorExpressionNode<PointSize> node_t;

	class ExpressionNode : public node_t {
		typedef typename node_t::vector_t vector_t;
		DataSharedPtr _dataPtr;
		const int _derivativeOrder;
	public:
		ExpressionNode(const DataSharedPtr & dataPtr, int derivativeOrder) : _dataPtr(dataPtr), _derivativeOrder(derivativeOrder){}
		virtual ~ExpressionNode(){}
		virtual int getSize() const override { return _dataPtr->getSpline().getPointSize(); }
	protected:
		inline void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd * applyChainRule) const {
			const int dimension=_dataPtr->getSpline().getDimension(), pointSize = _dataPtr->getSpline().getPointSize(), splineOrder = _dataPtr->getSpline().getSplineOrder();
			typename _CLASS::full_jacobian_t J(pointSize, dimension * splineOrder);
			auto & eval = _dataPtr->getEvaluator();
			eval.evalJacobian(_derivativeOrder, J);
			int col = 0;
			for(SegmentConstIterator i = eval.begin(), end = eval.end(); i != end; ++i)
			{
				internal::AddJac<typename _CLASS::full_jacobian_t, PointSize, Dimension>::addJac(J, col, &i->getDesignVariable(), outJacobians, applyChainRule, pointSize, dimension);
				col+=dimension;
			}
			if(_dataPtr->hasTimeExpression()){
				/* hack around the problem with the aslam optimizer and not adding Jacobians for all design variables to the JacobianContainer
				 * See :  https://github.com/ethz-asl/aslam_optimizer/issues/38.
				 * */
				_dataPtr->makeSureForEveryDesignVariableAJacobianGetsAdded(outJacobians, applyChainRule ? applyChainRule->rows() : pointSize);
				/* hack end */
				auto evalJac = eval.evalD(_derivativeOrder + 1);
				_dataPtr->getTimeExpression().evaluateJacobians(outJacobians, applyChainRule ? Eigen::MatrixXd(*applyChainRule * evalJac) : Eigen::MatrixXd(evalJac));
			}
		}

		virtual vector_t evaluateImplementation() const {
			return _dataPtr->getEvaluator().evalD(_derivativeOrder);
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
	return expression_t(boost::shared_ptr<node_t>(static_cast<node_t *> (new ExpressionNode(this->getDataPtr(), derivativeOrder))));
}

_TEMPLATE
std::vector<typename _CLASS::dv_t *> _CLASS::getDesignVariables(time_t tk) {
	std::vector<dv_t *> dvs(this->getSplineOrder());
	using namespace std;

	int j = 0;
	for (SegmentIterator back = this->getSegmentIterator(tk), i = this->getFirstRelevantSegmentByLast(back), end=++back; i != end; ++i) {
		dvs[j++] = &i->getDesignVariable();
	}
	return dvs;
}


_TEMPLATE
typename _CLASS::time_t _CLASS::appendSegments(KnotGenerator<time_t> & knotGenerator, int numSegments, const point_t * value) {
	SegmentIterator i = this->end();

	time_t t = parent_t::appendSegments(knotGenerator, numSegments, value);
	for(SegmentIterator end = this->end(); i != end; i++)
	{
		_designVariables.push_back((dv_t* )& i->getDesignVariable());
	}
	return t;
}

_TEMPLATE
void _CLASS::removeSegment() {
//TODO	_bspline.removeCurveSegment();
//	_designVariables.erase(_designVariables.begin());
}

#undef _CLASS
#undef _TEMPLATE

} // namespace bsplines

#ifdef USE_EXTERN_TEMPLATES
#ifndef USE_EXTERN_TEMPLATES_INSTANCIATION_FILE
#define OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD extern
#else
#define OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD
#endif
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>

#ifdef USE_EXTERN_TEMPLATES_INSTANCIATION_FILE
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::bsplines::DiffManifoldBSpline< ::bsplines::EuclideanBSplineConfiguration< ::manifolds::EuclideanSpaceConf<> > >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline< ::bsplines::EuclideanBSplineConfiguration< ::manifolds::EuclideanSpaceConf<> > >;

OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::bsplines::DiffManifoldBSpline< ::bsplines::EuclideanBSplineConfiguration< ::manifolds::EuclideanSpaceConf<>, 1> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline< ::bsplines::EuclideanBSplineConfiguration< ::manifolds::EuclideanSpaceConf<>, 1> >;

OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::bsplines::DiffManifoldBSpline< ::bsplines::EuclideanBSplineConfiguration< ::manifolds::EuclideanSpaceConf<>, 2> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline< ::bsplines::EuclideanBSplineConfiguration< ::manifolds::EuclideanSpaceConf<>, 2> >;


OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::bsplines::DiffManifoldBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< > >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::DesignVariableSegmentBSplineConf< ::bsplines::UnitQuaternionBSplineConfiguration< > >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< > >;

OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::bsplines::DiffManifoldBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 1> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::DesignVariableSegmentBSplineConf< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 1> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 1> >;

OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::bsplines::DiffManifoldBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 2> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::DesignVariableSegmentBSplineConf< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 2> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 2> >;

OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::bsplines::DiffManifoldBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 3> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::DesignVariableSegmentBSplineConf< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 3> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 3> >;

OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::bsplines::DiffManifoldBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 4> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::DesignVariableSegmentBSplineConf< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 4> >;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline< ::bsplines::UnitQuaternionBSplineConfiguration< ::manifolds::UnitQuaternionManifoldConf<>, 4> >;

OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<3>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<4>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<Eigen::Dynamic, 1>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<2, 1>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<3, 1>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<4, 1>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<Eigen::Dynamic, 2>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<2, 2>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<3, 2>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<4, 2>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<Eigen::Dynamic, 3>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<2, 3>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<3, 3>::CONF>;
OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD template class ::aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<4, 3>::CONF>;
#endif

#undef OPTBSPLINEIMPL_HPP_EXTERN_KEYWORD

#endif

#endif /* OPTBSPLINEIMPL_HPP_ */

/*
 * OPTBSpline.hpp
 *
 *  Created on: 05.08.2012
 *      Author: hannes
 */

#ifndef OPTUNITQUATERNIONBSPLINE_HPP_
#define OPTUNITQUATERNIONBSPLINE_HPP_

#include "OPTBSpline.hpp"
#include "bsplines/UnitQuaternionBSpline.hpp"

namespace bsplines {

template <typename TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy, typename TModifiedDerivedConf>
class DiffManifoldBSpline<aslam::splines::DesignVariableSegmentBSplineConf<UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TModifiedDerivedConf>, aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> >
	: public DiffManifoldBSpline<aslam::splines::DesignVariableSegmentBSplineConf<typename UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>::ParentConf, TModifiedDerivedConf>, aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> >{
protected:
	typedef DiffManifoldBSpline<aslam::splines::DesignVariableSegmentBSplineConf<typename UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>::ParentConf, TModifiedDerivedConf>, aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> > parent_t;
	typedef aslam::splines::DesignVariableSegmentBSplineConf<TModifiedDerivedConf> CONF;
private:
	typedef typename ::bsplines::UnitQuaternionBSpline<ISplineOrder>::TYPE TBSpline;
public:
	typedef typename parent_t::spline_t spline_t;
	typedef typename parent_t::TimePolicy TimePolicy;
	typedef typename parent_t::time_t time_t;
	typedef typename parent_t::point_t point_t;
	typedef typename parent_t::SegmentIterator SegmentIterator;
	typedef typename parent_t::SegmentConstIterator SegmentConstIterator;
	typedef typename parent_t::angular_jacobian_t angular_jacobian_t;

	enum {
		PointSize = TBSpline::PointSize,
		Dimension = TBSpline::Dimension
	};

	typedef aslam::backend::VectorExpression<3> angular_derivative_expression_t;

	DiffManifoldBSpline(const CONF & config = CONF()) : parent_t(config){}
	DiffManifoldBSpline(int splineOrder) : parent_t(typename CONF::ParentConf(splineOrder)){}

	template <typename FactoryData_>
	class ExpressionFactory : public parent_t::template ExpressionFactory<FactoryData_> {
	public:
		typedef typename parent_t::template ExpressionFactory<FactoryData_> ParentExpressionFactory;
		typedef typename ParentExpressionFactory::DataSharedPtr DataSharedPtr;
		typedef typename FactoryData_::eval_t eval_t;

		/// \brief get an expression
		angular_derivative_expression_t getAngularVelocityExpression() const;
		angular_derivative_expression_t getAngularAccelerationExpression() const;
	protected:
		inline ExpressionFactory(const FactoryData_ & factoryBase) : ParentExpressionFactory(factoryBase) {}
		friend class DiffManifoldBSpline;
	};

	template <int IMaxDerivativeOrder> inline ExpressionFactory<typename parent_t::template ConstTimeFactoryData<IMaxDerivativeOrder> > getExpressionFactoryAt(const time_t & t) const {
		return ExpressionFactory<typename parent_t::template ConstTimeFactoryData<IMaxDerivativeOrder> >(typename parent_t::template ConstTimeFactoryData<IMaxDerivativeOrder>(*this, t));
	}
	template <int IMaxDerivativeOrder> inline ExpressionFactory<typename parent_t::template TimeExpressionFactoryData<IMaxDerivativeOrder> > getExpressionFactoryAt(const typename parent_t::TimeExpression & t, time_t lowerBound, time_t upperBound) const {
		return ExpressionFactory<typename parent_t::template TimeExpressionFactoryData<IMaxDerivativeOrder> >(typename parent_t::template TimeExpressionFactoryData<IMaxDerivativeOrder>(*this, t, lowerBound, upperBound));
	}
};

} // namespace bsplines

#include "implementation/OPTUnitQuaternionBSplineImpl.hpp"

#endif /* OPTUNITQUATERNIONBSPLINE_HPP_ */

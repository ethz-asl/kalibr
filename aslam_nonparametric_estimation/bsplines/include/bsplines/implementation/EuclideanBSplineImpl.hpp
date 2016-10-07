/*
 * EuclideanBSpline.hpp
 *
 *  Created on: May 10, 2012
 *      Author: hannes
 */

#include "bsplines/EuclideanBSpline.hpp"

namespace bsplines {
#define _TEMPLATE template <typename TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy, typename TConfigurationDerived>
#define _CLASS DiffManifoldBSpline<EuclideanBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TConfigurationDerived>

	_TEMPLATE
	template <enum _CLASS::IteratorPosition EPosition, typename TIterator>
	inline void _CLASS::stepIterator (TIterator & it){
		if(EPosition == IteratorPosition_first) it++;
		else it--;
	}

	_TEMPLATE
	template <enum _CLASS::IteratorPosition EPosition, enum _CLASS::AddOrSet EAddOrSet, typename TCoefficientVector>
	inline void _CLASS::computeControlVertexSequenceLinearCombinationInto(const SegmentMapConstIterator & it, const TCoefficientVector & coefficients, point_t & result) {
		SegmentMapConstIterator lIt = it;

		if(EPosition == IteratorPosition_end){
			lIt --;
		}

		for(
			int
				start = EPosition == IteratorPosition_first ? 0 : coefficients.size() - 1,
				i = start,
				end = EPosition == IteratorPosition_first ? coefficients.size() : -1;
			i != end ;
			stepIterator<EPosition>(i)
		){
			if(EAddOrSet == AddOrSet_add || i != start){
				result += coefficients[i] * lIt->second.getControlVertex();
			}
			else result = coefficients[i] * lIt->second.getControlVertex();
			stepIterator<EPosition>(lIt);
		}
	}

	_TEMPLATE
	typename _CLASS::point_t _CLASS::evalIntegral(const time_t & t1, const time_t & t2) const
	{
		if(t1 > t2) return -evalIntegral(t2, t1);

		point_t integral = point_t::Zero((int)this->getDimension());
		if(t1 == t2) return integral;

		const int splineOrder = this->getSplineOrder();

		Evaluator<0> eval1 = this->getEvaluatorAt<0>(t1);

		// LHS remainder.
		SplineOrderVector v(splineOrder);
		double segmentLength = this->getDurationAsDouble(eval1.getSegmentLength());
		if(segmentLength > 1e-16 && eval1.getRelativePositionInSegment() > 1e-16){
			eval1.computeLocalViInto(v);
			computeControlVertexSequenceLinearCombinationInto<IteratorPosition_first, AddOrSet_add>(eval1.getFirstRelevantSegmentIterator(), (-segmentLength) * v, integral);
		}

		// central time segments.
		for(int i = 0; i < splineOrder; i++) v(i) = 1.0/(i + 1.0);

		auto it = eval1.getLastRelevantSegmentIterator();
		Evaluator<0> eval2 = this->getEvaluatorAt<0>(t2);
		const auto end = eval2.getLastRelevantSegmentIterator();

		SplineOrderVector localVi(splineOrder);
		for(; it != end; it++)
		{
			this->computeLocalViInto(v, localVi, it);
			computeControlVertexSequenceLinearCombinationInto<IteratorPosition_last, AddOrSet_add>(it, this->getDurationAsDouble(this->computeSegmentLength(it)) * localVi, integral);
		}

		// RHS remainder.
		segmentLength = this->getDurationAsDouble(eval2.getSegmentLength());
		if(segmentLength > 1e-16 && eval2.getRelativePositionInSegment() > 1e-16){
			eval2.computeLocalViInto(v);
			computeControlVertexSequenceLinearCombinationInto<IteratorPosition_first, AddOrSet_add>(eval2.getFirstRelevantSegmentIterator(), segmentLength * v, integral);
		}
		return integral;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	_CLASS::Evaluator<IMaximalDerivativeOrder>::Evaluator(const spline_t & spline, const time_t & t) : parent_t::template Evaluator<IMaximalDerivativeOrder> (spline, t)
	{
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::eval() const
	{
		return evalD(0);
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalD(int derivativeOrder) const
	{
		SM_ASSERT_GE(typename parent_t::Exception, derivativeOrder, 0, "To integrate, use the integral function");
		if(IMaximalDerivativeOrder != Eigen::Dynamic){
			SM_ASSERT_LT(typename parent_t::Exception, derivativeOrder, IMaximalDerivativeOrder + 1, "only derivatives up to the evaluator's template argument IMaximalDerivativeOrder are allowed");
		}

		point_t rv((int)this->_spline.getDimension());
		if(derivativeOrder < this->_spline.getSplineOrder())
			computeControlVertexSequenceLinearCombinationInto<IteratorPosition_last, AddOrSet_set>(this->getLastRelevantSegmentIterator(), this->getLocalBi(derivativeOrder), rv);
		else rv.setZero();
		return rv;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	void _CLASS::Evaluator<IMaximalDerivativeOrder>::evalJacobian(int derivativeOrder, full_jacobian_t & jacobian) const
	{
		SM_ASSERT_GE(typename parent_t::Exception, derivativeOrder, 0, "To integrate, use the integral function");
		if(IMaximalDerivativeOrder != Eigen::Dynamic){
			SM_ASSERT_LT(typename parent_t::Exception, derivativeOrder, IMaximalDerivativeOrder + 1, "only derivatives up to the evaluator's template argument IMaximalDerivativeOrder are allowed");
		}
		// The Jacobian
		const int D = this->_spline.getDimension(), splineOrder = this->_spline.getSplineOrder();
		if(jacobian.rows() != D || jacobian.cols() != D * splineOrder) jacobian.resize(D, D * splineOrder);
		const SplineOrderVector & u = this->getLocalBi(derivativeOrder);

		const auto id = (Eigen::Matrix<double, parent_t::Dimension, parent_t::Dimension>::Identity(D, D));
		for(int i = 0; i < splineOrder; i++)
		{
			jacobian.block(0, i*D, D, D) = id * u[i];
		}
	}

#undef _CLASS
#undef _TEMPLATE
}

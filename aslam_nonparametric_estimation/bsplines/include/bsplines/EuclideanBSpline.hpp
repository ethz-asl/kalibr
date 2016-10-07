/*
 * EuclideanBSpline.hpp
 *
 *  Created on: May 10, 2012
 *      Author: hannes
 */

#ifndef FLATBSPLINE_HPP_
#define FLATBSPLINE_HPP_

#include "manifolds/EuclideanSpace.hpp"
#include "DiffManifoldBSpline.hpp"

namespace bsplines {
	template <typename TDiffManifoldConfiguration, int ISplineOrder = Eigen::Dynamic, typename TTimePolicy = DefaultTimePolicy>
	struct EuclideanBSplineConfiguration : public DiffManifoldBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy> {
	public:
		typedef TDiffManifoldConfiguration ManifoldConf;
		typedef DiffManifoldBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy> ParentConf;
		typedef EuclideanBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy> Conf;
		typedef DiffManifoldBSpline<Conf> BSpline;

#if __cplusplus >= 201103L
		using ParentConf::ParentConf;
#else
		EuclideanBSplineConfiguration(TDiffManifoldConfiguration manifoldConfiguration = TDiffManifoldConfiguration(), int splineOrder = ISplineOrder) : ParentConf(manifoldConfiguration, splineOrder) {}
#endif
	};

	template <typename TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy, typename TConfigurationDerived>
	class DiffManifoldBSpline<EuclideanBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TConfigurationDerived> : public DiffManifoldBSpline<typename EuclideanBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>::ParentConf, TConfigurationDerived> {
	public:
		typedef DiffManifoldBSpline<EuclideanBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TConfigurationDerived> this_t;
		typedef DiffManifoldBSpline<typename EuclideanBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>::ParentConf, TConfigurationDerived> parent_t;
		typedef typename parent_t::configuration_t configuration_t;
		typedef typename parent_t::spline_t spline_t;
		typedef typename parent_t::manifold_t manifold_t;
		typedef typename parent_t::time_t time_t;
		typedef typename parent_t::point_t point_t;
		typedef typename parent_t::SplineOrderVector SplineOrderVector;
		typedef typename parent_t::SegmentMapConstIterator SegmentMapConstIterator;
		typedef typename parent_t::full_jacobian_t full_jacobian_t;

		DiffManifoldBSpline(int splineOrder = parent_t::SplineOrder, int dimension = parent_t::Dimension) : parent_t(configuration_t (typename configuration_t::ManifoldConf(dimension), splineOrder)){}
		DiffManifoldBSpline(configuration_t conf) : parent_t(conf){}

		point_t evalIntegral(const time_t & t1, const time_t & t2) const;

		template<int IMaximalDerivativeOrder>
		class Evaluator : public parent_t::template Evaluator<IMaximalDerivativeOrder> {
		public :
			Evaluator(const spline_t & spline, const time_t & t);

			point_t eval() const;

			point_t evalD(int derivativeOrder) const;

			void evalJacobian(int derivativeOrder, full_jacobian_t & jacobian) const;

			//friends:
			template <typename _TDiffManifoldBSplineConfiguration, typename _TConfigurationDerived> friend class DiffManifoldBSpline;
			FRIEND_TEST(EuclideanBSplineTestSuite, testEuclideanDiffManifoldBSplineBehavesAsVectorSpaceBSpline);
		};

		template<int IMaximalDerivativeOrder>
		inline Evaluator<IMaximalDerivativeOrder> getEvaluatorAt(const time_t & t) const { return parent_t::template getEvaluatorAt<IMaximalDerivativeOrder>(t); }

	protected:
		enum IteratorPosition { IteratorPosition_first, IteratorPosition_last, IteratorPosition_end };
		enum AddOrSet { AddOrSet_add, AddOrSet_set };

		template<enum IteratorPosition EPosition, typename TIterator>
		inline static void stepIterator(TIterator & it);

		template <enum IteratorPosition EPosition, enum AddOrSet EAddOrSet, typename TCoefficientVector>
		inline static void computeControlVertexSequenceLinearCombinationInto(const SegmentMapConstIterator & it, const TCoefficientVector & coefficients, point_t & result);

		template<int IMaximalDerivativeOrder> friend class Evaluator;
	};


	template <int ISplineOrder = Eigen::Dynamic, int IDimension = Eigen::Dynamic, typename TTimePolicy = DefaultTimePolicy, typename TScalar = double>

#if __cplusplus >= 201103L
	using EuclideanBSpline = typename EuclideanBSplineConfiguration<manifolds::EuclideanSpaceConf<IDimension, TScalar>, ISplineOrder, TTimePolicy>::BSpline;
#else
	class EuclideanBSpline {
	public:
		typedef EuclideanBSplineConfiguration<manifolds::EuclideanSpaceConf<IDimension, TScalar>, ISplineOrder, TTimePolicy> CONF;
		typedef typename CONF::BSpline TYPE;
	};
#endif
}

#include "implementation/EuclideanBSplineImpl.hpp"

#endif /* FLATBSPLINE_HPP_ */

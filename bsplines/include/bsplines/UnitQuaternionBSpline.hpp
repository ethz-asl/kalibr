/*
 * UnitQuaternionBSpline.hpp
 *
 *  Created on: May 10, 2012
 *      Author: hannes
 */

#ifndef UNITQUATERNIONBSPLINE_HPP_
#define UNITQUATERNIONBSPLINE_HPP_

#include "manifolds/UnitQuaternionManifold.hpp"
#include "DiffManifoldBSpline.hpp"

namespace bsplines {
	template <typename TDiffManifoldConfiguration = manifolds::UnitQuaternionManifoldConf<>, int ISplineOrder = Eigen::Dynamic, typename TTimePolicy = DefaultTimePolicy>
	struct UnitQuaternionBSplineConfiguration : public DiffManifoldBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy> {
	public:
		typedef DiffManifoldBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy> ParentConf;
		typedef UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy> Conf;
		typedef TDiffManifoldConfiguration ManifoldConf;
		typedef DiffManifoldBSpline<Conf> BSpline;

		UnitQuaternionBSplineConfiguration(TDiffManifoldConfiguration manifoldConfiguration = TDiffManifoldConfiguration(), int splineOrder = ISplineOrder) : ParentConf(manifoldConfiguration, splineOrder) {}
		UnitQuaternionBSplineConfiguration(int splineOrder) : ParentConf(TDiffManifoldConfiguration(), splineOrder) {}
	};

	namespace internal {
		template <typename TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy>
		struct DiffManifoldBSplineTraits<UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy> > {
			enum { NeedsCumulativeBasisMatrices = true };
		};
	}

	template <typename TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy, typename TConfigurationDerived>
	class DiffManifoldBSpline<UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TConfigurationDerived> : public DiffManifoldBSpline<typename UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>::ParentConf, TConfigurationDerived> {
		typedef DiffManifoldBSpline<typename UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>::ParentConf, TConfigurationDerived> parent_t;
	public:
		typedef typename parent_t::configuration_t configuration_t;
		typedef typename parent_t::spline_t spline_t;
		typedef typename parent_t::manifold_t manifold_t;
		typedef typename parent_t::time_t time_t;
		typedef typename parent_t::point_t point_t;
		typedef typename parent_t::tangent_vector_t tangent_vector_t;
		typedef typename parent_t::dmatrix_t dmatrix_t;
		typedef typename parent_t::full_jacobian_t full_jacobian_t;
		typedef typename parent_t::SplineOrderVector SplineOrderVector;
		typedef typename parent_t::SegmentMapConstIterator SegmentMapConstIterator;
		typedef Eigen::Matrix<double, configuration_t::Dimension::VALUE, multiplyEigenSize(configuration_t::Dimension::VALUE, ISplineOrder) > angular_jacobian_t;

		SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

		enum {
			MaxSupportedDerivativeOrderEvaluation = 3,
			MaxSupportedDerivativeOrderJacobian = 2
		};

		DiffManifoldBSpline(int splineOrder = parent_t::SplineOrder) : parent_t(UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>(typename configuration_t::ManifoldConf(), splineOrder)){}
		DiffManifoldBSpline(const configuration_t & conf) : parent_t(conf){}

	public:
		template<int IMaximalDerivativeOrder>
		class Evaluator : public parent_t::template Evaluator<IMaximalDerivativeOrder> {
			template <typename TSpline, int IDerivativeOrder> friend struct SplineEvalRiDTester;
			template <typename TSpline, int IDerivativeOrder> friend struct SplineEvalRiDJacobianTester;
			template <typename TSpline, int IDerivativeOrder> friend struct AngularDerivativeJacobianEvaluator;

		public :
			Evaluator(const spline_t & spline, const time_t & t);

			point_t evalD(int derivativeOrder) const;

			point_t evalD1Special() const;

			point_t evalDRecursive(int derivativeOrder) const;

			tangent_vector_t evalAngularVelocity() const;

			tangent_vector_t evalAngularAcceleration() const;

			void evalJacobian(int derivativeOrder, full_jacobian_t & jacobian) const;

			void evalJacobianDRecursive(int derivativeOrder, full_jacobian_t & jacobian) const;

			void evalJacobian(full_jacobian_t & jacobian) const;

			void evalAngularVelocityJacobian(angular_jacobian_t & jacobian) const;

			void evalAngularAccelerationJacobian(angular_jacobian_t & jacobian) const;

			template <int IDerivativeOrder>
			tangent_vector_t evalAngularDerivative() const;

			template <int IDerivativeOrder>
			void evalAngularDerivativeJacobian(angular_jacobian_t & jacobian) const;
		private:
			struct CalculationCache;
			inline int getNumVectors() const;

			point_t evalRiD(const CalculationCache & cache, int derivativeOrder, int i) const;

			point_t evalDRecursiveProductRest(const CalculationCache & cache, int derivativeOrder, int i, const int kombinatorialFactor) const;

			point_t getRiQuaternionProduct(const CalculationCache & cache, int from, int to) const;

			Eigen::Matrix<double, spline_t::Dimension, spline_t::Dimension> getPhiVectorJacobian(const CalculationCache & cache, int i) const;

			static bool isJacobianZero(int i, int j);

			static double getJacobianSignum(int i, int j);

			dmatrix_t getRiDJacobian(const CalculationCache & cache, int derivativeOrder, int i, int j) const;

			dmatrix_t getRiDJacobian(const CalculationCache & cache, int derivativeOrder, int i) const;
			dmatrix_t evalJacobianDRecursiveProductRest(const CalculationCache & cache, int derivativeOrder, int i, const int kombinatorialFactor, int j) const;
		};

		template<int IMaximalDerivativeOrder>
		inline Evaluator<IMaximalDerivativeOrder> getEvaluatorAt(const time_t & t) const { return parent_t::template getEvaluatorAt<IMaximalDerivativeOrder>(t); }
	};


	template <int ISplineOrder = Eigen::Dynamic, typename TTimePolicy = DefaultTimePolicy, typename TScalar = double>
#if __cplusplus >= 201103L
	using UnitQuaternionBSpline = typename UnitQuaternionBSplineConfiguration<manifolds::UnitQuaternionManifoldConf<TScalar>, ISplineOrder, TTimePolicy>::BSpline;
#else
	class UnitQuaternionBSpline {
	public:
		typedef UnitQuaternionBSplineConfiguration<manifolds::UnitQuaternionManifoldConf<TScalar>, ISplineOrder, TTimePolicy> CONF;
		typedef typename CONF::BSpline TYPE;
	};
#endif
}

#include "implementation/UnitQuaternionBSplineImpl.hpp"
#endif /* UNITQUATERNIONBSPLINE_HPP_ */

/*
* UnitQuaternionBSpline.hpp
*
*  Created on: May 10, 2012
*      Author: hannes
*/

#include "bsplines//DiffManifoldBSpline.hpp"
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/rotations.hpp>

namespace bsplines{

	#define _TEMPLATE template <typename TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy, typename TConfigurationDerived>
	#define _CLASS DiffManifoldBSpline<UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TConfigurationDerived>


	using namespace sm::kinematics;
	using namespace eigenTools;
	using namespace manifolds;

	inline int factorial(int i){
		SM_ASSERT_LT_DBG(std::runtime_error, i ,  13, "factorial is only supported up to 12");
		if(i == 0) return 1;
		long ret = 1;
		for(; i > 0; i--) ret*= i;
		return ret;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	struct _CLASS::Evaluator<IMaximalDerivativeOrder>::CalculationCache {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		enum{
			NumVectors = ISplineOrder == Eigen::Dynamic ? Eigen::Dynamic : ISplineOrder - 1
		};

		//TODO optimize : store vectors in segment map

		DynOrStaticSizedArray<tangent_vector_t, NumVectors> localPhiVectors;
		DynOrStaticSizedArray<point_t, NumVectors> localRiPoints;
		DynOrStaticSizedArray<const point_t *, ISplineOrder> localControlVertices;

		inline const tangent_vector_t getLocalPhiVector(int i) const {
			return localPhiVectors[i - 1];
		}
		inline const point_t getLocalRiPoint(int i) const {
			return localRiPoints[i - 1];
		}
		inline const point_t getLocalControlVertex(int i) const {
			return *localControlVertices[i];
		}
		inline const point_t getLocalStartPoint() const {
			return getLocalControlVertex(0);
		}

		CalculationCache(const Evaluator * eval) : localPhiVectors(eval->getNumVectors()), localRiPoints(eval->getNumVectors()), localControlVertices(eval->_spline.getSplineOrder())
		{
			const auto & geo = eval->_spline.getManifold();
			SegmentMapConstIterator it = eval->_firstRelevantControlVertexIt;
			const point_t * lastControlVertex = & it->second.getControlVertex();
			localControlVertices[0] = lastControlVertex;
			for (int k = 0, n = eval->getNumVectors(); k < n; k++){
				it++;
				const point_t * nextControlVertex = & it->second.getControlVertex();
				geo.logInto(*lastControlVertex, *nextControlVertex, localPhiVectors[k]);
				localControlVertices[k+1] = nextControlVertex;
				localRiPoints[k] = eval->_spline.getManifold().expAtId(localPhiVectors[k] * eval->getLocalCumulativeBi(0)[k+1]);
				lastControlVertex = nextControlVertex;
			}
		}
	};

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline int _CLASS::Evaluator<IMaximalDerivativeOrder>::getNumVectors() const {
		return this->_spline.getSplineOrder() - 1;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalRiD(const CalculationCache & cache, int derivativeOrder, int i) const{
//				std::cout << "d = "<< derivativeOrder << " i =" << i << std::endl;
		switch(derivativeOrder){
		case 0:
			return cache.getLocalRiPoint(i);
		case 1:
			return qplus((this->_spline.getManifold().V() * (cache.getLocalPhiVector(i) * this->getLocalCumulativeBi(1)[i])), cache.getLocalRiPoint(i));
		case 2:
			{
				point_t phiQuad = this->_spline.getManifold().V() * cache.getLocalPhiVector(i);
				point_t tmp = cache.getLocalRiPoint(i);

				double cumBiPrime = this->getLocalCumulativeBi(1)[i];
				point_t tmp2 = phiQuad * (cumBiPrime * cumBiPrime);
				tmp2[3] += this->getLocalCumulativeBi(2)[i];

				return qplus(phiQuad, qplus(tmp, tmp2));
			}
		case 3:
			{
				point_t phiQuad = this->_spline.getManifold().V() * cache.getLocalPhiVector(i);
				point_t tmp = cache.getLocalRiPoint(i);

				double cumBiPrime = this->getLocalCumulativeBi(1)[i], cummBiDoublePrime = this->getLocalCumulativeBi(2)[i];

				point_t tmp2 = phiQuad * (cumBiPrime * cumBiPrime);
				tmp2[3] += cummBiDoublePrime;

				point_t tmp3 = phiQuad * (2 * cumBiPrime * cummBiDoublePrime);
				tmp3[3] += this->getLocalCumulativeBi(3)[i];

				return qplus(phiQuad, qplus(tmp, qplus(cumBiPrime *phiQuad, tmp2) + tmp3));
			}
		default:
			throw Exception("only derivatives up to order 3 are supported yet");
		}
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	_CLASS::Evaluator<IMaximalDerivativeOrder>::Evaluator(const spline_t & spline, const time_t & t) : parent_t::template Evaluator<IMaximalDerivativeOrder> (spline, t)
	{
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalD(int derivativeOrder) const {
		SM_ASSERT_GE_LT_DBG(Exception, derivativeOrder, 0, 4, "only derivatives up to 3 are supported yet");
		if(IMaximalDerivativeOrder != Eigen::Dynamic) {
			SM_ASSERT_LT_DBG(Exception, derivativeOrder, IMaximalDerivativeOrder + 1, "only derivatives up to the evaluator's template argument IMaximalDerivativeOrder are allowed");
		}

		switch(derivativeOrder){
		case 0:
			return this->evalGeneric();
		case 1:
			return this->evalD1Special();
		default:
			return this->evalDRecursive(derivativeOrder);
		}
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalD1Special() const{
		/*
		 * we have to calculate the product rule here :
		 * the usual pattern of
		 *   B_1 A_2 A_3 ..
		 * + A_1 B_2 A_3 ..
		 * + A_1 A_2 B_3 ..
		 *
		 * can be rewritten as ( with A = A_1 A_2 A_3 ... )
		 *
		 *   B_1 A_1^-1 A
		 * + A_1 B_2 A_2^-1 A_1^-1 A
		 * + A_1 A_2 B_3 A_3^-1 A_2^-1 A_1^-1 A
		 * = ( B_1 A_1^-1
		 *   + A_1 B_2 A_2^-1 A_1^-1
		 *   + A_1 A_2 B_3 A_3^-1 A_2^-1 A_1^-1
		 *   + ... ) A
		 * the latter can save computational cost and be rather memory efficient at the same time as long as ^-1 is cheap - as it is for unit quaternions!
		 */

		CalculationCache cache(this);
		point_t A, sum, B;
		A = this->_spline.getManifold().getIdentity();
		sum.setZero();
		for (int i = 1, n = this->_spline.getSplineOrder(); i < n; i++){
			B = qplus(A, evalRiD(cache, 1, i));
			A = qplus(A, evalRiD(cache, 0, i));
			sum += qplus(B, quatInv(A));
		}
		return qplus(cache.getLocalStartPoint(), qplus(sum, A));
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalDRecursiveProductRest(const CalculationCache & cache, int derivativeOrder, int i, const int kombinatorialFactor) const {
		if(i == getNumVectors()){
//					std::cout << "r_"<< i << " = " << evalRiD(phi_vectors[vectorPos], 0, vectorPos + 1) << ", phi_"<< vectorPos << " = " << phi_vectors[vectorPos] << std::endl;
			return evalRiD(cache, derivativeOrder, i) * (kombinatorialFactor / factorial(derivativeOrder));
		}
//				std::cout << "r_"<< i << " = " << evalRiD(phi_vectors[vectorPos], 0, vectorPos + 1) << ", phi_"<< vectorPos << " = " << phi_vectors[vectorPos] << std::endl;
		point_t result = qplus(evalRiD(cache, 0, i), evalDRecursiveProductRest(cache, derivativeOrder, i+1, kombinatorialFactor));
		for(int d = 1; d <= derivativeOrder; d++){
			result += qplus(evalRiD(cache, d, i), evalDRecursiveProductRest(cache, derivativeOrder - d, i+1, kombinatorialFactor / factorial(d)));
		}
		return result;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::getRiQuaternionProduct(const CalculationCache & cache, int from, int to) const {
		point_t ret;
		if(from == to)
			ret = this->_spline.getManifold().getIdentity();
		else {
			ret = cache.getLocalRiPoint(from);
			for(int i = from + 1; i < to; i++){
				ret = qplus(ret, cache.getLocalRiPoint(i));
			}
		}
		return ret;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline Eigen::Matrix<double, _CLASS::spline_t::Dimension, _CLASS::spline_t::Dimension> _CLASS::Evaluator<IMaximalDerivativeOrder>::getPhiVectorJacobian(const CalculationCache & cache, int i) const {
		return this->_spline.getManifold().LByVec(cache.getLocalPhiVector(i)) * quat2r(quatInv(cache.getLocalControlVertex(i-1)));
	}


	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline bool _CLASS::Evaluator<IMaximalDerivativeOrder>::isJacobianZero(int i, int j){
		return i != j && j+1 != i;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline double _CLASS::Evaluator<IMaximalDerivativeOrder>::getJacobianSignum(int i, int j){
		return i == j ? 1:-1;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline typename _CLASS::dmatrix_t _CLASS::Evaluator<IMaximalDerivativeOrder>::getRiDJacobian(const CalculationCache & cache, int derivativeOrder, int i, int j) const {
		if(isJacobianZero(i, j))
			return dmatrix_t::Zero();
		return getJacobianSignum(i, j) * getRiDJacobian(cache, derivativeOrder, i);
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	typename _CLASS::dmatrix_t _CLASS::Evaluator<IMaximalDerivativeOrder>::getRiDJacobian(const CalculationCache & cache, int derivativeOrder, int i) const {
		if(derivativeOrder == 0){
//					std::cout << "r_i=\n" << cache.getLocalRiPoint(i) << "\n, q_i-1=\n" << cache.getLocalControlVertex(i - 1) << "\n, C_i-1=\n" << quat2r(quatInv(cache.getLocalControlVertex(i-1))) << ", ret = " << ret << "\n, oplus_i = \n"<< quatOPlus(cache.getLocalRiPoint(i))<< "\n, beta_i = \n" << this->getLocalCumulativeBi(0)[i]<< std::endl;
			return quatOPlus(cache.getLocalRiPoint(i)) * this->_spline.getManifold().V() * (this->_spline.getManifold().S(this->getLocalCumulativeBi(0)[i] * cache.getLocalPhiVector(i)) * this->getLocalCumulativeBi(0)[i] * getPhiVectorJacobian(cache, i));
		}

		dmatrix_t VPhiJac = this->_spline.getManifold().V() * getPhiVectorJacobian(cache, i);
		point_t phiQuad = this->_spline.getManifold().V() * cache.getLocalPhiVector(i);
		dmatrix_t PhiRiJac = quatOPlus(cache.getLocalRiPoint(i)) * VPhiJac + quatPlus(phiQuad) * getRiDJacobian(cache, 0, i);
		switch(derivativeOrder){
		case 1:
			return this->getLocalCumulativeBi(derivativeOrder)[i] * PhiRiJac;
		case 2:
			{
				double cumBiPrime = this->getLocalCumulativeBi(1)[i];
				point_t tmp = phiQuad * (cumBiPrime * cumBiPrime);
				tmp[3] += this->getLocalCumulativeBi(2)[i];

				return quatPlus(tmp) * PhiRiJac + quatOPlus(qplus(phiQuad, cache.getLocalRiPoint(i))) * (cumBiPrime * cumBiPrime) * VPhiJac;
			}
//				case 3: //TODO implement: Jacobian of third order time derivative
//					{
//						point_t phiQuad = this->_spline.getManifold().V() * cache.getLocalPhiVector(i);
//						point_t tmp = cache.getLocalRiPoint(i);
//
//						double cummBiPrime = this->getLocalCumulativeBi(1)[i], cummBiDoublePrime = this->getLocalCumulativeBi(2)[i];
//
//						point_t tmp2 = phiQuad * (cummBiPrime * cummBiPrime);
//						tmp2[3] += cummBiDoublePrime;
//
//						point_t tmp3 = phiQuad * (2 * cummBiPrime * cummBiDoublePrime);
//						tmp3[3] += this->getLocalCumulativeBi(3)[i];
//
//						return qplus(phiQuad, qplus(tmp, qplus(cummBiPrime *phiQuad, tmp2) + tmp3));
//					}
		default:
			throw Exception("only derivatives up to order 2 are supported yet");
		}
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	typename _CLASS::dmatrix_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalJacobianDRecursiveProductRest(const CalculationCache & cache, int derivativeOrder, int i, const int kombinatorialFactor, int j) const {
		if(i == getNumVectors()){
			if(!isJacobianZero(i, j))
				return getRiDJacobian(cache, derivativeOrder, i, j) * (kombinatorialFactor / factorial(derivativeOrder));
			else return dmatrix_t::Zero();
		}
		dmatrix_t result = quatPlus(evalRiD(cache, 0, i)) * evalJacobianDRecursiveProductRest(cache, derivativeOrder, i+1, kombinatorialFactor, j);
		if(!isJacobianZero(i, j)){
			result += quatOPlus(evalDRecursiveProductRest(cache, derivativeOrder, i+1, kombinatorialFactor)) * getRiDJacobian(cache, 0, i, j);
		}
		for(int d = 1; d <= derivativeOrder; d++){
			if(!isJacobianZero(i, j)){
				result += quatOPlus(evalDRecursiveProductRest(cache, derivativeOrder - d, i+1, kombinatorialFactor / factorial(d))) * getRiDJacobian(cache, d, i, j);
			}
			result += quatPlus(evalRiD(cache, d, i)) * evalJacobianDRecursiveProductRest(cache, derivativeOrder - d, i+1, kombinatorialFactor / factorial(d), j);
		}
		return result;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	template <int IDerivativeOrder>
	typename _CLASS::tangent_vector_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalAngularDerivative() const{
		static_assert(IDerivativeOrder <= 3 && IDerivativeOrder >= 1, "Only angular derivatives of order 1, 2, 3 are supported.");

		point_t d = evalD(IDerivativeOrder);
		Eigen::Vector3d deps = qeps(d);
		point_t v = this->eval();
		Eigen::Vector3d veps = qeps(v);
		Eigen::Vector3d tmp = (qeta(v) * deps - qeta(d) * veps - veps.cross(deps));

		switch(IDerivativeOrder){
			case 1:
			case 2:
				return 2 * tmp;
			case 3:
			{
				point_t d1 = evalD(1);
				point_t d2 = evalD(2);
				return 2 * (tmp + qeta(d1) * qeps(d2) - qeta(d2) * qeps(d1) - qeps(d1).cross(qeps(d2)));
			}
		}
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalDRecursive(int derivativeOrder) const {
//				std::cout << "p_0 = " << *pStart << std::endl;
		CalculationCache cache(this);
		point_t riProduct = evalDRecursiveProductRest(cache, derivativeOrder, 1, factorial(derivativeOrder));
		return qplus(cache.getLocalStartPoint(), riProduct);
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline typename _CLASS::tangent_vector_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalAngularVelocity() const{
		return evalAngularDerivative<1>();
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline typename _CLASS::tangent_vector_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalAngularAcceleration() const{
		return evalAngularDerivative<2>();
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	void _CLASS::Evaluator<IMaximalDerivativeOrder>::evalJacobian(int derivativeOrder, full_jacobian_t & jacobian) const{
		SM_ASSERT_GE_DBG(Exception, derivativeOrder, 0, "To integrate, use the integral function");
		SM_ASSERT_LT_DBG(Exception, derivativeOrder, IMaximalDerivativeOrder + 1, "only derivatives up to the evaluator's template argument IMaximalDerivativeOrder are allowed");

		switch(derivativeOrder){
		case 0:
			evalJacobian(jacobian);
			break;
		default:
			evalJacobianDRecursive(derivativeOrder, jacobian);
			break;
		}
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	void _CLASS::Evaluator<IMaximalDerivativeOrder>::evalJacobianDRecursive(int derivativeOrder, full_jacobian_t & jacobian) const
	{
		SM_ASSERT_GE(Exception, derivativeOrder, 0, "To integrate, use the integral function");
		const int D = this->_spline.getPointSize(), splineOrder = this->_spline.getSplineOrder(), dim = this->_spline.getManifold().getDimension();
		const int n = this->_spline.getSplineOrder();
		if(jacobian.rows() != D || jacobian.cols() != dim * splineOrder) jacobian.resize(D, dim * splineOrder);

		CalculationCache cache(this);
		const auto & geo = this->_spline.getManifold();
		dmatrix_t riQuaternionJacobian;
		Eigen::Matrix<double, spline_t::PointSize, spline_t::PointSize> riProduct;

		for(int j = 0; j < n; j++)
		{
			auto block = jacobian.block(0, j*dim, D, dim);
			if(j == 0)
				block = quatOPlus(qplus(cache.getLocalStartPoint(), evalDRecursiveProductRest(cache, derivativeOrder, 1, factorial(derivativeOrder)))) * geo.V();
			else block.setZero();

			block += quatPlus(cache.getLocalStartPoint()) * evalJacobianDRecursiveProductRest(cache, derivativeOrder, 1, factorial(derivativeOrder), j);
		}
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	void _CLASS::Evaluator<IMaximalDerivativeOrder>::evalJacobian(full_jacobian_t & jacobian) const
	{
		const int D = this->_spline.getPointSize(), splineOrder = this->_spline.getSplineOrder(), dim = this->_spline.getManifold().getDimension();
		const int n = this->_spline.getSplineOrder();
		if(jacobian.rows() != D || jacobian.cols() != dim * splineOrder) jacobian.resize(D, dim * splineOrder);

		CalculationCache cache(this);
		const auto & geo = this->_spline.getManifold();
		dmatrix_t riQuaternionJacobian;
		Eigen::Matrix<double, spline_t::PointSize, spline_t::PointSize> riProduct;

		for(int j = 0; j < n; j++)
		{
			auto block = jacobian.block(0, j*dim, D, dim);
			if(j == 0)
				block = quatOPlus(qplus(cache.getLocalStartPoint(), getRiQuaternionProduct(cache, 1, n))) * geo.V();
			else block.setZero();

			if(j != 0){
				block += riProduct * riQuaternionJacobian;
			}

			if(j != n - 1){ // not the last j
				riQuaternionJacobian = getRiDJacobian(cache, 0, j + 1);
				if(j == 0){
					riProduct = quatPlus(cache.getLocalStartPoint());
				}
				else {
					riProduct = quatPlus(qplus(cache.getLocalStartPoint(), getRiQuaternionProduct(cache, 1, j + 1)));
				}
				if(j + 2 != n)
					riProduct *= quatOPlus(getRiQuaternionProduct(cache, j + 2, n));
				block -= riProduct * riQuaternionJacobian;
			}
		}
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	template <int IDerivativeOrder>
	void _CLASS::Evaluator<IMaximalDerivativeOrder>::evalAngularDerivativeJacobian(angular_jacobian_t & jacobian) const{
		static_assert(IDerivativeOrder <= IMaximalDerivativeOrder, "You have to set the evaluator's IMaximalDerivativeOrder template argument to at least the requested angular derivative's order!");
		point_t d = evalD(IDerivativeOrder);
		Eigen::Vector3d deps = qeps(d);
		point_t v = this->eval();
		Eigen::Vector3d veps = qeps(v);

		int dim = this->_spline.getDimension(), splineOrder = this->_spline.getSplineOrder(), pointSize = this->_spline.getPointSize();
		full_jacobian_t qJac(pointSize, dim * splineOrder), dqJac(pointSize, dim * splineOrder);
		evalJacobian(IDerivativeOrder, dqJac);

		const auto depsJac = dqJac.block(0, 0, dim, splineOrder * dim);
		const auto etaDJac = dqJac.block(dim, 0, 1, splineOrder * dim);

		evalJacobian(0, qJac);
		const auto vepsJac = qJac.block(0, 0, dim, splineOrder * dim);
		const auto etaJac = qJac.block(dim, 0, 1, splineOrder * dim);

		//TODO optimize : move the factor 2 to the d and v terms to save a big matrix scaling
//				return 2 * (qeta(v) * deps - qeta(d) * veps - veps.cross(deps));
		jacobian = 2 * (deps * etaJac + qeta(v) * depsJac
				- veps * etaDJac - qeta(d) * vepsJac
				- crossMx(veps) * depsJac
				+ crossMx(deps) * vepsJac
		);
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline void _CLASS::Evaluator<IMaximalDerivativeOrder>::evalAngularVelocityJacobian(angular_jacobian_t & jacobian) const{
		evalAngularDerivativeJacobian<1>(jacobian);
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline void _CLASS::Evaluator<IMaximalDerivativeOrder>::evalAngularAccelerationJacobian(angular_jacobian_t & jacobian) const{
		evalAngularDerivativeJacobian<2>(jacobian);
	}

	#undef _CLASS
	#undef _TEMPLATE
} // namespace bsplines

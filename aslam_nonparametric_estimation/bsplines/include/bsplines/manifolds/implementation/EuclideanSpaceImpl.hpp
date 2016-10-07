/*
 * EuclideanSpaceImpl.hpp
 *
 *  Created on: Jul 26, 2012
 *      Author: hannes
 */


#include "bsplines/manifolds/EuclideanSpace.hpp"

namespace manifolds {
#define _TEMPLATE template <int IDimension, typename TScalar, typename TConfigurationDerived>
#define _CLASS DiffManifold<EuclideanSpaceConf<IDimension, TScalar>, TConfigurationDerived>

	_TEMPLATE
	inline void _CLASS::getIdentityInto(point_t & pt) const {
		pt = point_t::Zero((int)this->getPointSize());
	}

	_TEMPLATE
	inline void _CLASS::multInto(const point_t & a, const point_t & b, point_t & result) const {
		result = a + b;
	}

	_TEMPLATE
	inline typename _CLASS::dmatrix_point2point_t _CLASS::dMultL(const point_t & /* mult */, bool /* oppositeMult */ ) const
	{
		return _CLASS::dmatrix_point2point_t::Identity((int)this->getPointSize(), (int)this->getPointSize());
	}

	_TEMPLATE
	inline void _CLASS::invertInto(const point_t & p, point_t & result) const {
		result = -p;
	}

	_TEMPLATE
	inline void _CLASS::logInto(const point_t & from, const point_t & to, tangent_vector_t & result) const {
		result = to - from;
	}

	_TEMPLATE
	void _CLASS::logAtIdInto(const point_t & to, tangent_vector_t & result) const{
		result = to;
	}

	_TEMPLATE
	void _CLASS::dlogInto(const point_t & point, const tangent_vector_t & vec, dmatrix_transposed_t & result) const{
		result = dmatrix_t::Identity(result.rows(), result.cols());
	}

	_TEMPLATE
	void _CLASS::dlogAtIdInto(const tangent_vector_t & /* vec */, dmatrix_transposed_t & result) const {
		result = dmatrix_t::Identity(result.rows(), result.cols());
	}

	_TEMPLATE
	inline void _CLASS::expInto(const point_t & point, const tangent_vector_t & vec, point_t & result) const {
		result = point + vec;
	}

	_TEMPLATE
	inline void _CLASS::expAtIdInto(const tangent_vector_t & vec, point_t & p) const {
		p = vec;
	}

	_TEMPLATE
	inline void _CLASS::dexpAtIdInto(const tangent_vector_t & /* vec */, dmatrix_t & result) const {
		result = dmatrix_t::Identity(result.rows(), result.cols());
	}

	_TEMPLATE
	inline void _CLASS::dexpInto(const point_t & /* from */, const tangent_vector_t & /* vec */, dmatrix_t & result) const {
		result = dmatrix_t::Identity(result.rows(), result.cols());
	}

	_TEMPLATE
	inline void _CLASS::randomizePoint(point_t & pt) const {
		pt = 2 * point_t::Random((int)this->getDimension()) - point_t::Ones((int)this->getDimension());
	}

#undef _CLASS
#undef _TEMPLATE
}

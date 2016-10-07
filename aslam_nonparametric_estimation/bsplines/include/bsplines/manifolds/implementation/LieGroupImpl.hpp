
#include "bsplines/manifolds/LieGroup.hpp"

namespace manifolds {
#define _TEMPLATE template<int IDimension, int IPointSize, typename TScalar, typename TConfigurationDerived>
#define _CLASS DiffManifold< LieGroupConf<IDimension, IPointSize, TScalar>, TConfigurationDerived>

_TEMPLATE
inline typename _CLASS::point_t _CLASS::getIdentity() const
{
	point_t p((int)this->getPointSize());
	this->getDerived().getIdentityInto(p);
	return p;
}

_TEMPLATE
inline typename _CLASS::point_t _CLASS::getDefaultPoint() const
{
	return this->getDerived().getIdentity();
}

_TEMPLATE
inline typename _CLASS::point_t _CLASS::mult(const point_t & a, const point_t & b, bool oppositeMult) const
{
	point_t p(a.rows());
	this->getDerived().multIntoCO(a, b, p, oppositeMult);
	return p;
}

_TEMPLATE
inline void _CLASS::expInto(const point_t & point, const tangent_vector_t & vec, point_t & result) const
{
	this->getDerived().canonicalMultInto(point, this->getDerived().expAtId(vec), result);
}

_TEMPLATE
inline typename _CLASS::point_t _CLASS::exp(const point_t & point, const tangent_vector_t & vec) const
{
	point_t p(point.rows());
	this->getDerived().expInto(point, vec, p);
	return p;
}

_TEMPLATE
inline typename _CLASS::point_t _CLASS::expAtId(const tangent_vector_t & vec) const
{
	point_t result((int)this->getPointSize());
	this->getDerived().expAtIdInto(vec, result);
	return result;
}

_TEMPLATE
inline typename _CLASS::dmatrix_t _CLASS::dexpAtId(const tangent_vector_t & vec) const
{
	dmatrix_t result((int)this->getPointSize(), (int)this->getDimension());
	this->getDerived().dexpAtIdInto(vec, result);
	return result;
}

_TEMPLATE
inline void _CLASS::dexpInto(const point_t & point, const tangent_vector_t & vec, dmatrix_t & result) const
{
	result = this->getDerived().dCanonicalMult(point) * this->getDerived().dexpAtId(vec);
}

_TEMPLATE
inline typename _CLASS::dmatrix_t _CLASS::dexp(const point_t & point, const tangent_vector_t & vec) const
{
	dmatrix_t result((int)this->getPointSize(), (int)this->getDimension());
	this->getDerived().dexpInto(point, vec, result);
	return result;
}

_TEMPLATE
inline typename _CLASS::tangent_vector_t _CLASS::log(const point_t & from, const point_t & to) const {
	tangent_vector_t v((int)this->getDimension());
	this->getDerived().logInto(from, to, v);
	return v;
}

_TEMPLATE
inline typename _CLASS::tangent_vector_t _CLASS::logAtId(const point_t & to) const {
	tangent_vector_t v((int)this->getDimension());
	this->getDerived().logAtIdInto(to, v);
	return v;
}

_TEMPLATE
inline void _CLASS::logInto(const point_t & from, const point_t & to, tangent_vector_t & result) const {
	auto & d = this->getDerived();
	d.logAtIdInto(d.canonicalMult(d.invert(from), to), result);
}

_TEMPLATE
inline typename _CLASS::dmatrix_transposed_t _CLASS::dlogAtId(const point_t & to) const {
	dmatrix_transposed_t result((int)this->getDimension(), (int)this->getPointSize());
	this->getDerived().dlogAtIdInto(to, result);
	return result;
}

_TEMPLATE
void _CLASS::dlogInto(const point_t & from, const point_t & to, dmatrix_transposed_t & result) const {
	auto & d = this->getDerived();
	auto fromInv = d.invert(from);
	result = d.dlogAtId(d.canonicalMult(fromInv, to)) * d.dCanonicalMult(fromInv);
}

_TEMPLATE
inline typename _CLASS::dmatrix_transposed_t _CLASS::dlog(const point_t & from, const point_t & to) const {
	dmatrix_transposed_t result((int)this->getDimension(), (int)this->getPointSize());
	this->getDerived().dlogInto(from, to, result);
	return result;
}

_TEMPLATE
inline typename _CLASS::point_t _CLASS::invert(const point_t & p) const
{
	point_t result((int)this->getPointSize());
	this->getDerived().invertInto(p, result);
	return result;
}

#undef _CLASS
#undef _TEMPLATE
}

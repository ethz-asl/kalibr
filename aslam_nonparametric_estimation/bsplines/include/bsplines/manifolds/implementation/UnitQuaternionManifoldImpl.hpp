/*
* UnitQuaternionBSpline.hpp
*
*  Created on: May 10, 2012
*      Author: hannes
*/


#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/rotations.hpp>

#include "bsplines/manifolds/UnitQuaternionManifold.hpp"


namespace manifolds {

#define _TEMPLATE template <typename TScalar, typename TConfigurationDerived>
#define _CLASS DiffManifold<UnitQuaternionManifoldConf<TScalar>, TConfigurationDerived>

_TEMPLATE
inline void _CLASS::multInto(const point_t & a, const point_t & b, point_t & result)
{
	result = ::sm::kinematics::qplus(a, b);
}

_TEMPLATE
inline typename _CLASS::dmatrix_point2point_t _CLASS::dMultL(const point_t & mult, bool oppositeMult) const
{
	return !oppositeMult ? ::sm::kinematics::quatPlus(mult) : ::sm::kinematics::quatOPlus(mult);
}

_TEMPLATE
inline void _CLASS::invertInto(const point_t & p, point_t & result) const
{
	result = ::sm::kinematics::quatInv(p);
}

_TEMPLATE
inline void _CLASS::logAtIdInto(const point_t & to, tangent_vector_t & result)
{
	result = ::sm::kinematics::quat2AxisAngle(to);
}

_TEMPLATE
inline typename _CLASS::point_t _CLASS::expAtId(const tangent_vector_t & vec)
{
	return ::sm::kinematics::axisAngle2quat(vec);
}

_TEMPLATE
inline void _CLASS::expAtIdInto(const tangent_vector_t & vec, point_t & result)
{
	result = expAtId(vec);
}

_TEMPLATE
inline void _CLASS::getIdentityInto(point_t & result) {
	result = ::sm::kinematics::quatIdentity();
}

_TEMPLATE
inline const typename _CLASS::dmatrix_t & _CLASS::V(){
	return sm::kinematics::quatV<TScalar>();
}

_TEMPLATE
inline Eigen::Matrix3d _CLASS::S(const tangent_vector_t & vec){
	return ::sm::kinematics::expDiffMat(vec);
}

_TEMPLATE
inline Eigen::Matrix3d _CLASS::LByVec(const tangent_vector_t & vec){
	return ::sm::kinematics::logDiffMat(vec);
}

_TEMPLATE
void _CLASS::dlogAtIdInto(const point_t & to, dmatrix_transposed_t & result) const {
	result = ::sm::kinematics::quatLogJacobian2(to);
}

_TEMPLATE
inline void _CLASS::dexpAtIdInto(const tangent_vector_t & vec, dmatrix_t & result) const
{
	result = ::sm::kinematics::quatExpJacobian(vec);
}


_TEMPLATE
bool _CLASS::isInManifold(const point_t & pt)
{
	return fabs(pt.norm() - 1) < fabs(1E-9);
}

_TEMPLATE
void _CLASS::projectIntoManifold(point_t & pt)
{
	SM_ASSERT_GT_DBG(std::runtime_error, pt.norm(), 1E-3, "This quaternion cannot be projected into the unit quaternions!");
	pt /= pt.norm();
}

_TEMPLATE
void _CLASS::randomizePoint(point_t & pt)
{
	double norm;
	do{ //TODO improve : realize uniform distribution on S3
		pt = 5*(Eigen::Vector4d::Random() * 2 - Eigen::Vector4d::Ones());
	}while((norm = pt.norm()) < 1e-10);
	pt /= norm;
}

#undef _TEMPLATE
#undef _CLASS
}

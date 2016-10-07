/*
 * UnitQuaternionDiffManifold.hpp
 *
 *  Created on: Jul 26, 2012
 *      Author: hannes
 */

#ifndef UNITQUATERNIONGEOMETRY_HPP_
#define UNITQUATERNIONGEOMETRY_HPP_

#include "LieGroup.hpp"
#include <limits>

namespace manifolds {
	template <typename TScalar = double>
	struct UnitQuaternionManifoldConf : public LieGroupConf<3, 4, TScalar> {
		typedef LieGroupConf<3, 4, TScalar> ParentConf;
		typedef DiffManifold<ParentConf> ParentManifold;
		typedef UnitQuaternionManifoldConf Conf;
		typedef DiffManifold<Conf> Manifold;

		typename ParentConf::Dimension getDimension() const { return ParentConf::Dimension::VALUE; }
		typename ParentConf::PointSize getPointSize() const { return ParentConf::PointSize::VALUE; }
	};

	template <typename TScalar, typename TConfigurationDerived >
	class DiffManifold< UnitQuaternionManifoldConf<TScalar>, TConfigurationDerived> : public DiffManifold<typename UnitQuaternionManifoldConf<TScalar>::ParentConf, TConfigurationDerived> {
	public:
		typedef DiffManifold<typename UnitQuaternionManifoldConf<TScalar>::ParentConf, TConfigurationDerived> parent_t;
		typedef TConfigurationDerived configuration_t;
		typedef internal::DiffManifoldConfigurationTypeTrait<configuration_t> Types;
		typedef typename Types::scalar_t scalar_t;
		typedef typename Types::point_t point_t;
		typedef typename Types::tangent_vector_t tangent_vector_t;
		typedef typename Types::dmatrix_t dmatrix_t;
		typedef typename Types::dmatrix_transposed_t dmatrix_transposed_t;
		typedef typename Types::dmatrix_point2point_t dmatrix_point2point_t;

		DiffManifold(configuration_t configuration) : parent_t(configuration) {};
		DiffManifold() : parent_t(configuration_t()) {};

		static const dmatrix_t & V();
		static Eigen::Matrix3d S(const tangent_vector_t & vec);
		static Eigen::Matrix3d LByVec(const tangent_vector_t & vec);

		static void getIdentityInto(point_t & pt);
		static bool isInManifold(const point_t & pt);
		static void projectIntoManifold(point_t & pt);
		static void randomizePoint(point_t & pt);

		static void multInto(const point_t & a, const point_t & b, point_t & result);
		inline dmatrix_point2point_t dMultL(const point_t & mult, bool oppositeMult) const;
		inline void invertInto(const point_t & p, point_t & result) const;

		static void expAtIdInto(const tangent_vector_t & vec, point_t & result);
		static point_t expAtId(const tangent_vector_t & vec);
		void dexpAtIdInto(const tangent_vector_t & vec, dmatrix_t & result) const;

		static void logAtIdInto(const point_t & to, tangent_vector_t & result);
		void dlogAtIdInto(const point_t & to, dmatrix_transposed_t & result) const;
	};
}

#include "implementation/UnitQuaternionManifoldImpl.hpp"
#endif /* UNITQUATERNIONGEOMETRY_HPP_ */

/*
 * LieGroup.hpp
 *
 *  Created on: 28.07.2012
 *      Author: hannes
 */

#ifndef LIEGROUP_HPP_
#define LIEGROUP_HPP_

#include "DiffManifold.hpp"

namespace manifolds {

	template <int IDimension, int IPointSize, typename TScalar>
	struct LieGroupConf : public DiffManifoldConfiguration<IDimension, IPointSize, TScalar> {
		typedef LieGroupConf<IDimension, IPointSize, TScalar> Conf;
		typedef DiffManifoldConfiguration<IDimension, IPointSize, TScalar> ParentConf;
		typedef DiffManifold<Conf> Manifold;
	};

	namespace internal{
		template <typename TConfigurationDerived, int IDimension, int IPointSize, typename TScalar>
		struct DiffManifoldPointUpdateTraits<LieGroupConf<IDimension, IPointSize, TScalar>, TConfigurationDerived> {
			typedef manifolds::internal::DiffManifoldConfigurationTypeTrait<TConfigurationDerived> Types;
			typedef typename Types::point_t point_t;
			typedef typename Types::tangent_vector_t tangent_vector_t;
			typedef typename TConfigurationDerived::Manifold Manifold;
			static inline void update(const Manifold & manifold, point_t & point, const tangent_vector_t & vec){
		//	manifold.expInto(point, vec, point); TODO improve: cleanup the mixture of left and right canonical connection in the LieGroup BSpline algorithms. This can then become just exp.
				point = manifold.mult(manifold.expAtId(vec), point);
				manifold.projectIntoManifold(point);
			}

			static inline void minimalDifference(const Manifold & manifold, const Eigen::MatrixXd& xHat, const point_t & to, Eigen::VectorXd& outDifference) {
				SM_ASSERT_TRUE(std::runtime_error, (xHat.rows() == manifold.getPointSize()) && (xHat.cols() == 1), "xHat has incompatible dimensions");
				outDifference = manifold.logAtId(manifold.mult(to, manifold.invert(xHat)));
			};

			static inline void minimalDifferenceAndJacobian(const Manifold & manifold, const Eigen::MatrixXd& xHat, const point_t & to, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) {
				SM_ASSERT_TRUE(std::runtime_error, (xHat.rows() == manifold.getPointSize()) && (xHat.cols() == 1), "xHat has incompatible dimensions");
				auto invertedXHat = manifold.invert(xHat);
				auto quot = manifold.mult(to, invertedXHat);
				outDifference = manifold.logAtId(quot);
				outJacobian = manifold.dlogAtId(quot) * manifold.dMultL(quot, true) * manifold.dexpAtId(Manifold::tangent_vector_t::Zero((int)manifold.getDimension()));  //TODO optimize : this is quite slow (V matrix not directly used)
			};
		};
	}

	//TODO improve names / hierarchy : this is LieGroup is more a LieGroup with an affine connection (because it has geometric exp, log). This is quite unclean. It would be better to have an additional affine connection type attached to it somehow.

	template <int IDimension, int IPointSize, typename TScalar, typename TConfigurationDerived>
	class DiffManifold<LieGroupConf<IDimension, IPointSize, TScalar>, TConfigurationDerived> : public DiffManifold<typename LieGroupConf<IDimension, IPointSize, TScalar>::ParentConf, TConfigurationDerived> {
	public:
		typedef DiffManifold<typename LieGroupConf<IDimension, IPointSize, TScalar>::ParentConf, TConfigurationDerived> parent_t;
		typedef TConfigurationDerived configuration_t;
		typedef internal::DiffManifoldConfigurationTypeTrait<configuration_t> Types;
		typedef typename Types::scalar_t scalar_t;
		typedef typename Types::point_t point_t;
		typedef typename Types::tangent_vector_t tangent_vector_t;
		typedef typename Types::dmatrix_t dmatrix_t;
		typedef typename Types::dmatrix_point2point_t dmatrix_point2point_t;
		typedef typename Types::dmatrix_transposed_t dmatrix_transposed_t;
		typedef typename configuration_t::Manifold base_t;

		static constexpr inline CanonicalConnection getCanonicalConnection()  { return CanonicalConnection::LEFT; }

		DiffManifold(TConfigurationDerived confiuration) : parent_t (confiuration) {}

		void getIdentityInto(point_t & p) const;
		point_t getIdentity() const;
		point_t getDefaultPoint() const;

		void multInto(const point_t & a, const point_t & b, point_t & result) const;
		inline void multIntoCO(const point_t & a, const point_t & b, point_t & result, bool oppositeMult) const { if(oppositeMult) this->getDerived().multInto(b, a, result); else this->getDerived().multInto(a, b, result); }
		point_t mult(const point_t & a, const point_t & b, bool oppositeMult = false) const;
		inline void canonicalMultInto(const point_t & mult, const point_t & point, point_t & result) const { multIntoCO(mult, point, result, this->getDerived().getCanonicalConnection() == CanonicalConnection::RIGHT); }
		inline point_t canonicalMult(const point_t & mult, const point_t & point) const { return this->getDerived().mult(mult, point, this->getDerived().getCanonicalConnection() == CanonicalConnection::RIGHT); }
		dmatrix_point2point_t dMultL(const point_t & mult, bool oppositeMult = false) const;
		inline dmatrix_point2point_t dCanonicalMult(const point_t & mult) const { return this->getDerived().dMultL(mult, this->getDerived().getCanonicalConnection() == CanonicalConnection::RIGHT); }

		void invertInto(const point_t & p, point_t & result) const;
		point_t invert(const point_t & p) const;

		void expAtIdInto(const tangent_vector_t & vec, point_t & result) const;
		void expInto(const point_t & point, const tangent_vector_t & vec, point_t & result) const;
		void dexpAtIdInto(const tangent_vector_t & vec, dmatrix_t & result) const;
		void dexpInto(const point_t & point, const tangent_vector_t & vec, dmatrix_t & result) const;

		point_t expAtId(const tangent_vector_t & vec) const;
		point_t exp(const point_t & point, const tangent_vector_t & vec) const;
		dmatrix_t dexpAtId(const tangent_vector_t & vec) const;
		dmatrix_t dexp(const point_t & point, const tangent_vector_t & vec) const;

		void logAtIdInto(const point_t & to, tangent_vector_t & result) const;
		void logInto(const point_t & from, const point_t & to, tangent_vector_t & result) const;
		void dlogInto(const point_t & point, const point_t & to, dmatrix_transposed_t & result) const;
		void dlogAtIdInto(const tangent_vector_t & vec, dmatrix_transposed_t & result) const;

		tangent_vector_t logAtId(const point_t & to) const;
		tangent_vector_t log(const point_t & from, const point_t & to) const;
		dmatrix_transposed_t dlogAtId(const point_t & to) const;
		dmatrix_transposed_t dlog(const point_t & from, const point_t & to) const;
	};
}

#include "implementation/LieGroupImpl.hpp"

#endif /* LIEGROUP_HPP_ */

/*
 * Manifold.hpp
 *
 *  Created on: 28.07.2012
 *      Author: hannes
 */

#ifndef MANIFOLD_HPP_
#define MANIFOLD_HPP_

#include "bsplines/DynamicOrTemplateInt.hpp"
namespace manifolds {
	enum class CanonicalConnection { LEFT, RIGHT };

	template <typename TConfiguration, typename TConfigurationDerived = TConfiguration>
	class DiffManifold : public DiffManifold<typename TConfiguration::ParentConf, TConfigurationDerived>{
		typedef DiffManifold<typename TConfiguration::ParentConf, TConfigurationDerived> parent_t;
	protected:
		DiffManifold(const TConfigurationDerived & conf) : parent_t(conf) {}
	};

	class DiffManifoldConfigurationBase {
	};

	template <int IDimension, int IPointSize, typename TScalar>
	struct DiffManifoldConfiguration : public DiffManifoldConfigurationBase {
		typedef DiffManifoldConfigurationBase ParentConf;
		typedef DiffManifoldConfiguration Conf;

		typedef eigenTools::DynamicOrTemplateInt<IDimension> Dimension;
		typedef eigenTools::DynamicOrTemplateInt<IPointSize> PointSize;

		Dimension getDimension() const;
		PointSize getPointSize() const;

		typedef DiffManifold<Conf> Manifold;
		typedef TScalar scalar_t;
	};

	namespace internal {
		template <typename TConfiguration>
		struct DiffManifoldConfigurationData : public TConfiguration {
		};

		template <typename TConfiguration>
		struct DiffManifoldConfigurationTypeTrait {
			enum { Dimension = TConfiguration::Dimension::VALUE, PointSize = TConfiguration::PointSize::VALUE};

			typedef TConfiguration configuration_t;
			typedef typename TConfiguration::Manifold Manifold;
			typedef typename TConfiguration::scalar_t scalar_t;
			typedef Eigen::Matrix<scalar_t, PointSize, 1> point_t;
			typedef Eigen::Matrix<scalar_t, Dimension, 1> tangent_vector_t;
			typedef Eigen::Matrix<scalar_t, PointSize, Dimension> dmatrix_t;
			typedef Eigen::Matrix<scalar_t, Dimension, PointSize> dmatrix_transposed_t;
			typedef Eigen::Matrix<scalar_t, PointSize, PointSize> dmatrix_point2point_t;
		};

		template <typename TManifoldConf, typename TConfigurationDerived = TManifoldConf>
		struct DiffManifoldPointUpdateTraits : public DiffManifoldPointUpdateTraits<typename TManifoldConf::ParentConf, TConfigurationDerived> {
		};

		template <typename TConfigurationDerived>
		struct DiffManifoldPointUpdateTraits<DiffManifoldConfigurationBase, TConfigurationDerived> {
			typedef DiffManifoldConfigurationTypeTrait<TConfigurationDerived> Types;
			typedef typename Types::point_t point_t;
			typedef typename Types::tangent_vector_t tangent_vector_t;
			typedef typename Types::Manifold Manifold;
			inline static void update(const Manifold & manifold, point_t & point, const tangent_vector_t & vec){
				SM_THROW(std::runtime_error, NotImplementedMessage);
			}
			inline static void minimalDifference(const Manifold &, const Eigen::MatrixXd& xHat, const typename Manifold::point_t & to, Eigen::VectorXd& outDifference) {
				SM_THROW(std::runtime_error, NotImplementedMessage);
			};
			inline static void minimalDifferenceAndJacobian(const Manifold &, const Eigen::MatrixXd& xHat, const typename Manifold::point_t & to, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) {
				SM_THROW(std::runtime_error, NotImplementedMessage);
			};
		 private:
			constexpr static const char * NotImplementedMessage = "Point updates / minimal differences not implemented for this manifold. Pleas specialize manifolds::internal::DiffManifoldPointUpdateTraits";
		};
	}

	template <typename TConfigurationDerived>
	class DiffManifold<DiffManifoldConfigurationBase, TConfigurationDerived> {
	public:
		typedef internal::DiffManifoldConfigurationTypeTrait<TConfigurationDerived> Types;
		enum { Dimension = Types::Dimension, PointSize = Types::PointSize};
		typedef typename Types::configuration_t configuration_t;
		typedef typename Types::scalar_t scalar_t;
		typedef typename Types::point_t point_t;
		typedef typename Types::tangent_vector_t tangent_vector_t;
		typedef typename Types::dmatrix_t dmatrix_t;
		typedef typename Types::dmatrix_transposed_t dmatrix_transposed_t;
		typedef typename Types::dmatrix_point2point_t dmatrix_point2point_t;

		inline DiffManifold(const TConfigurationDerived & configuration) : _configuration(configuration) {}

		inline configuration_t getConfiguration() const { return _configuration; }
		inline typename configuration_t::Dimension getDimension() const { return _configuration.getDimension(); };
		inline typename configuration_t::PointSize getPointSize() const  { return _configuration.getPointSize(); };

		point_t getDefaultPoint() const;

		static inline void scaleVectorInPlace(tangent_vector_t & vec, const scalar_t scalar){ vec *= scalar; }
		static inline void scaleVectorInto(const tangent_vector_t & vec, const scalar_t scalar, tangent_vector_t & result){ result = vec * scalar;}
		static inline tangent_vector_t scaleVector(const tangent_vector_t & vec, const scalar_t scalar){ return vec * scalar; }

      bool isInManifold(const point_t & /* pt */) const { return true; }
      void projectIntoManifold(point_t & /* pt */) const { }
		void randomizePoint(point_t & pt) const { pt = point_t::random(getPointSize()); }
		point_t getRandomPoint() const { point_t p((int)getPointSize());  getDerived().randomizePoint(p); return p; }
	protected:
		TConfigurationDerived _configuration;
		typedef DiffManifold<TConfigurationDerived, TConfigurationDerived> DERIVED;
		inline DERIVED & getDerived() { return static_cast<DERIVED&>(*this); }
		inline const DERIVED & getDerived() const { return static_cast<const DERIVED&>(*this); }
	};
}

#endif /* MANIFOLD_HPP_ */

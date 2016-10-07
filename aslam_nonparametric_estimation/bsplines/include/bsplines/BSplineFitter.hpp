/*
 * DiffManifoldBSplineFitter.hpp
 *
 *  Created on: May 10, 2012
 *      Author: hannes
 */

#ifndef RIEMANNIANBSPLINEFITTER_HPP_
#define RIEMANNIANBSPLINEFITTER_HPP_

#include <functional>
#include "DynamicOrTemplateInt.hpp"
#include "DiffManifoldBSpline.hpp"
#include <sparse_block_matrix/sparse_block_matrix.h>
#include "KnotArithmetics.hpp"

namespace bsplines{
enum class FittingBackend {
	DENSE,
	SPARSE,
	DEFAULT = SPARSE
};

namespace internal {
	template<enum FittingBackend FittingBackend_> struct FittingBackendTraits {
	};

	template<> struct FittingBackendTraits<FittingBackend::DENSE> {
		typedef Eigen::MatrixXd Matrix;
		typedef Eigen::VectorXd Vector;
	};

	template<> struct FittingBackendTraits<FittingBackend::SPARSE> {
		typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Matrix;
		typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Vector;
	};
}

template <typename TSpline>
class BSplineFitter {
	typedef typename TSpline::point_t point_t;
	typedef typename TSpline::time_t time_t;
	typedef typename TSpline::duration_t duration_t;
	typedef typename TSpline::scalar_t scalar_t;
	typedef typename TSpline::TimePolicy TimePolicy;
public :
	SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
	/**
	 * Initializes a spline by fitting it to a family of time point pairs.
	 * @param spline the spline
	 * @param knotGenerator the knot generator determines the initial amount and time stamps of the created spline knots
	 * @param times the time stamps of the points to fit
	 * @param points the points to fit. The control vertices will be chosen to minimized the cumulative distance of the spline to these points. The lambda parameter (s. below) may add an additional term to this cost function.
	 * @param lambda A nonnegative fraction acting as a weight of an additional additive cost term (s. points). This term is the integral of the acceleration squared along the whole spline.
	 * @param fittingBackend the fitting backend. Usually one wants FittingBackend::SPARSE.
	 */
	static void initSpline(TSpline & spline, KnotGenerator<time_t> & knotGenerator, const std::vector<time_t> & times, const std::vector<point_t> & points, double lambda, FittingBackend fittingBackend = FittingBackend::DEFAULT);
	static void initUniformSpline(TSpline & spline, time_t minT, time_t maxT, const std::vector<time_t> & times, const std::vector<point_t> & points, int numSegments, double lambda, FittingBackend backend = FittingBackend::DEFAULT);
	static void initUniformSpline(TSpline & spline, const std::vector<time_t> & times, const std::vector<point_t> & points, int numSegments, double lambda, FittingBackend backend = FittingBackend::DEFAULT);
	static DeltaUniformKnotGenerator<TimePolicy> initUniformSplineWithKnotDelta(TSpline & spline, const std::vector<time_t> & times, const std::vector<point_t> & points, const duration_t knotDelta, double lambda, FittingBackend backend = FittingBackend::DEFAULT);

	/**
	 * Fits a part (given by the times) of an already initialized spline to a weighted family of time point pairs while not changing the other control vertices.
	 * @param spline the spline
	 * @param times the time stamps
	 * @param points the points to fit
	 * @param lambda the acceleration integral's weight (@see #initSpline)
	 * @param fixNFirstRelevantControlVertices Normally (i.e. this param = 0) all control vertices are optimized that are relevant for the splines value in the period times[0]..times[times.size() - 1]. With this great then zero the first relevant control vertices can be hold fixed.
	 * @param weights a function mapping an time point pairs' index to its weight. The empty function is semantically equivalently treated as returning always 1.0.
	 * @param fittingBackend selects the fitting backend. Usually one wants FittingBackend::SPARSE.
	 * @param calculateControlVertexOffsets decides whether in the fitting backend the vertices are globally calculated or only offsets for them. If the fitting is under constrained that makes a difference. Globally (false case) the vertices will be pulled towards zero otherwise this happens to the offsets only!
	 */
	static void fitSpline(TSpline & spline, const std::vector<time_t> & times, const std::vector<point_t> & points, double lambda, int fixNFirstRelevantControlVertices = 0, std::function<scalar_t(int i) > weights = std::function<scalar_t(int i) >(), FittingBackend fittingBackend = FittingBackend::DEFAULT, const bool calculateControlVertexOffsets = false);
	/**
	 * Extends (adding knots and control vertices) a spline as necessary to be valid in the whole times range and then fits the spline to the time point pairs given by times and points.
	 * @param spline the spline
	 * @param knotGenerator the knot generator determines the time stamps of the newly created spline knots and must be the same as used to initialize the spline!
	 * @param times the time stamps of the points to fit
	 * @param points the points to fit. The control vertices will be optimized to minimize the distance of the spline to these points. The lambda parameter (s. below) can add an additional term to this cost function.
	 * @param lambda the acceleration integral's weight (@see #initSpline)
	 * @param honorCurrentValuePercentage The current spline's value is considered according to honorCurrentValueCoefficient (0 = ignore current value, 100 = ignore points where the spline is already defined).
	 * @param fittingBackend selects the fitting backend. Usually one wants FittingBackend::SPARSE.
	 * @param calculateControlVertexOffsets decides whether in the fitting backend the vertices are globally calculated or only offsets for them. If the fitting is under constrained that makes a difference. Globally (false case) the vertices will be pulled towards zero otherwise this happens to the offsets only!
	 */
	static void extendAndFitSpline(TSpline & spline, KnotGenerator<time_t> & knotGenerator, const std::vector<time_t> & times, const std::vector<point_t> & points, double lambda, unsigned char honorCurrentValuePercentage, FittingBackend fittingBackend = FittingBackend::DEFAULT, const bool calculateControlVertexOffsets = false);

private:
	int getNumberOfRelevantControlVertices(const std::vector<time_t>& times, const time_t& upToTime, TSpline& spline, int fixFirstVertices);
	static void calcFittedControlVertices(TSpline & spline, const KnotIndexResolver<time_t> & knotIndexResolver, const std::vector<time_t> & times, const std::vector<point_t> & points, std::function<scalar_t(int i) > weights, double lambda, int fixNFirstRelevantControlVertices = 0, FittingBackend fittingBackend = FittingBackend::DEFAULT, const bool calculateControlVertexOffsets = false);
	template<enum FittingBackend FittingBackend_>
	static void calcFittedControlVertices(TSpline & spline, const KnotIndexResolver<time_t> & knotIndexResolver, const std::vector<time_t> & times, const std::vector<point_t> & points, std::function<scalar_t(int i) > weights, double lambda, int fixNFirstRelevantControlVertices = 0, const bool calculateControlVertexOffsets = false);

	template<enum FittingBackend FittingBackend_>
	static void addCurveQuadraticIntegralDiagTo(const TSpline & spline, typename TSpline::SegmentConstIterator start, typename TSpline::SegmentConstIterator end, int startIndex, const point_t & Wdiag, int derivativeOrder, typename internal::FittingBackendTraits<FittingBackend_>::Matrix & toMatrix, typename internal::FittingBackendTraits<FittingBackend_>::Vector & toB);

	template<typename M_T>
	static void addOrSetSegmentQuadraticIntegralDiag(const TSpline & spline, const point_t & Wdiag, typename TSpline::SegmentConstIterator segmentIt, int derivativeOrder, M_T toMatrix, bool add);

	template <typename M_T>
	static void computeBijInto(const TSpline & spline, const typename TSpline::SegmentConstIterator & segmentIndex, int columnIndex, M_T B);

	template <typename M_T>
	static void computeMiInto(const TSpline & spline, const typename TSpline::SegmentConstIterator & segmentIndex, M_T & M);
};

}


#include "bsplines/implementation/BSplineFitterImpl.hpp"

#endif /* RIEMANNIANBSPLINEFITTER_HPP_ */

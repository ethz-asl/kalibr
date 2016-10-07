#ifndef BSPLINEFITTERIMPL_HPP_
#define BSPLINEFITTERIMPL_HPP_

#include "DiffManifoldBSplineTools.hpp"
#include "../DynamicOrTemplateInt.hpp"
#include "../DiffManifoldBSpline.hpp"

#include <memory>
#include <map>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
#include <sparse_block_matrix/linear_solver_cholmod.h>

namespace bsplines{
#define _TEMPLATE template <typename TSpline>
#define _CLASS BSplineFitter <TSpline>

//TODO improve : support different containers of times and points
_TEMPLATE
inline void _CLASS::initUniformSpline(TSpline & spline, const std::vector<time_t> & times, const std::vector<point_t> & points, int numSegments, double lambda, FittingBackend backend){
	SM_ASSERT_GE(Exception, times.size(), 2, "There must be at least two time point pairs");
	initUniformSpline(spline, times.front(), times.back(), times, points, numSegments, lambda, backend);
}

_TEMPLATE
inline void _CLASS::initUniformSpline(TSpline & spline, time_t minT, time_t maxT, const std::vector<time_t> & times, const std::vector<point_t> & points, int numSegments, double lambda, FittingBackend backend){
	spline.assertConstructing();
	SM_ASSERT_GE(Exception, times.front(), minT, "Times must all be >= minT");
	SM_ASSERT_LE(Exception, times.back(), maxT, "Times must all be <= maxT");
	IntervalUniformKnotGenerator<TimePolicy> knotGenerator(spline.getSplineOrder(), minT, maxT, numSegments);
	initSpline(spline, knotGenerator, times, points, lambda, backend);
}

_TEMPLATE
inline DeltaUniformKnotGenerator<typename _CLASS::TimePolicy> _CLASS::initUniformSplineWithKnotDelta(TSpline & spline, const std::vector<time_t> & times, const std::vector<point_t> & points, const duration_t knotDelta, double lambda, FittingBackend backend){
	spline.assertConstructing();
	SM_ASSERT_GE(Exception, times.size(), 2, "There must be at least two time point pairs");
	DeltaUniformKnotGenerator<TimePolicy> knotGenerator(spline.getSplineOrder(), *times.begin(), *--times.end(), knotDelta);
	initSpline(spline, knotGenerator, times, points, lambda, backend);
	return knotGenerator;
}

namespace internal{
	template<typename T>
	class PointerGuard {
		bool _deleteIt;
		T * _ptr;
	 public:
		PointerGuard() : _deleteIt(false), _ptr(nullptr) {}
		inline void init(T* ptr, bool deleteIt = true) {
			_deleteIt = deleteIt;
			_ptr = ptr;
		}
		inline ~PointerGuard(){
			if(_deleteIt) delete _ptr;
		}
		inline T& operator *(){
			return *_ptr;
		}
	};

	template<typename TTime>
	struct MapKnotIndexResolver : public KnotIndexResolver<TTime>{
		std::map<TTime, int> knots;
		inline int getKnotIndexAtTime(TTime t) const { return (--knots.upper_bound(t))->second; };
	};

	template <typename TException, typename TTime>
	inline void checkTimesBounds(const std::vector<TTime>& times)
	{
		if(times.size() > 1){
			TTime min = *times.begin(), max=*--times.end();
			for(TTime t : times){
				SM_ASSERT_LE(TException, min, t, "The time sequence must be bounded by its bounding elements. But " << t << " is smaller than the first element :" << min);
				SM_ASSERT_LE(TException, t, max, "The time sequence must be bounded by its bounding elements. But " << t << " is bigger than the last element :" << max);
			}
		}
	}
}


	_TEMPLATE
	void _CLASS::initSpline(TSpline & spline, KnotGenerator<time_t> & knotGenerator, const std::vector<time_t> & times, const std::vector<point_t> & points, double lambda, FittingBackend fittingBackend){
		spline.assertConstructing();
		const size_t numPoints = points.size();

		SM_ASSERT_EQ(Exception, times.size(), numPoints, "The number of times and the number of points must be equal");
		SM_ASSERT_GE(Exception, numPoints, 2, "There must be at least two time point pairs");
		internal::checkTimesBounds<Exception>(times);

		internal::PointerGuard<const KnotIndexResolver<time_t> > resolverPtr;

		if(knotGenerator.hasKnotIndexResolver()){
			for(size_t i = 0; knotGenerator.hasNextKnot(); i++){
				spline.addKnot(knotGenerator.getNextKnot());
			}
			resolverPtr.init(&knotGenerator.getKnotIndexResolver(), false);
		}else{
			auto mapResolver = new internal::MapKnotIndexResolver<time_t>();
			resolverPtr.init(mapResolver, true);
			for(size_t i = 0; knotGenerator.hasNextKnot(); i++){
				time_t nextKnot = knotGenerator.getNextKnot();
				spline.addKnot(nextKnot);
				mapResolver->knots.insert(std::make_pair(nextKnot, i));
			}
		}

		spline.init();

		calcFittedControlVertices(spline, *resolverPtr, times, points, std::function<scalar_t(int) > (), lambda, 0, fittingBackend);
	}

	_TEMPLATE
	void _CLASS::fitSpline(TSpline & spline, const std::vector<time_t> & times, const std::vector<point_t> & points, double lambda, int fixNFirstRelevantControlVertices, std::function<scalar_t(int i) > weights, FittingBackend fittingBackend, const bool calculateControlVertexOffsets){
		spline.assertEvaluable();
		const size_t numPoints = points.size();
		SM_ASSERT_GE(Exception, numPoints, 1, "The must be at least one time point pair!");
		SM_ASSERT_EQ(Exception, times.size(), numPoints, "The number of times and the number of points must be equal");
		internal::checkTimesBounds<Exception>(times);

		auto lastTime = times[numPoints - 1];
		SM_ASSERT_GE(Exception, times[0], spline.getMinTime(), "The values in times may not exceed the time domain of the spline.");
		SM_ASSERT_LE(Exception, lastTime, spline.getMaxTime(), "The values in times may not exceed the time domain of the spline.");

		internal::MapKnotIndexResolver<time_t> mapResolver;
		int i = 0;
		for(auto it = spline.getFirstRelevantSegmentByLast(spline.getSegmentIterator(times[0])), end = spline.getAbsoluteEnd(); it != end; ++it){
			time_t nextKnot = it->getKnot();
			if(nextKnot > lastTime) break;
			spline.addKnot(nextKnot);
			mapResolver.knots.insert(std::make_pair(nextKnot, i++));
		}

		calcFittedControlVertices(spline, mapResolver, times, points, weights, lambda, fixNFirstRelevantControlVertices, fittingBackend, calculateControlVertexOffsets);
	}

namespace internal{
	template<typename TSpline>
	unsigned int getNumberOfRelevantControlVertices(TSpline& spline, typename TSpline::time_t startTime, const typename TSpline::time_t& upToTime, std::function<void(typename TSpline::SegmentIterator it)> apply = std::function<void(typename TSpline::SegmentIterator it)>())
	{
		unsigned int fixFirstVertices = 0;
		for (auto it = spline.getFirstRelevantSegmentByLast(spline.getSegmentIterator(startTime));it->getKnot() < upToTime;fixFirstVertices++, it++) {
			if(apply) apply(it);
		}
		return fixFirstVertices;
	}
}

	_TEMPLATE
	void _CLASS::extendAndFitSpline(TSpline & spline, KnotGenerator<time_t> & knotGenerator, const std::vector<time_t> & times, const std::vector<point_t> & points, double lambda, unsigned char honorCurrentValuePercentage, FittingBackend fittingBackend, const bool calculateControlVertexOffsets){
		const size_t numPoints = points.size();
		SM_ASSERT_TRUE(Exception, knotGenerator.supportsAppending(), "The knot generator must support appending!");
		SM_ASSERT_GE(Exception, numPoints, 1, "The must be at least one time point pair!");
		SM_ASSERT_EQ(Exception, times.size(), numPoints, "The number of times and the number of points must be equal");
		SM_ASSERT_LE(Exception, honorCurrentValuePercentage, 100, "honorCurrentValuePercentage must be in [0, 100]");

		// first we get the time to which to extend the spline.
		const time_t lastTime = times[times.size() - 1];

		const time_t oldMaxTime = spline.getMaxTime();
		if(lastTime > oldMaxTime){
			knotGenerator.extendBeyondTime(lastTime);
			spline.appendSegments(knotGenerator, -1, nullptr);
		}

		if(honorCurrentValuePercentage == 0 || honorCurrentValuePercentage == 100){
			unsigned int fixFirstVertices;
			if(honorCurrentValuePercentage) {
				if(lastTime > oldMaxTime){
					fixFirstVertices = internal::getNumberOfRelevantControlVertices(spline, times[0], oldMaxTime);
				} else {
					// no new vertices but old should not be touched
					return;
				}
			}
			else{// ignore former values
				fixFirstVertices = 0;
			}
			fitSpline(spline, times, points, lambda, (int)fixFirstVertices, std::function<scalar_t(int i) >(), fittingBackend, calculateControlVertexOffsets);
		}else{
			std::vector<scalar_t> weights;
			scalar_t baseWeight = scalar_t(honorCurrentValuePercentage) / scalar_t(100 - honorCurrentValuePercentage);

			std::vector<time_t> newTimes;
			std::vector<point_t> newPoints;
			const time_t minTime = spline.getMinTime();

			const unsigned int relevantOldVCs = internal::getNumberOfRelevantControlVertices<TSpline>(spline, times[0], oldMaxTime,
				[&](typename TSpline::SegmentIterator it)
				{
					it++;
					time_t t(it->getKnot());
					if(t >= minTime){
						newTimes.push_back(t);
						newPoints.push_back(spline.template getEvaluatorAt<0>(t).eval());
						weights.push_back(baseWeight); //TODO improve: normalize the weight base on the amount of points given per segment.
					}
				}
			);
			//TODO optimize: use better ways to concatenate times and points.
			SM_ASSERT_EQ_DBG(std::runtime_error, newTimes.size(), relevantOldVCs, "BUG in "  << __FILE__ << ":" << __LINE__);
			SM_ASSERT_EQ_DBG(std::runtime_error, newPoints.size(), relevantOldVCs, "BUG in "  << __FILE__ << ":" << __LINE__);

			newTimes.resize(times.size() + relevantOldVCs);
			copy(times.begin(), times.end(), newTimes.begin() + relevantOldVCs);
			newPoints.resize(points.size() + relevantOldVCs);
			copy(points.begin(), points.end(), newPoints.begin() + relevantOldVCs);

			fitSpline(spline, newTimes, newPoints, lambda, 0, [relevantOldVCs, &weights](int i){ return i < (int)relevantOldVCs ? weights[i]: scalar_t(1.0);}, fittingBackend, calculateControlVertexOffsets);
		}
	}

	_TEMPLATE
	void _CLASS::calcFittedControlVertices(TSpline & spline, const KnotIndexResolver<time_t> & knotIndexResolver, const std::vector<time_t> & times, const std::vector<point_t> & points, std::function<scalar_t(int i) > weights, double lambda, int fixNFirstRelevantControlVertices, FittingBackend fittingBackend, const bool calculateControlVertexOffsets){
		switch(fittingBackend){
			case FittingBackend::DENSE:
				calcFittedControlVertices<FittingBackend::DENSE>(spline, knotIndexResolver, times, points, weights, lambda, fixNFirstRelevantControlVertices, calculateControlVertexOffsets);
				break;
			case FittingBackend::SPARSE:
				calcFittedControlVertices<FittingBackend::SPARSE>(spline, knotIndexResolver, times, points, weights, lambda, fixNFirstRelevantControlVertices, calculateControlVertexOffsets);
				break;
		}
	}

	namespace internal {
		template< enum FittingBackend FittingBackend_> struct FittingBackendFunctions {
		};

		template<> struct FittingBackendFunctions<FittingBackend::DENSE> {
			typedef typename FittingBackendTraits<FittingBackend::DENSE>::Matrix Matrix;
			typedef typename FittingBackendTraits<FittingBackend::DENSE>::Vector Vector;

			inline static Matrix createA(unsigned int constraintSize, unsigned int coefficientDim, unsigned int /* D */) {
				return Matrix::Zero(constraintSize, coefficientDim);
			}
			inline static Vector createB(int constraintSize) {
				return Vector::Zero(constraintSize);
			}

			inline static auto blockA(Matrix & A, int row, int col, int D, int size = 1) -> decltype(A.block(0, 0, 0, 0)){
				return A.block(row * D, col * D, size * D, size * D);
			}
			inline static typename Vector::SegmentReturnType segmentB(Vector & b, int row, int D){
				return b.segment(row * D, D);
			}
			template <typename DERIVED, typename DERIVED2>
			inline static void addBlockToBlock(Eigen::MatrixBase<DERIVED> & A, int aRow, int aCol, const Eigen::MatrixBase<DERIVED2> & Q, int qRow, int qCol, int rows, int cols, int D, bool segmentInRowVector){
				rows *= D;
				if(segmentInRowVector){
					cols = 1;
				}
				else{
					cols *= D;
				}
				A.block(aRow * D, aCol * D, rows, cols) += Q.block(qRow * D, qCol * D, rows, cols);
			}

			inline static Eigen::VectorXd solve(Matrix & A, Vector & b){
				return A.ldlt().solve(b);
			}
		};


		template<> struct FittingBackendFunctions<FittingBackend::SPARSE> {
			typedef typename FittingBackendTraits<FittingBackend::SPARSE>::Matrix Matrix;
			typedef typename FittingBackendTraits<FittingBackend::SPARSE>::Vector Vector;

			std::vector<int> rows;
			std::vector<int> cols;

			const static bool allocateBlock = true;

			inline Matrix createA(unsigned int constraintSize, unsigned int coefficientDim, unsigned int D) {
				for(unsigned int i = D; i <= constraintSize; i+=D) rows.push_back(i);
				for(unsigned int i = D; i <= coefficientDim; i+=D) cols.push_back(i);
				return Matrix(rows,cols, true);
			}

			inline Vector createB(int /* constraintSize */) {
				std::vector<int> bcols(1);
				bcols[0] = 1;
				return Vector(rows,bcols, true);
			}

			inline static typename Matrix::SparseMatrixBlock & blockA(Matrix & A, int row, int col, int /* D */, int size = 1){
				SM_ASSERT_EQ(std::runtime_error, 1, size, "This function should does not support size != 1");
				return *A.block(row, col, allocateBlock);
			}
			inline static typename Vector::SparseMatrixBlock & segmentB(Vector & b, int row, int /* D */){
				return *b.block(row, 0, allocateBlock);
			}
			template <typename DERIVED>
			inline static void addBlockToBlock(Matrix & A, int aRow, int aCol, const Eigen::MatrixBase<DERIVED> & Q, int qRow, int qCol, int rows, int cols, int D, bool segmentInRowVector){
				for(int i = 0; i < rows; ++i){
					for(int j = 0; j < cols; ++j){
						*A.block(aRow + i, aCol + j, true) += Q.block((qRow + i) * D, (qCol + j) * D, D, segmentInRowVector ? 1 : D);
					}
				}
			}

			inline static Eigen::VectorXd solve(Matrix & A, Vector & b){
				// solve:
				sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd> solver;
				solver.init();

				Eigen::VectorXd c(A.rows());
				c.setZero();
				Eigen::VectorXd b_dense = b.toDense();
				bool result = solver.solve(A,&c[0],&b_dense[0]);
				if(!result) {
					c.setZero();
					// fallback => use nonsparse solver:
					std::cout << "Fallback to Dense Solver" << std::endl;
					Eigen::MatrixXd Adense = A.toDense();
					c = Adense.ldlt().solve(b_dense);
				}
				return c;
			}
		};
	}

	_TEMPLATE
	template <enum FittingBackend FittingBackend_>
	void _CLASS::calcFittedControlVertices(TSpline & spline, const KnotIndexResolver<time_t> & knotIndexResolver, const std::vector<time_t> & times, const std::vector<point_t> & points, std::function<scalar_t(int i) > weights, double lambda, int fixNFirstRelevantControlVertices, const bool calculateControlVertexOffsets)
	{
		if(calculateControlVertexOffsets){
			//TODO implement : support calculateControlVertexOffsets in addCurveQuadraticIntegralDiagTo and remove this check!
			SM_ASSERT_EQ(Exception, 0.0, lambda, "control vertex offsets aren't supported together with lambda != 0, yet!");
		}

		SM_ASSERT_GE_DBG(Exception, fixNFirstRelevantControlVertices, 0, "fixNFirstRelevantControlVertices must be nonnegative.");
		const time_t splineMaxTime = spline.getMaxTime(), lastTime = times[times.size() - 1];
		const int splineOrder = spline.getSplineOrder();
		const int minKnotIndex = knotIndexResolver.getKnotIndexAtTime(times[0]) - (splineOrder - 1) - (times[0] == splineMaxTime ? 1 : 0); // get minimal relevant knot's index //TODO improve: remove this irregularity
		const int controlVertexIndexOffset = minKnotIndex + fixNFirstRelevantControlVertices;
		const int numToFitControlVertices = knotIndexResolver.getKnotIndexAtTime(lastTime) - controlVertexIndexOffset + 1 - (lastTime == splineMaxTime ? 1 : 0);//TODO improve: remove this irregularity

		if(numToFitControlVertices <= 0){
			SM_THROW_DBG(Exception, "No control vertices are left over to be optimized.")
			return;
		}

		const int capturedCurrentControlVertices = calculateControlVertexOffsets ? fixNFirstRelevantControlVertices + numToFitControlVertices : fixNFirstRelevantControlVertices;
		const typename TSpline::point_t* currentControlVertices[capturedCurrentControlVertices];
		if(capturedCurrentControlVertices){
			auto it = spline.getFirstRelevantSegmentByLast(spline.getSegmentIterator(times[0]));
			for(int i = 0; i < capturedCurrentControlVertices; ++i){
				currentControlVertices[i] = &it->getControlVertex();
				++it;
			}
		}

		const size_t numPoints = points.size();

		// What is the vector coefficient dimension
		const size_t D = spline.getPointSize();

		// Now we have to solve an Ax = b linear system to determine the correct coefficient vectors.
		size_t coefficientDim = (size_t) numToFitControlVertices * D;

		const int constraintSize = numPoints * D;

		internal::FittingBackendFunctions<FittingBackend_> backend;

		auto A = backend.createA(constraintSize, coefficientDim, D);
		auto b = backend.createB(constraintSize);

		int brow = 0;

		// Add the position constraints.
		for(size_t i = 0; i < numPoints; i++)
		{
			time_t time = times[i];
			scalar_t weight = weights? weights(i) : scalar_t(1.0);
			int knotIndex = knotIndexResolver.getKnotIndexAtTime(time) - controlVertexIndexOffset;
			//TODO optimize : a uniform time spline evaluator could be much faster here!
			typename TSpline::SplineOrderVector bi = spline.template getEvaluatorAt<1>(times[i]).getLocalBi();

			if(weight != scalar_t(1.0)){
				bi *= weight;
			}

			knotIndex -= splineOrder - 1; // get first relevant knot's index
			if(time == splineMaxTime){ //TODO improve: remove this irregularity
				knotIndex --;
			}

			for(int j = 0; j < splineOrder; j ++){
				const int col = knotIndex + j;
				if(col >= 0){ // this control vertex is not fixed
					if(bi[j] != 0.0)
						backend.blockA(A, brow, col, D).diagonal().setConstant(bi[j]);
				}
				if(col < 0 || calculateControlVertexOffsets){
					const int vertexIndex = fixNFirstRelevantControlVertices + col;
					SM_ASSERT_GE_DBG(std::runtime_error, vertexIndex, 0, "BUG in BSplineFitter");
					SM_ASSERT_LT_DBG(std::runtime_error, vertexIndex, capturedCurrentControlVertices, "BUG in BSplineFitter");
					backend.segmentB(b, brow, D) -= (*currentControlVertices[vertexIndex]) * bi[j];
				}
			}

			if(knotIndex >= 0 && ! calculateControlVertexOffsets)
				backend.segmentB(b, brow, D) = points[i] * weight;
			else{
				backend.segmentB(b, brow, D) += points[i] * weight;
			}
			++brow;
		}

		b = A.transpose() * b;
		A = A.transpose() * A;

		if(lambda != 0.0){
			// Add the motion constraint.
			point_t W = point_t::Constant(D, lambda);
			auto start = spline.getSegmentIterator(times[0]);
			auto end = spline.getSegmentIterator(lastTime);
			const int moveCount = fixNFirstRelevantControlVertices - (splineOrder - 1);
			const int movedCount = moveIterator(start, moveCount < 0 ? spline.begin() : end, moveCount);
			const int moveEndCount = splineOrder;
			moveIterator(end, spline.end(), moveEndCount);
			if(moveCount - movedCount <= 0){ // did we not reach end?
				addCurveQuadraticIntegralDiagTo<FittingBackend_>(spline, start, end, movedCount, W, 2, A, b);
			}
		}

		// Solve for the coefficient vector.
		Eigen::VectorXd c = backend.solve(A, b);

		if(calculateControlVertexOffsets){
			const auto & manifold = spline.getManifold();
			spline.manipulateControlVertices([&c, D, &manifold](int i, point_t & v) {
					v += c.block(i * D, 0, D, 1);
					manifold.projectIntoManifold(v);
				}, numToFitControlVertices, times[0], fixNFirstRelevantControlVertices);
		}
		else{
			spline.setControlVertices(c, times[0], fixNFirstRelevantControlVertices);
		}
	}

	_TEMPLATE
	template <enum FittingBackend FittingBackend_>
	void _CLASS::addCurveQuadraticIntegralDiagTo(const TSpline & spline, typename TSpline::SegmentConstIterator start, typename TSpline::SegmentConstIterator end, int startIndex, const point_t & Wdiag, int derivativeOrder, typename internal::FittingBackendTraits<FittingBackend_>::Matrix & toMatrix, typename internal::FittingBackendTraits<FittingBackend_>::Vector & toB)
	{
		SM_ASSERT_EQ(Exception,Wdiag.rows(), (int)spline.getPointSize(), "Wdiag must be of control point size");
		SM_ASSERT_EQ(Exception, toMatrix.rows(), toB.rows(), "Wdiag must be of control point size");

		typedef internal::FittingBackendFunctions<FittingBackend_> Backend;

		const int D = spline.getPointSize();
		const int blocksInQ = spline.getSplineOrder();
		SM_ASSERT_EQ(Exception,Wdiag.rows(), D, "Wdiag must be of control point size");

		const auto qiSize = spline.getSplineOrder() * spline.getPointSize();
		typedef decltype(qiSize) QiSize;
		typedef Eigen::Matrix<double, QiSize::VALUE, QiSize::VALUE> Q_T;
		Q_T Q((int)qiSize, (int)qiSize);

		int blockRow = startIndex;
		const int blockRows = toMatrix.rows() / D;
		for(typename TSpline::SegmentConstIterator sIt = start; sIt != end; sIt++)
		{
			if(FittingBackend_ == FittingBackend::DENSE && blockRow >= 0 && blockRow <= blockRows - blocksInQ){ // all splineOrder many diagonal blocks are contained in the toMatrix
				addOrSetSegmentQuadraticIntegralDiag(spline, Wdiag, sIt, derivativeOrder, Backend::blockA(toMatrix, blockRow, blockRow, D, blocksInQ), true);
			} else { // we have to assign sub blocks individually
				addOrSetSegmentQuadraticIntegralDiag<Q_T &>(spline, Wdiag, sIt, derivativeOrder, Q, false);
				/*
				 * Q and toMatrix (=:M) (both square) have shifted block index space. i_Q + brow = i_M.
				 * Lets x denote the coefficients of all non fixed control vertices
				 * first we need the overlapping region:
				 * its left most index in M is : max(0, 0 + brow).
				 * its right most index in M is : min(cols(M) - 1, cols(Q) - 1 + brow).
				 */
				const int overlappingBlocksLeftIndex = std::max(0, blockRow);
				const int overlappingBlocksRightIndex = std::min(blockRows, blocksInQ + blockRow) - 1;
				const int numOverlappingBlocks = overlappingBlocksRightIndex - overlappingBlocksLeftIndex + 1;
				if(numOverlappingBlocks > 0){
					Backend::addBlockToBlock(toMatrix, overlappingBlocksLeftIndex, overlappingBlocksLeftIndex, Q, overlappingBlocksLeftIndex - blockRow, overlappingBlocksLeftIndex - blockRow, numOverlappingBlocks, numOverlappingBlocks, D, false);
					if(numOverlappingBlocks != blocksInQ){
						/*
						 * Now we have to subtract the effect of the fixed control vertices on the acceleration term.
						 *
						 * [Q_l  Q_x  Q_r]  * [ c_l; x; c_r] = b_x
						 * <=> Q_x * x  = b_x - Q_l * c_l - Q_r * c_r
						 *
						 * We start with c_r because sIt corresponds to its bottom at the moment.
						 */
						auto constantVertexIt = sIt;
						const int Q_r_LeftColumnIndex = overlappingBlocksRightIndex - blockRow + 1;
						const int Q_l_RightColumnIndex = - blockRow - 1;
						for(int i = blocksInQ-1; i >= 0; i--) {
							if(i >= Q_r_LeftColumnIndex || i <= Q_l_RightColumnIndex){ // then vertex is fixed and we have to subtract its effect on the right hand side
								Backend::addBlockToBlock(toB, overlappingBlocksLeftIndex, 0, (-Q.block((overlappingBlocksLeftIndex - blockRow) * D, i * D, numOverlappingBlocks * D, D) * constantVertexIt->getControlVertex()).eval(), 0, 0, numOverlappingBlocks, 1, D, true);
							}
							constantVertexIt--;
						}
					}
				}
			}
			blockRow ++;
		}
	}

	_TEMPLATE
	template<typename M_T>
	inline void _CLASS::addOrSetSegmentQuadraticIntegralDiag(const TSpline & spline, const point_t & Wdiag, typename TSpline::SegmentConstIterator segmentIt, int derivativeOrder, M_T toMatrix, bool add)
	{
		const int D = spline.getPointSize();
		const int splineOrder = spline.getSplineOrder();
		SM_ASSERT_GE_LT(Exception, segmentIt.getKnot(), spline.getMinTime(), spline.getMaxTime(), "Out of range");
		SM_ASSERT_EQ(Exception, Wdiag.rows(), D, "Wdiag must be of control point size");

		typename TSpline::SplineOrderSquareMatrix Dm(splineOrder, splineOrder);
		Dm.setZero(splineOrder, splineOrder);
		spline.computeDiiInto(segmentIt, Dm);
		typename TSpline::SplineOrderSquareMatrix V(splineOrder, splineOrder);
		V.setZero(splineOrder, splineOrder);
		spline.computeViInto(segmentIt, V);

		// Calculate the appropriate derivative version of V
		// using the matrix multiplication version of the derivative.
		for(int i = 0; i < derivativeOrder; i++)
		{
			V = (Dm.transpose() * V * Dm).eval();
		}

		const auto splineOrderTimesPointSize = spline.getSplineOrder() * spline.getPointSize();
		typedef decltype(splineOrderTimesPointSize) SplineOrderTimesPointSize;
		typedef Eigen::Matrix<double, SplineOrderTimesPointSize::VALUE, SplineOrderTimesPointSize::VALUE> MType;

		MType WV((int)splineOrderTimesPointSize, (int)splineOrderTimesPointSize);

		WV.setZero(splineOrderTimesPointSize, splineOrderTimesPointSize);

		for(int d = 0; d < D; d++)
		{
			WV.block(splineOrder*d, splineOrder*d, splineOrder, splineOrder) = Wdiag(d) * V;
		}

		MType M((int)splineOrderTimesPointSize, (int)splineOrderTimesPointSize);
		computeMiInto(spline, segmentIt, M);

		if(add) toMatrix += M.transpose() * WV * M;
		else toMatrix = M.transpose() * WV * M;
	}

	_TEMPLATE
	template <typename M_T>
	void _CLASS::computeBijInto(const TSpline & spline, const typename TSpline::SegmentConstIterator & segmentIndex, int columnIndex, M_T B)
	{
		const int D = spline.getPointSize();
		const int splineOrder = spline.getSplineOrder();

		for(int i = 0; i < D; i++)
		{
			B.block(i*splineOrder,i,splineOrder,1) = segmentIndex->getBasisMatrix().col(columnIndex);
		}
	}

	_TEMPLATE
	template <typename M_T>
	void _CLASS::computeMiInto(const TSpline & spline, const typename TSpline::SegmentConstIterator & segmentIndex, M_T & M)
	{
		const int D = spline.getPointSize();
		const int splineOrder = spline.getSplineOrder();
		const int splineOrderTimesPointSize = splineOrder * D;
		M.setZero();

		for(int j = 0; j < splineOrder; j++)
		{
			computeBijInto(spline, segmentIndex, j, M.block(0, j*D, splineOrderTimesPointSize, D));
		}
	}
}

#undef _TEMPLATE
#undef _CLASS


#endif /* BSPLINEFITTERIMPL_HPP_ */

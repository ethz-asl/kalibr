/*
 * DiffManifoldBSpline.hpp
 *
 *  Created on: Apr 23, 2012
 *      Author: Hannes Sommer
 */

#ifndef RIEMANNIANBSPLINE_HPP_
#define RIEMANNIANBSPLINE_HPP_


#include "ExternalIncludes.hpp"
#include <gtest/gtest_prod.h>
#include "DynamicOrTemplateInt.hpp"
#include "SimpleTypeTimePolicy.hpp"
#include "manifolds/DiffManifold.hpp"
#include "KnotArithmetics.hpp"


namespace bsplines {
	typedef SimpleTypeTimePolicy<double> DefaultTimePolicy;

	template <typename TDiffManifoldBSplineConfiguration, typename TConfigurationDerived = TDiffManifoldBSplineConfiguration>
	class DiffManifoldBSpline{
	};

	template <typename TDiffManifoldConfiguration, int ISplineOrder = Eigen::Dynamic, typename TTimePolicy = DefaultTimePolicy>
	struct DiffManifoldBSplineConfiguration : public TDiffManifoldConfiguration {
	public:
		typedef DiffManifoldBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy> Conf;

		typedef eigenTools::DynamicOrTemplateInt<ISplineOrder> SplineOrder;

		typedef DiffManifoldBSpline<Conf> BSpline;
		typedef TDiffManifoldConfiguration ManifoldConf;
		typedef TTimePolicy TimePolicy;

		DiffManifoldBSplineConfiguration(TDiffManifoldConfiguration manifoldConfiguration, int splineOrder = ISplineOrder) : TDiffManifoldConfiguration(manifoldConfiguration), _splineOrder(splineOrder) {}
#if __cplusplus >= 201103L
		DiffManifoldBSplineConfiguration(const DiffManifoldBSplineConfiguration&) = default;
		DiffManifoldBSplineConfiguration() = default;
#endif

		const SplineOrder getSplineOrder() const { return _splineOrder; }

	private:
		SplineOrder _splineOrder;
	};


	namespace internal {
		namespace state {
			enum SplineState { CONSTRUCTING, EVALUABLE };
		}

		template <typename TDiffManifoldBSplineConfiguration>
		struct DiffManifoldBSplineTraits {
			enum { NeedsCumulativeBasisMatrices = false };
		};

		template <typename TDiffManifoldBSplineConfiguration>
		struct SegmentData {
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			typedef typename TDiffManifoldBSplineConfiguration::Manifold Manifold;
			typedef typename Manifold::point_t point_t;
			typedef typename TDiffManifoldBSplineConfiguration::TimePolicy::time_t time_t;
			typedef Eigen::Matrix<typename Manifold::scalar_t, TDiffManifoldBSplineConfiguration::SplineOrder::VALUE, TDiffManifoldBSplineConfiguration::SplineOrder::VALUE> basis_matrix_t;

		public:
			inline point_t & getControlVertex() { return _point; }
			inline basis_matrix_t & getBasisMatrix() { return _basisMatrix; }

			inline void setControlVertex(const point_t & point) { _point = point; }

			inline const point_t & getControlVertex() const { return _point; }
			inline const basis_matrix_t & getBasisMatrix() const { return _basisMatrix; }
			inline time_t getKnot() const { return _t; }
			inline time_t getTime() const { return getKnot(); }

          inline SegmentData(const TDiffManifoldBSplineConfiguration & conf, const typename TDiffManifoldBSplineConfiguration::Manifold & /* manifold */, const time_t & t, const point_t & point) : _point(point), _basisMatrix((int)conf.getSplineOrder(), (int)conf.getSplineOrder()), _t(t) {}
		protected:
			point_t _point;
			basis_matrix_t _basisMatrix;
			time_t _t;
		};

		template <typename TDiffManifoldBSplineConfiguration>
		struct SegmentMap : public std::map<typename TDiffManifoldBSplineConfiguration::TimePolicy::time_t, SegmentData<TDiffManifoldBSplineConfiguration> > {
			typedef SegmentData<TDiffManifoldBSplineConfiguration> segment_data_t;
			typedef std::map<typename TDiffManifoldBSplineConfiguration::TimePolicy::time_t, segment_data_t> parent_t;
			typedef typename segment_data_t::point_t point_t;
			typedef typename segment_data_t::basis_matrix_t basis_matrix_t;
			typedef typename TDiffManifoldBSplineConfiguration::TimePolicy::time_t time_t;
			typedef typename std::map<time_t, SegmentData<TDiffManifoldBSplineConfiguration> > segment_map_t;
			typedef typename segment_map_t::iterator SegmentMapIterator;
			typedef typename segment_map_t::const_iterator SegmentMapConstIterator;

			template<typename SEGMENTDATA, typename BASE>
			struct SegmentIteratorT : public BASE {
				SegmentIteratorT(){}
				SegmentIteratorT(const BASE & it) : BASE(it) {}
				inline const time_t & getKnot() const { return (* static_cast<const BASE *>(this))->first; }
				inline const time_t & getTime() const { return getKnot(); }
				inline SEGMENTDATA & operator *() const { return (* static_cast<const BASE *>(this))->second; }
				inline SEGMENTDATA * operator ->() const { return &(* static_cast<const BASE *>(this))->second; }
			};

			struct SegmentConstIterator : public SegmentIteratorT<const segment_data_t, SegmentMapConstIterator> {
				SegmentConstIterator(){}
				SegmentConstIterator(const SegmentMapIterator & it) : SegmentIteratorT<const segment_data_t, SegmentMapConstIterator>(it) {}
				SegmentConstIterator(const SegmentMapConstIterator & it) : SegmentIteratorT<const segment_data_t, SegmentMapConstIterator>(it) {}
			};

			struct SegmentIterator : public SegmentIteratorT<segment_data_t, SegmentMapIterator> {
				SegmentIterator(){}
				SegmentIterator(const SegmentMapIterator & it) : SegmentIteratorT<segment_data_t, SegmentMapIterator>(it) {}
			};
		};

		template <typename Spline, int IMaximalDerivativeOrder>
		struct get_evaluator {
			typedef typename Spline::template Evaluator<IMaximalDerivativeOrder> type;
		};

		template <typename Spline>
		struct AssertInitializedSpline{
			AssertInitializedSpline(const Spline & s){
				s.assertEvaluable();
			}
		};
	}

	template <typename TSpline> class BSplineFitter;

	template <typename TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy, typename TConfigurationDerived>
	class DiffManifoldBSpline<DiffManifoldBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TConfigurationDerived> {
	public:
		SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

		typedef TConfigurationDerived configuration_t;
		typedef typename configuration_t::BSpline spline_t;
		typedef typename configuration_t::Manifold manifold_t;
		typedef typename manifold_t::scalar_t scalar_t;
		typedef typename manifold_t::point_t point_t;
		typedef typename manifold_t::tangent_vector_t tangent_vector_t;
		typedef typename manifold_t::dmatrix_t dmatrix_t;

		typedef typename configuration_t::TimePolicy TimePolicy;
		typedef typename TimePolicy::time_t time_t;
		typedef typename TimePolicy::duration_t duration_t;

		enum {
			SplineOrder = configuration_t::SplineOrder::VALUE,
			Dimension = manifold_t::Dimension,
			PointSize = manifold_t::PointSize,
			NeedsCumulativeBasisMatrices = internal::DiffManifoldBSplineTraits<TConfigurationDerived>::NeedsCumulativeBasisMatrices
		};

		typedef Eigen::Matrix<scalar_t, manifold_t::PointSize, multiplyEigenSize(Dimension, SplineOrder) > full_jacobian_t;
		typedef Eigen::Matrix<scalar_t, SplineOrder, 1> SplineOrderVector;
		typedef Eigen::Matrix<scalar_t, SplineOrder, SplineOrder> SplineOrderSquareMatrix;

#if __cplusplus >= 201103L
	public:
		// Deprecated: Only for compatibility with the pre C++11 type maps, which are template aliases when c++11 is available.
		typedef TConfigurationDerived CONF;
		typedef typename CONF::BSpline TYPE;
		typedef typename CONF::BSpline BSpline;
#endif

	public:
		typedef internal::SegmentMap<configuration_t> segment_map_t;
		typedef typename segment_map_t::segment_data_t segment_data_t;

	protected:
		typedef typename segment_map_t::SegmentMapIterator SegmentMapIterator;
		typedef typename segment_map_t::SegmentMapConstIterator SegmentMapConstIterator;

	public:
		typedef typename segment_map_t::SegmentIterator SegmentIterator;
		typedef typename segment_map_t::SegmentConstIterator SegmentConstIterator;


		DiffManifoldBSpline(const configuration_t & configuration) : _configuration(configuration), _state(internal::state::CONSTRUCTING), _manifold(configuration), _segments(new segment_map_t()) {}

		DiffManifoldBSpline(const DiffManifoldBSpline & other) : _configuration(other._configuration), _state(other._state), _manifold(other._manifold), _segments(new segment_map_t(*other._segments)) { if(isInitialized()) initIterators();}
		DiffManifoldBSpline(DiffManifoldBSpline && /* other */) = default;

		DiffManifoldBSpline & operator=(const DiffManifoldBSpline & other);
		DiffManifoldBSpline & operator=(DiffManifoldBSpline && other) = default;

		inline typename configuration_t::SplineOrder getSplineOrder() const { return _configuration.getSplineOrder(); }
		inline typename configuration_t::Dimension getDimension() const { return _configuration.getDimension(); }
		inline typename configuration_t::PointSize getPointSize() const { return _configuration.getPointSize(); }

		inline const manifold_t & getManifold() const;

		inline const SegmentConstIterator getAbsoluteBegin() const;

		inline const SegmentIterator getAbsoluteBegin();

		inline const SegmentConstIterator getAbsoluteEnd() const;

		inline const SegmentIterator getAbsoluteEnd();

		int getNumKnotsRequired(int validSegments) const;
		int getMinimumKnotsRequired() const { return getNumKnotsRequired(1); };
		int getNumControlVerticesRequired(int validSegments) const;

		inline size_t getAbsoluteNumberOfSegments() const;

		inline void addKnot(const time_t & time);

		void addControlVertex(const time_t & time, const point_t & point);

		void init();

		void addKnots(const Eigen::VectorXd & knotTimes);
		void addKnotsAndControlVertices(const Eigen::VectorXd & knotTimes, const Eigen::MatrixXd & controlVertices);

		void initWithKnots(const Eigen::VectorXd & knotTimes);
		void initWithKnotsAndControlVertices(const Eigen::VectorXd & knotTimes, const Eigen::MatrixXd & controlVertices);

		/**
		 * Iterates a callback function over the first manipulateTheFirstNControlVertices control vertices relevant for this slice (after moving from begin() by offset).
		 * @param controlVertexManipulator This callback function manipulates the control vertex having the index given by the first argument.
		 * @param manipulateTheFirstNControlVertices That much control vertices will be given to the manipulator starting from the first relevant control vertex.
		 * @param offset
		 */
		void manipulateControlVertices(std::function<void (int index, point_t & controlVertex)> controlVertexManipulator, int manipulateTheFirstNControlVertices, int offset = 0);
		/**
		 * Same as above but with an additional starting time. The control vertices are changed starting at the one reached after moving offset from the first relevant for startingAtFirstRelevantControlVertexForThisTime time.
		 * @param controlVertexManipulator
		 * @param manipulateTheFirstNControlVertices
		 * @param startingAtFirstRelevantControlVertexForThisTime
		 * @param offset
		 */
		void manipulateControlVertices(std::function<void (int index, point_t & controlVertex)> controlVertexManipulator, int manipulateTheFirstNControlVertices, const time_t startingAtFirstRelevantControlVertexForThisTime, int offset = 0);

		/**
		 * Sets as much control vertices of the spline as contained in the controlVertices matrix. Starting at the first relevant control vertex.
		 * It it or its transposed must be (dim * n) x 1 or dim x n, where n is the number of contained control vertices.
		 * @param controlVertices
		 */
		void setControlVertices(const Eigen::MatrixXd & controlVertices);

		/**
		 * Same as above but with an additional starting time. The control vertices are changed starting at the the one reached after moving offset from the first relevant for startingAtFirstRelevantControlVertexForThisTime time.
		 * @param controlVertices
		 * @param startingAtFirstRelevantControlVertexForThisTime
		 * @param offset
		 */
		void setControlVertices(const Eigen::MatrixXd & controlVertices, const time_t startingAtFirstRelevantControlVertexForThisTime, int offset = 0);

		void initConstantSpline(KnotGenerator<time_t> & knotGenerator, const point_t & constant);
		void initConstantUniformSpline(const time_t & tMin, const time_t & tMax, int numSegments, const point_t & constant);
		inline DeltaUniformKnotGenerator<TimePolicy> initConstantUniformSplineWithKnotDelta(const time_t & tMin, const time_t & beyondThisTime, const duration_t knotDelta, const point_t & constant);

		/**
		 * Appends numSegments knots to the spline, such that after that the last 2 + numSegments knots in the spline are uniformly spaced.
		 * These knots are all in the future (i.e. greater or equal to getMaxTime() result before the knot was appended.
		 * The new value of getMaxTime() will be the knot position numSegmetns after the one with the former getMaxTime value.
		 *
		 * This is only allowed on slices up to the splines end. Then it extends the slice as well.
		 *
		 * @param numSegments the number of segments to be added or negative to activate beyondThisTime as end condition.
		 * @param value the value for the newly relevant control vertices may be nullptr. Then the manifold's default point is used.
		 * @param beyondThisTime If numSegments < 0 then segments are added until getMaxTime is greater then beyondThisTime.
		 * @return returns the new maximal time.
		 */
		time_t appendSegmentsUniformly(unsigned int numSegments = 1, const point_t * value = nullptr, const time_t beyondThisTime = time_t());

		/**
		 * Appends numSegments knots to the spline, according to given KnotGenerator.
		 *
		 * This is only allowed on slices up to the splines end. Then it extends the slice as well.
		 *
		 * @param knotGenerator The knot generator determines the amount (if numSegments < 0) and time stamps of the appended spline knots.
		 * @param numSegments the number of segments to be added or negative to activate the knot generator's end condition.
		 * @param value the value for the newly relevant control vertices may be nullptr. Then the manifold's default point is used.
		 * @return returns the new maximal time.
		 */
		time_t appendSegments(KnotGenerator<time_t> & knotGenerator, int numSegments = 1, const point_t * value = nullptr);

		/**
		 * Get the number of valid time segments in the slice. Valid is a time segment, in whose interior spline order many basis functions are nonzero.
		 * This requires at least spline order -1 many knots before its left hand knot and after its right hand knot.
		 * @return the number of valid time segments
		 */
		inline int getNumValidTimeSegments() const;

		/**
		 * Get the number of relevant control vertices for this slice. Relevant is a control vertex if its value affects the splines value in the valid time segments of the slice.
		 * @return the number relevant control vertices
		 */
		inline int getNumControlVertices() const;

		/**
		 * get the number of knots in this slice
		 * @return
		 */
		inline size_t getNumKnots() const;

		inline bool isInitialized() const { return _state == internal::state::SplineState::EVALUABLE; }

		inline time_t getMinTime() const;
		inline time_t getMaxTime() const;

		inline std::pair<time_t, time_t> getTimeInterval() const { return std::pair<time_t, time_t> (getMinTime(), getMaxTime()); };

		inline SegmentIterator firstRelevantSegment();
		inline SegmentConstIterator firstRelevantSegment() const;
		inline SegmentIterator begin();
		inline SegmentConstIterator begin() const;
		inline SegmentIterator end();
		inline SegmentConstIterator end() const;

		void computeDiiInto(const SegmentConstIterator & segmentIt, SplineOrderSquareMatrix & D) const;

		void computeViInto(SegmentConstIterator segmentIt, SplineOrderSquareMatrix & V) const;

		/**
		 * find the greatest knot less or equal to t
		 * @param t time
		 * @return knot iterator
		 */
		inline SegmentIterator getSegmentIterator(const time_t & t);
		inline SegmentConstIterator getSegmentIterator(const time_t & t) const;

		time_t getMinimalDistanceToNeighborKnots(const time_t & t) const;

		inline SegmentConstIterator getFirstRelevantSegmentByLast(const SegmentConstIterator & first) const;
		inline SegmentIterator getFirstRelevantSegmentByLast(const SegmentIterator & first);

		inline void setLocalCoefficientVector(const time_t & t, const Eigen::VectorXd & coefficients, int pointSize);

		inline void getLocalCoefficientVector(const time_t & t, Eigen::VectorXd & coefficients, int pointSize);


		template <typename TValue, typename TFunctor>
		TValue evalFunctorIntegralNumerically(const time_t & t1, const time_t & t2, const TFunctor & f, int numberOfPoints = 100) const;

		template <typename TValue, typename TFunctor>
		inline TValue evalFunctorIntegral(const time_t & t1, const time_t & t2, const TFunctor & f) const { return evalFunctorIntegralNumerically<TValue, TFunctor>(t1, t2, f); }

		point_t evalIntegralNumerically(const time_t & t1, const time_t & t2, int numberOfPoints = 100) const;
		inline point_t evalIntegral(const time_t & t1, const time_t & t2) const { return evalIntegralNumerically(t1, t2); }
		inline point_t evalI(const time_t & t1, const time_t & t2) const { return evalIntegral(t1, t2); }

		template<int IMaximalDerivativeOrder>
		class Evaluator : private internal::AssertInitializedSpline<spline_t> {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			Evaluator(const spline_t & spline, const time_t & t);
			SegmentConstIterator getFirstRelevantSegmentIterator() const;
			inline SegmentConstIterator begin() const { return getFirstRelevantSegmentIterator(); }

			SegmentConstIterator getLastRelevantSegmentIterator() const;

			SegmentConstIterator end() const;

			inline time_t getKnot() const { return _ti.getKnot(); }
			inline time_t getTime() const { return _t; }

			inline point_t eval() const;

			inline point_t evalD(int derivativeOrder) const;

			inline point_t evalGeneric() const;

			inline point_t evalDGeneric(int derivativeOrder) const;

			const duration_t getSegmentLength() const;

			const duration_t getPositionInSegment() const;

			double getRelativePositionInSegment() const;

			const spline_t& getSpline() const;

		protected:
			enum { NumberOfPreparedDerivatives = (IMaximalDerivativeOrder <= SplineOrder ? IMaximalDerivativeOrder : SplineOrder) + 1};
			SplineOrderVector _localBi[NumberOfPreparedDerivatives];
			//TODO optimize : cache results
			mutable SplineOrderVector _tmp;

			const spline_t & _spline;
			const time_t _t;
			const SegmentConstIterator _ti, _firstRelevantControlVertexIt, _end;
			const duration_t _segmentLength;
			const duration_t _positionInSegment;
			const double _relativePositionInSegment;

			inline void computeLocalViInto(SplineOrderVector & localVi) const;
			inline const SplineOrderVector & getLocalBi(int derivativeOrder = 0) const;
			inline const SplineOrderVector & getLocalCumulativeBi(int derivativeOrder = 0) const;
			inline void computeLocalBiInto(SplineOrderVector & ret, int derivativeOrder = 0) const;
			inline void computeLocalCumulativeBiInto(SplineOrderVector & ret, int derivativeOrder = 0) const;
			inline void computeLocalCumulativeBiInto(const SplineOrderVector & localBi, SplineOrderVector & ret, int derivativeOrder = 0, bool argumentsAreTheSame = false) const;
			inline int dmul(int i, int derivativeOrder) const;
			inline void computeUInto(int derivativeOrder, SplineOrderVector & u) const;
			inline void computeVInto(SplineOrderVector & v) const;

			template<bool BCumulative> inline void computeLocalBiIntoT(SplineOrderVector& ret, int derivativeOrder) const;
			template<bool BCumulative> inline const SplineOrderVector& getLocalBiT(int derivativeOrder = 0) const;

			//friends:
			template<typename TSpline> friend class BSplineFitter;
			FRIEND_TEST(DiffManifoldBSplineTestSuite, testGetBi);
			FRIEND_TEST(DiffManifoldBSplineTestSuite, testInitialization);
			FRIEND_TEST(DiffManifoldBSplineTestSuite, testAddingSegments);
			FRIEND_TEST(UnitQuaternionBSplineTestSuite, evalAngularVelocityAndAcceleration);
		};

		template<int IMaximalDerivativeOrder>
		inline typename internal::get_evaluator<spline_t, IMaximalDerivativeOrder>::type getEvaluatorAt(const time_t & t) const;

	protected:
		typedef typename knot_arithmetics::UniformTimeCalculator<TimePolicy> UniformTimeCalculator;

		TConfigurationDerived _configuration;
		enum internal::state::SplineState _state;
		manifold_t _manifold;
		std::shared_ptr<segment_map_t> _segments;
		SegmentIterator _begin, _end, _firstRelevantSegment;

		inline const spline_t & getDerived() const { return *static_cast<const spline_t *>(this); }
		inline spline_t & getDerived() { return *static_cast<spline_t *>(this); }

		inline segment_map_t & getSegmentMap();
		inline const segment_map_t & getSegmentMap() const;

		inline internal::state::SplineState getState() const { return _state; }
		inline void setState(internal::state::SplineState state) { _state = state; }

		inline void assertEvaluable() const;
		inline void assertConstructing() const;

		/**
		 * compute duration
		 * @param from
		 * @param till
		 * @return till - from
		 */
		inline static duration_t computeDuration(time_t from, time_t till);

		/**
		 * divide durations
		 * @param a
		 * @param b
		 * @return a/b
		 */
		inline static double divideDurations(duration_t a, duration_t b);

		inline static double getDurationAsDouble(duration_t d);

		void initializeBasisMatrices();
		void initializeBasisMatrices(SegmentMapIterator startKnotIt, SegmentMapIterator startBasisMatrixIt);

		segment_data_t createSegmentData(const time_t & time, const point_t & point) const;

		Eigen::MatrixXd M(int k, const std::deque<time_t> & knots);

		static double d_0(int k, int i, int j, const std::deque<time_t> & knots);
		static double d_1(int k, int i, int j, const std::deque<time_t> & knots);

		inline duration_t computeSegmentLength(SegmentMapConstIterator segmentIt) const;
		inline static void computeLocalViInto(const SplineOrderVector & v, SplineOrderVector& localVi, const SegmentMapConstIterator & it);

		//friends:
		template<int IMaximalDerivativeOrder> friend class Evaluator;
		friend class internal::AssertInitializedSpline<spline_t>;
		template<typename TSpline> friend class BSplineFitter;
	private:
		void initIterators();
	};
}

#include "implementation/DiffManifoldBSplineImpl.hpp"

#endif /* RIEMANNIANBSPLINE_HPP_ */

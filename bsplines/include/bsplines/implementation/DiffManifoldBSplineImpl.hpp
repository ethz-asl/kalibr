/*
 * DiffManifoldBSpline.hpp
 *
 *  Created on: Apr 23, 2012
 *      Author: Hannes Sommer
 */

#include "DiffManifoldBSplineTools.hpp"
#include "../NumericIntegrator.hpp"

namespace bsplines {

#define _TEMPLATE template <class TDiffManifoldConfiguration, int ISplineOrder, typename TTimePolicy, typename TConfigurationDerived>
#define _CLASS DiffManifoldBSpline<DiffManifoldBSplineConfiguration<TDiffManifoldConfiguration, ISplineOrder, TTimePolicy>, TConfigurationDerived>

	_TEMPLATE
	_CLASS & _CLASS::operator=(const DiffManifoldBSpline & other){
		_state = other._state;
		_configuration = other._configuration;
		_manifold = other._manifold;
		_segments = std::shared_ptr<segment_map_t>(new segment_map_t(*other._segments));
		if(isInitialized()){
			initIterators();
		}
		return *this;
	}

	_TEMPLATE
	inline
	const typename _CLASS::manifold_t & _CLASS::getManifold() const {
		return _manifold;
	}

	_TEMPLATE
	inline typename _CLASS::segment_map_t & _CLASS::getSegmentMap(){
		return * _segments;
	}

	_TEMPLATE
	inline const typename _CLASS::segment_map_t & _CLASS::getSegmentMap() const {
		return * _segments;
	}

	_TEMPLATE
	inline const typename _CLASS::SegmentConstIterator _CLASS::getAbsoluteBegin() const {
		return _segments->begin();
	}

	_TEMPLATE
	inline const typename _CLASS::SegmentIterator _CLASS::getAbsoluteBegin() {
		return _segments->begin();
	}

	_TEMPLATE
	inline const typename _CLASS::SegmentConstIterator _CLASS::getAbsoluteEnd() const {
		return _segments->end();
	}

	_TEMPLATE
	inline const typename _CLASS::SegmentIterator _CLASS::getAbsoluteEnd() {
		return _segments->end();
	}


	_TEMPLATE
	inline size_t _CLASS::getAbsoluteNumberOfSegments() const {
		return _segments->size();
	}


	_TEMPLATE
	inline typename _CLASS::segment_data_t _CLASS::createSegmentData(const time_t & time, const point_t & p) const {
		return segment_data_t(_configuration, _manifold, time, p);
	}

	_TEMPLATE
	inline void _CLASS::addKnot(const time_t & time){
		_segments->insert(typename segment_map_t::value_type(time, createSegmentData(time, getManifold().getDefaultPoint())));
	}

	_TEMPLATE
	void _CLASS::addControlVertex(const time_t & time, const point_t & point){
		SM_ASSERT_TRUE(Exception, getManifold().isInManifold(point), "All control points must lay in the manifold but at least control point "<< point << " does not!");
		_segments->insert(typename segment_map_t::value_type(time, createSegmentData(time, point)));
	}

	_TEMPLATE
	inline
	void _CLASS::assertEvaluable() const {
		SM_ASSERT_EQ(Exception, (int)internal::state::EVALUABLE, (int)getState(), "This spline is currently not evaluable. Did you call init() after initialization or last knot manipulations?");
	}

	_TEMPLATE
	inline
	void _CLASS::assertConstructing() const {
		SM_ASSERT_EQ(Exception, (int)internal::state::CONSTRUCTING, (int)getState(), "This spline is currently in constructing state. Did you call init() twice without intermediate knot manipulations?");
	}

	_TEMPLATE
	void _CLASS::init(){
		assertConstructing();

		SM_ASSERT_GE(Exception, getNumValidTimeSegments(), 1, "There must be at least one segment");

		initIterators();

		initializeBasisMatrices();

		getDerived().setState(internal::state::EVALUABLE);
	}


	_TEMPLATE
	void _CLASS::initIterators() {
		_firstRelevantSegment = _segments->begin();
		internal::moveIterator(_begin = _segments->begin(), SegmentIterator(_segments->end()), getSplineOrder() - 1);
		internal::moveIterator(_end = _segments->end(), SegmentIterator(_segments->begin()), - getSplineOrder());
	}


	_TEMPLATE
	void _CLASS::addKnots(const Eigen::VectorXd & knotTimes)
	{
		assertConstructing();

		for(int i = 0, n = knotTimes.rows(); i < n; i++){
			addKnot(knotTimes[i]);
		}
	}

	_TEMPLATE
	void _CLASS::addKnotsAndControlVertices(const Eigen::VectorXd & knotTimes, const Eigen::MatrixXd & controlVertices)
	{
		assertConstructing();
		SM_ASSERT_EQ(Exception, controlVertices.cols(), knotTimes.size(), "There must be as many knot times as controlVertices!");

		for(int i = 0, n = controlVertices.cols(); i < n; i++){
			addControlVertex(knotTimes[i], controlVertices.col(i));
		}
	}

	_TEMPLATE
	void _CLASS::initWithKnotsAndControlVertices(const Eigen::VectorXd & knotTimes, const Eigen::MatrixXd & controlVertices)
	{
		assertConstructing();
		const int numKnots = knotTimes.size();
		const int numVertices = controlVertices.cols();
		SM_ASSERT_EQ(Exception, controlVertices.cols() + getSplineOrder(), numKnots, "There must be as many knots as controlVertices + splineOrder!");

		int i;
		for(i = 0; i < numVertices; i++){
			addControlVertex(knotTimes[i], controlVertices.col(i));
		}
		for(; i < numKnots; i++){
			addKnot(knotTimes[i]);
		}
		init();
	}

	_TEMPLATE
	void _CLASS::initWithKnots(const Eigen::VectorXd & knotTimes)
	{
		assertConstructing();
		const int numKnots = knotTimes.size();
		SM_ASSERT_GE(Exception, numKnots, getMinimumKnotsRequired(), "Too view knots for this spline order!");

		for(int i = 0; i < numKnots; i++){
			addKnot(knotTimes[i]);
		}
		init();
	}

	_TEMPLATE
	void _CLASS::setControlVertices(const Eigen::MatrixXd & controlVertices){
		setControlVertices(controlVertices, getMinTime());
	}

	_TEMPLATE
	void _CLASS::setControlVertices(const Eigen::MatrixXd & controlVertices, const time_t startTime, int offset)
	{
		bool columnsMode;
		int n = controlVertices.size();
		int D = getPointSize();

		if(D == 1){
			columnsMode = controlVertices.rows() == 1;
		}
		else{
			if((columnsMode = (controlVertices.rows() == D)) || controlVertices.cols() == D){
				n /= D;
				D = 1;
			}
			else if ((columnsMode = (controlVertices.rows() == 1)) || controlVertices.cols() == 1){
				SM_ASSERT_EQ(Exception, (n % D), 0, "The size of the matrix must be a multiple of the spline's dimension!");
				n /= D;
			}
			else {
				SM_THROW(Exception, "The size of the matrix has illegal shape: " << controlVertices.rows() << " x " << controlVertices.cols() << "!");
			}
		}

		auto & manifold = this->getManifold();

		std::function<void (int, point_t &)> vertexManipulator;
		if(D == 1){
			if(columnsMode)
				vertexManipulator = [&controlVertices, &manifold](int i, point_t & v) { v = controlVertices.col(i); manifold.projectIntoManifold(v);};
			else
				vertexManipulator = [&controlVertices, &manifold](int i, point_t & v) { v = controlVertices.row(i); manifold.projectIntoManifold(v); };
		}
		else {
			if(columnsMode)
				vertexManipulator = [&controlVertices, D, &manifold](int i, point_t & v) { v = controlVertices.block(0, i * D, 1, D).transpose(); manifold.projectIntoManifold(v); };
			else
				vertexManipulator = [&controlVertices, D, &manifold](int i, point_t & v) { v = controlVertices.block(i * D, 0, D, 1); manifold.projectIntoManifold(v); };
		}
		manipulateControlVertices(vertexManipulator, n, startTime, offset);
	}

	_TEMPLATE
	void _CLASS::manipulateControlVertices(std::function<void (int index, point_t & controlVertex)> controlVertexManipulator, int manipulateTheFirstNControlVertices, int offset)
	{
		manipulateControlVertices(controlVertexManipulator, manipulateTheFirstNControlVertices, getMinTime(), offset);
	}

	_TEMPLATE
	void _CLASS::manipulateControlVertices(std::function<void (int index, point_t & controlVertex)> controlVertexManipulator, int manipulateTheFirstNControlVertices, const time_t startingAtFirstRelevantControlVertexForThisTime, int offset)
	{
		SM_ASSERT_LE(Exception, manipulateTheFirstNControlVertices, getNumControlVertices(), "There must be getNumControlVertices() or less many controlVertices to be manipulated!");
		int i = 0;
		SegmentIterator it = getFirstRelevantSegmentByLast(getSegmentIterator(startingAtFirstRelevantControlVertexForThisTime));
		if(offset){
			internal::moveIterator(it, offset > 0 ? end() : begin(), offset);
		}
		for(SegmentIterator  end = getAbsoluteEnd(); it != end && i < manipulateTheFirstNControlVertices; it ++){
			controlVertexManipulator(i++, it->getControlVertex());
		}
	}

	_TEMPLATE
	void _CLASS::initConstantSpline(KnotGenerator<time_t> & knotGenerator, const point_t & constant)
	{
		assertConstructing();
		SM_ASSERT_GE(Exception, constant.size(), 1, "The constant vector must be of at least length 1");

		while(knotGenerator.hasNextKnot())
			addControlVertex(knotGenerator.getNextKnot(), constant);

		getDerived().init();
	}

	_TEMPLATE
	void _CLASS::initConstantUniformSpline(const time_t & tMin, const time_t & tMax, int numSegments, const point_t & constant)
	{
		IntervalUniformKnotGenerator<TimePolicy> generator(getSplineOrder(), tMin, tMax, numSegments);
		initConstantSpline(generator, constant) ;
	}

	_TEMPLATE
	inline DeltaUniformKnotGenerator<typename _CLASS::TimePolicy> _CLASS::initConstantUniformSplineWithKnotDelta(const time_t & tMin, const time_t & beyondThisTime, const duration_t knotDelta, const point_t & constant){
		DeltaUniformKnotGenerator<TimePolicy> generator(getSplineOrder(), tMin, beyondThisTime, knotDelta);
		initConstantSpline(generator, constant);
		return generator;
	}


	_TEMPLATE
	typename _CLASS::time_t _CLASS::appendSegments(KnotGenerator<time_t> & knotGenerator, int numSegments, const point_t * value) {
		assertEvaluable();
		SM_ASSERT_TRUE(Exception, knotGenerator.supportsAppending(), "The knot generator needs to support appending!");

		SegmentIterator it = getAbsoluteEnd(), absBegin = getAbsoluteBegin();
		SegmentIterator currentValidEnd = _end;
		internal::moveIterator(it, absBegin, -(getSplineOrder()));
		SM_ASSERT_TRUE(Exception, currentValidEnd == it, "Append may only be called on a tail slice.");

		int j;
		for(j = 0; ; j++) {
			if(numSegments >= 0) {
				if(j >= numSegments)
					break;
			}
			else {
				if(!knotGenerator.hasNextKnot()){
					break;
				}
			}

			time_t newKnot = knotGenerator.getNextKnot();
			addKnot(newKnot);
			currentValidEnd++;
			if(value != NULL) {
				it->setControlVertex(*value);
				it++;
			}
		}

		if(_end != currentValidEnd){
			SegmentIterator formerLast = internal::getMovedIterator(_end, absBegin, -1);

			_end = currentValidEnd;

			if(getState() == internal::state::EVALUABLE){
				initializeBasisMatrices(internal::getMovedIterator(formerLast, absBegin, -(getSplineOrder() - 1)), formerLast);
			}
		}
		return getMaxTime();
	}

	_TEMPLATE
	typename _CLASS::time_t _CLASS::appendSegmentsUniformly(unsigned int numSegments, const point_t * value, const time_t beyondThisTime) {
		SegmentIterator it = getAbsoluteEnd(), aBegin = getAbsoluteBegin();
		internal::moveIterator(it, aBegin, -1);

		time_t atEndKnot = it.getKnot();

		internal::moveIterator(it, aBegin, -1);
		time_t beforeAbsEndKnot = it.getKnot();

		auto knotGenerator = DeltaUniformKnotGenerator<TimePolicy>(getSplineOrder(), beforeAbsEndKnot, beyondThisTime, computeDuration(beforeAbsEndKnot, atEndKnot), true);
		knotGenerator.jumpOverNextKnots(2);
		return getDerived().appendSegments(knotGenerator, numSegments, value);
	}


	_TEMPLATE
	inline typename _CLASS::duration_t _CLASS::computeDuration(typename _CLASS::time_t from, typename _CLASS::time_t till){
		return TTimePolicy::computeDuration(from, till);
	}


	_TEMPLATE
	inline double _CLASS::divideDurations(duration_t a, duration_t b){
		return TTimePolicy::divideDurations(a, b);
	}


	_TEMPLATE
	inline double _CLASS::getDurationAsDouble(duration_t d){
		return divideDurations(d, TTimePolicy::getOne());
	}

	_TEMPLATE
	int _CLASS::getNumKnotsRequired(int validSegments) const
	{
		return knot_arithmetics::getNumKnotsRequired(validSegments, getSplineOrder());
	}

	_TEMPLATE
	int _CLASS::getNumControlVerticesRequired(int validSegments) const
	{
		return knot_arithmetics::getNumControlVerticesRequired(validSegments, getSplineOrder());
	}

	_TEMPLATE
	void _CLASS::initializeBasisMatrices(){
		initializeBasisMatrices(_firstRelevantSegment,  _begin);
	}

	_TEMPLATE
	void _CLASS::initializeBasisMatrices(SegmentMapIterator currentKnotIt, SegmentMapIterator currentBasisMatrixIt)
	{
		const int splineOrder = getSplineOrder();
		const SegmentMapIterator end = _segments->end();

		std::deque<time_t> relevantKnots; //TODO optimize : replace deque with a fixed size (splineOrder * 2 - 1) circular sequence on the stack

		// try to collect the first splineOrder * 2 - 1 knots
		for (int c = 0, n = splineOrder * 2 - 1; c < n; c++){
			relevantKnots.push_back(currentKnotIt->first);
			if(c == splineOrder - 1){
				SM_ASSERT_TRUE(Exception, currentBasisMatrixIt == currentKnotIt, "internal error");
			}
			if(currentKnotIt == end){
				SM_THROW(Exception, "internal failure : there should be spline order -1 many control points behind the slice beginning");
				return;
			}
			currentKnotIt++;
		}

		// collect further knots and build the basis matrices until end
		for(; currentKnotIt != end && currentBasisMatrixIt != _end; currentKnotIt++)
		{
			relevantKnots.push_back(currentKnotIt->first);
			currentBasisMatrixIt->second.getBasisMatrix() = M(getSplineOrder(), relevantKnots);
			currentBasisMatrixIt ++;
			relevantKnots.erase(relevantKnots.begin());
		}
	}


	_TEMPLATE
	Eigen::MatrixXd _CLASS::M(int k, const std::deque<time_t> & knots)
	{
		int i = getSplineOrder() - 1;
		SM_ASSERT_GE_DBG(Exception, k, 1, "The parameter k must be greater than or equal to 1");
		SM_ASSERT_EQ_DBG(Exception, (int)knots.size(), getSplineOrder() * 2, "The parameter knots must have ISplineOrder many elements");
		if(k == 1)
		{
			// The base-case for recursion.
			Eigen::MatrixXd M(1,1);
			M(0,0) = 1;
			return M;
		}
		else
		{
			Eigen::MatrixXd M_km1 = M(k-1, knots);
			// The recursive equation for M
			// M_k = [ M_km1 ] A  + [  0^T  ] B
			//       [  0^T  ]      [ M_km1 ]
			//        -------        -------
			//         =: M1          =: M2
			//
			//     = M1 A + M2 B
			Eigen::MatrixXd M1 = Eigen::MatrixXd::Zero(M_km1.rows() + 1, M_km1.cols());
			Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(M_km1.rows() + 1, M_km1.cols());

			M1.topRightCorner(M_km1.rows(),M_km1.cols()) = M_km1;
			M2.bottomRightCorner(M_km1.rows(),M_km1.cols()) = M_km1;

			Eigen::MatrixXd A = Eigen::MatrixXd::Zero(k-1, k);
			for(int idx = 0; idx < A.rows(); idx++)
			{
				int j = i - k + 2 + idx;
				double d0 = d_0(k, i, j, knots);
				A(idx, idx  ) = 1.0 - d0;
				A(idx, idx+1) = d0;
			}

			Eigen::MatrixXd B = Eigen::MatrixXd::Zero(k-1, k);
			for(int idx = 0; idx < B.rows(); idx++)
			{
				int j = i - k + 2 + idx;
				double d1 = d_1(k, i, j, knots);
				B(idx, idx  ) = -d1;
				B(idx, idx+1) = d1;
			}


			Eigen::MatrixXd M_k;

			return M_k = M1 * A + M2 * B;
		}
	}


	_TEMPLATE
	double _CLASS::d_0(int k, int i, int j, const std::deque<time_t> & knots)
	{
		SM_ASSERT_GE_LT_DBG(Exception,j+k-1,0,(int)knots.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
		SM_ASSERT_GE_LT_DBG(Exception,j,0,(int)knots.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
		SM_ASSERT_GE_LT_DBG(Exception,i,0,(int)knots.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
		duration_t denom = computeDuration(knots[j], knots[j+k-1]);
		if(denom <= TTimePolicy::getZero())
			return 0.0;

		duration_t numerator = computeDuration(knots[j], knots[i]);

		return divideDurations(numerator, denom);
	}


	_TEMPLATE
	double _CLASS::d_1(int k, int i, int j, const std::deque<time_t> & knots)
	{
		SM_ASSERT_GE_LT_DBG(Exception,j+k-1,0,(int)knots.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
		SM_ASSERT_GE_LT_DBG(Exception,i+1,0,(int)knots.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
		SM_ASSERT_GE_LT_DBG(Exception,i,0,(int)knots.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
		duration_t denom = computeDuration(knots[j], knots[j+k-1]);
		if(denom <= TTimePolicy::getZero())
			return 0.0;

		duration_t numerator = computeDuration(knots[i], knots[i+1]);

		return divideDurations(numerator, denom);
	}


	_TEMPLATE
	inline typename _CLASS::duration_t _CLASS::computeSegmentLength(SegmentMapConstIterator segmentIt) const {
		if(segmentIt == _segments->end()){
			//TODO discuss
			return TTimePolicy::getZero();
		}
		else{
			return computeDuration(segmentIt->first, internal::getMovedIterator<SegmentMapConstIterator>(segmentIt, _segments->end(), 1)->first);
		}
	}

	/**
	 * dose the same as result = A.transpose() * b;
	 * but about 20 % faster.
	 *
	 * @param A the matrix
	 * @param b the vector
	 * @param result = A.transpose() * b
	 */
	template<typename Matrix, typename Vector1, typename Vector2>
	inline void fastMultiplyAtransposedTimesBInto(const Matrix & A, const Vector1 & b, Vector2 & result)
	{
		for(int j = result.rows() - 1; j >= 0; j--)
		{
			result(j) = b.dot(A.col(j));
		}
	}


	_TEMPLATE
	inline int _CLASS::getNumValidTimeSegments() const
	{
		return knot_arithmetics::getNumValidTimeSegments(getNumKnots(), getSplineOrder());
	}


	_TEMPLATE
	inline int _CLASS::getNumControlVertices() const{
		assertEvaluable();
		return knot_arithmetics::getNumControlVerticesRequired(getNumValidTimeSegments(), getSplineOrder());
	}


	_TEMPLATE
	inline size_t _CLASS::getNumKnots() const
	{
		//TODO adapt to slices
		return _segments->size();
	}


	_TEMPLATE
	inline typename _CLASS::time_t _CLASS::getMinTime() const
	{
		return _begin.getKnot();
	}

	_TEMPLATE
	inline typename _CLASS::time_t _CLASS::getMaxTime() const
	{
		return _end.getKnot();
	}

	_TEMPLATE
	inline typename _CLASS::SegmentConstIterator _CLASS::firstRelevantSegment() const {
		return _firstRelevantSegment;
	}

	_TEMPLATE
	inline typename _CLASS::SegmentIterator _CLASS::firstRelevantSegment() {
		return _firstRelevantSegment;
	}

	_TEMPLATE
	inline typename _CLASS::SegmentConstIterator _CLASS::begin() const {
		return _begin;
	}

	_TEMPLATE
	inline typename _CLASS::SegmentIterator _CLASS::begin() {
		return _begin;
	}

	_TEMPLATE
	inline typename _CLASS::SegmentConstIterator _CLASS::end() const {
		return _end;
	}

	_TEMPLATE
	inline typename _CLASS::SegmentIterator _CLASS::end() {
		return _end;
	}

	_TEMPLATE
	void _CLASS::computeDiiInto(const SegmentConstIterator & segmentIt, SplineOrderSquareMatrix & D) const
	{
		SM_ASSERT_LE_DBG(Exception, segmentIt.getKnot(), getMaxTime(), "Out of range");
		duration_t dt = computeSegmentLength(segmentIt);

		D.setZero();
		if(dt == TTimePolicy::getZero()){
			return;
		}
		double recip_dt = 0.0;
		if(dt > 0)
			recip_dt = TTimePolicy::divideDurations(TTimePolicy::getOne(), dt);
		for(int i = 0, n = getSplineOrder() - 1; i < n; i++)
		{
			D(i,i+1) = (i+1.0) * recip_dt;
		}
	}


	_TEMPLATE
	void _CLASS::computeViInto(SegmentConstIterator segmentIt, SplineOrderSquareMatrix & V) const
	{
		SM_ASSERT_GE_LT_DBG(Exception, segmentIt.getKnot(), getMinTime(), getMaxTime(), "Out of range");
//			SM_ASSERT_GE_LT_DBG(Exception, segmentIndex, 0, getNumValidTimeSegments(), "Segment index out of bounds");

		const auto twiceSplineOrder = getSplineOrder() * eigenTools::DynamicOrTemplateInt<2>();
		typedef decltype(twiceSplineOrder) TwiceSplineOrder;
		Eigen::Matrix<double, TwiceSplineOrder::VALUE, 1> vals(twiceSplineOrder);
		for (int i = 0, n = vals.rows(); i < n; ++i)
		{
			vals[i] = 1.0/(i + 1.0);
		}

		for(int r = 0, nr = V.rows(), nc = V.cols(); r < nr; r++)
		{
			for(int c = 0; c < nc; c++)
			{
				V(r,c) = vals[r + c];
			}
		}

		V *= getDurationAsDouble(computeSegmentLength(segmentIt));
	}


	_TEMPLATE
	inline typename _CLASS::SegmentIterator _CLASS::getSegmentIterator(const time_t & t)
	{
		SegmentMapIterator ub;
		SM_ASSERT_GE(Exception, t, getMinTime(), "The time is less then the spline's minimum time :" << (computeDuration(getMinTime(), t)));
		SM_ASSERT_LE(Exception, t, getMaxTime(), "The time is greater than the spline's maximum time :" << (computeDuration(t, getMaxTime())));
		if(t == getMaxTime())
		{
			//TODO discuss
			// This is a special case to allow us to evaluate the spline at the boundary of the
			// interval. This is not strictly correct but it will be useful when we start doing
			// estimation and defining knots at our measurement times.
			ub = _end;
		}
		else
		{
			ub = _segments->upper_bound(t);
		}
		return --ub;
	}


	_TEMPLATE
	inline typename _CLASS::SegmentConstIterator _CLASS::getSegmentIterator(const time_t & t) const
	{
		return const_cast<typename TConfigurationDerived::BSpline *>((const typename TConfigurationDerived::BSpline *)this)->getSegmentIterator(t);
	}


	_TEMPLATE
	inline typename _CLASS::time_t _CLASS::getMinimalDistanceToNeighborKnots(const time_t & t) const {
		auto it = getSegmentIterator(t);
		return std::min(t - it->getKnot(), getMovedIterator(it, getAbsoluteEnd(), 1)->getKnot() - t);
	}

	_TEMPLATE
	inline typename _CLASS::SegmentConstIterator _CLASS::getFirstRelevantSegmentByLast(const SegmentConstIterator & first) const
	{
		return internal::getMovedIterator(first, getAbsoluteBegin(), -(getSplineOrder() - 1));
	}


	_TEMPLATE
	inline typename _CLASS::SegmentIterator _CLASS::getFirstRelevantSegmentByLast(const SegmentIterator & first)
	{
		return internal::getMovedIterator(first, getAbsoluteBegin(), -(getSplineOrder() - 1));
	}


	_TEMPLATE
	inline void _CLASS::setLocalCoefficientVector(const time_t & t, const Eigen::VectorXd & coefficients, int pointSize){
		SegmentIterator it = getSegmentIterator(t);
		const int D = pointSize;
		int index = (getSplineOrder()-1) * D;
		SM_ASSERT_EQ(Exception, coefficients.rows(), index + D, "wrong number of coefficients")

		for(SegmentMapIterator end = _segments->begin(); index >= 0; index -= D){
			it->getControlVertex() = coefficients.segment(index, D);
			if(it == end) break;
			it--;
		}
	}


	_TEMPLATE
	inline void _CLASS::getLocalCoefficientVector(const time_t & t, Eigen::VectorXd & coefficients, int pointSize){
		SegmentIterator it = getSegmentIterator(t);
		const int D = pointSize;
		coefficients.resize(D * getSplineOrder());
		int index = (getSplineOrder()-1) * D;
		SM_ASSERT_EQ(Exception, coefficients.rows(), index + D, "wrong number of coefficients")

		for(SegmentMapIterator end = _segments->begin(); index >= 0; index -= D){
			coefficients.segment(index, D) = it->getControlVertex();
			if(it == end) break;
			it--;
		}
	}

	template <typename SplineT>
	struct EvalFunctor{
		inline typename SplineT::point_t eval(const SplineT & spline, typename SplineT::time_t t) const {
			return spline.template getEvaluatorAt<0>(t).eval();
		}
		inline typename SplineT::point_t  getZeroValue(const SplineT & spline) const {
			return SplineT::point_t::Zero((int)spline.getPointSize());
		}
	};

	_TEMPLATE
	inline typename _CLASS::point_t _CLASS ::evalIntegralNumerically(const time_t & t1, const time_t & t2, int numberOfPoints) const {
		return getDerived().template evalFunctorIntegralNumerically<point_t>(t1, t2, EvalFunctor<spline_t>(), numberOfPoints);
	}

	template <typename SplineT, typename TFunctor, typename TValue>
	struct IntegrandFunctor {
		const SplineT & _spline;
		const TFunctor & _f;
		IntegrandFunctor(const SplineT & spline, const TFunctor & f): _spline(spline), _f(f){}

		inline TValue operator() (typename SplineT::time_t t) const {
			return _f.eval(_spline, t);
		}
	};

	_TEMPLATE
	template <typename TValue, typename TFunctor>
	inline TValue _CLASS ::evalFunctorIntegralNumerically(const time_t & t1, const time_t & t2, const TFunctor & f, int numberOfPoints) const {
		if(t1 > t2) return -getDerived().template evalFunctorIntegralNumerically<TValue, TFunctor>(t2, t1, f, numberOfPoints);
		typedef IntegrandFunctor<spline_t, TFunctor, TValue> IFunc;
		return numeric_integrator::template integrateFunctor<numeric_integrator::algorithms::Default, TValue, time_t, const IFunc>(t1, t2, IFunc(this->getDerived(), f), numberOfPoints, f.getZeroValue(this->getDerived()));
	}

	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	_CLASS::Evaluator<IMaximalDerivativeOrder >::Evaluator(const spline_t & spline, const time_t & t) :
		internal::AssertInitializedSpline<spline_t>(spline),
		_tmp(spline.getSplineOrder()),
		_spline(spline),
		_t(t),
		_ti(spline.getSegmentIterator(t)),
		_firstRelevantControlVertexIt(spline.getFirstRelevantSegmentByLast(_ti)),
		_end(internal::getMovedIterator(_ti, spline.getAbsoluteEnd(), 1)),
		_segmentLength(spline.computeSegmentLength(_ti)),
		_positionInSegment(computeDuration(_ti.getKnot(), t)),
		_relativePositionInSegment(((duration_t) (_segmentLength) == (duration_t) (TTimePolicy::getZero())) ? 0 : divideDurations(_positionInSegment, _segmentLength))
	{
		const auto splineOrder = _spline.getSplineOrder();
		//TODO optimize : set cache while calculating the u
		for(int derivativeOrder = 0; derivativeOrder < NumberOfPreparedDerivatives; derivativeOrder++){
			SplineOrderVector & lBi = _localBi[derivativeOrder];
			if(splineOrder.isDynamic())
				lBi.resize(splineOrder);
			computeLocalBiIntoT<NeedsCumulativeBasisMatrices>(lBi, derivativeOrder);
		}
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline typename _CLASS::SegmentConstIterator
	_CLASS::Evaluator<IMaximalDerivativeOrder>::getFirstRelevantSegmentIterator() const {
		return _firstRelevantControlVertexIt;
	}

	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline typename _CLASS::SegmentConstIterator _CLASS::Evaluator<IMaximalDerivativeOrder>::getLastRelevantSegmentIterator() const {
		return _ti;
	}

	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline typename _CLASS::SegmentConstIterator _CLASS::Evaluator<IMaximalDerivativeOrder>::end() const {
		return _end;
	}

	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline void _CLASS::Evaluator<IMaximalDerivativeOrder>::computeLocalBiInto(SplineOrderVector & ret, int derivativeOrder) const
	{
		SplineOrderVector u(_spline.getSplineOrder());
		computeUInto(derivativeOrder, u);
		fastMultiplyAtransposedTimesBInto(_ti->getBasisMatrix(), u, ret);
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline void _CLASS::Evaluator<IMaximalDerivativeOrder>::computeLocalCumulativeBiInto(SplineOrderVector & ret, int derivativeOrder) const
	{
		computeLocalBiInto(ret, derivativeOrder);
		computeLocalCumulativeBiInto(ret, ret, derivativeOrder, true);
	}

	//TODO implement cumulative basis matrix
	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline void _CLASS::Evaluator<IMaximalDerivativeOrder>::computeLocalCumulativeBiInto(const SplineOrderVector & localBi, SplineOrderVector & ret, int derivativeOrder, bool argumentsAreTheSame) const
	{
		//TODO assert derivability at the knots of given derivative order
		ret[0] = derivativeOrder == 0 ? 1.0 : 0.0; // the sum of splineOrder many successive spline basis functions is always 1
		int maxIndex = _spline.getSplineOrder() - 1;
		for(int i = 1; i <= maxIndex; i ++){
			double sum = 0;
			for(int j = maxIndex; j > i; j--)
				sum += localBi[j];
			if(argumentsAreTheSame)
				ret[i] += sum;
			else
				ret[i] = localBi[i] + sum;
		}
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline int _CLASS::Evaluator<IMaximalDerivativeOrder>::dmul(int i, int derivativeOrder) const
	{
		if(derivativeOrder == 0)
			return 1;
		else if(derivativeOrder == 1)
			return i;
		else
			return i * dmul(i-1,derivativeOrder-1) ;
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline void _CLASS::Evaluator<IMaximalDerivativeOrder>::computeUInto(int derivativeOrder, SplineOrderVector & u) const
	{
		if(_segmentLength > TTimePolicy::getZero()){
			double multiplier = 1.0/pow(getDurationAsDouble(_segmentLength), derivativeOrder);
			double uu = 1.0;
			int i = 0;
			for(; i < derivativeOrder; i ++){
				u[i] = 0;
			}
			for(int n = _spline.getSplineOrder(); i < n; i++)
			{
				u(i) = multiplier * uu * dmul(i, derivativeOrder);
				uu = uu * _relativePositionInSegment;
			}
		} else {
			u.setZero(_spline.getSplineOrder());
		}
	}

	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline void _CLASS::Evaluator<IMaximalDerivativeOrder>::computeVInto(SplineOrderVector & v) const
	{
		if(_segmentLength > TTimePolicy::getZero()){
			double du = getRelativePositionInSegment();
			for(int i = 0, n = _spline.getSplineOrder(); i < n; i++)
			{
				v(i) = du/(i + 1.0);
				du *= getRelativePositionInSegment();
			}
		} else {
			v.setZero(_spline.getSplineOrder());
		}
	}

	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	template<bool BCumulative>
	inline void _CLASS::Evaluator<IMaximalDerivativeOrder>::computeLocalBiIntoT(SplineOrderVector & ret, int derivativeOrder) const {
		if(BCumulative)
			computeLocalCumulativeBiInto(ret, derivativeOrder);
		else
			computeLocalBiInto(ret, derivativeOrder);
	}

	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	template<bool BCumulative>
	inline const typename _CLASS::SplineOrderVector & _CLASS::Evaluator<IMaximalDerivativeOrder>::getLocalBiT(int derivativeOrder) const {
		SM_ASSERT_GE_DBG(Exception, derivativeOrder, 0, "");
		const auto splineOrder = _spline.getSplineOrder();

		if(NeedsCumulativeBasisMatrices == BCumulative && derivativeOrder < NumberOfPreparedDerivatives){
			return _localBi[derivativeOrder];
		}
		else {
			if(derivativeOrder >= splineOrder) {
				return _tmp = SplineOrderVector::Zero(splineOrder);
			}
			else {
				computeLocalBiIntoT<BCumulative>(_tmp, derivativeOrder);
				return _tmp;
			}
		}
	}



	_TEMPLATE
	inline void _CLASS::computeLocalViInto(const SplineOrderVector & v, SplineOrderVector & localVi, const SegmentMapConstIterator & ti)
	{
		fastMultiplyAtransposedTimesBInto(ti->second.getBasisMatrix(), v, localVi);
	}



	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline void _CLASS::Evaluator<IMaximalDerivativeOrder>::computeLocalViInto(SplineOrderVector& localVi) const {
		SplineOrderVector v(_spline.getSplineOrder());
		computeVInto(v);
		spline_t::computeLocalViInto(v, localVi, _ti);
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline const typename _CLASS::SplineOrderVector & _CLASS::Evaluator<IMaximalDerivativeOrder>::getLocalBi(int derivativeOrder) const {
		return getLocalBiT<false>(derivativeOrder);
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline const typename _CLASS::SplineOrderVector & _CLASS::Evaluator<IMaximalDerivativeOrder>::getLocalCumulativeBi(int derivativeOrder) const {
		return getLocalBiT<true>(derivativeOrder);
	}

	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::eval() const {
		return evalGeneric();
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalD(int derivativeOrder) const {
		return evalDGeneric(derivativeOrder);
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	inline typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalGeneric() const {
		static_assert(_CLASS::spline_t::manifold_t::getCanonicalConnection() == ::manifolds::CanonicalConnection::LEFT, "Only Lie groups with canonical left multiplications are supported here, yet.");
		const SplineOrderVector & cumulativeBi = getLocalCumulativeBi();
		SegmentMapConstIterator it = _firstRelevantControlVertexIt;
		const point_t * lastControlVertex_t = & it->second.getControlVertex();
		point_t p = *lastControlVertex_t;
		tangent_vector_t vec;

		auto & manifold = _spline.getManifold();
		for (int i = 1, n = _spline.getSplineOrder(); i < n; i++){
			it++;
			const point_t * nextControlVertex_t = & it->second.getControlVertex();
			manifold.logInto(*lastControlVertex_t, *nextControlVertex_t, vec);
			lastControlVertex_t = nextControlVertex_t;
			double d = cumulativeBi[i];
			if(d == 0.0)
				continue;
			manifold.scaleVectorInPlace(vec, d);
			manifold.expInto(p, vec, p);
		}
		return p;
	}


//	_TEMPLATE
//	template <int IMaximalDerivativeOrder>
//	inline typename _CLASS::point_t _CLASS::Evaluator<IMaximalDerivativeOrder>::evalDGeneric(int derivativeOrder) const {
//		SM_ASSERT_GE_LT_DBG(Exception, derivativeOrder, 0, 2, "only derivative of order 0 and 1 are generically supported yet");
//
//		if(derivativeOrder == 0){
//			return evalGeneric();
//		}
//
//		const SplineOrderVector & cumulativeBi = getLocalCumulativeBi(0);
//		const SplineOrderVector & cumulativeBiD = getLocalCumulativeBi(1);
//
//		SegmentMapConstIterator it = _firstRelevantControlVertexIt;
//		const point_t * lastControlVertex_t = & it->second.getControlVertex();
//		point_t p = *lastControlVertex_t;
//		int n = _spline.getSplineOrder();
//		tangent_vector_t vec, vecD;
//
//		//TODO implement generic evalD
//		return p;
//	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	const typename _CLASS::duration_t _CLASS::Evaluator<IMaximalDerivativeOrder>::getSegmentLength() const {
		return _segmentLength;
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	const typename _CLASS::duration_t _CLASS::Evaluator<IMaximalDerivativeOrder>::getPositionInSegment() const {
		return _positionInSegment;
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	double _CLASS::Evaluator<IMaximalDerivativeOrder>::getRelativePositionInSegment() const {
		return _relativePositionInSegment;
	}


	_TEMPLATE
	template <int IMaximalDerivativeOrder>
	const typename _CLASS::spline_t & _CLASS::Evaluator<IMaximalDerivativeOrder>::getSpline() const {
		return _spline;
	}

	_TEMPLATE
	template<int IMaximalDerivativeOrder>
	inline typename internal::get_evaluator<typename _CLASS::spline_t, IMaximalDerivativeOrder>::type _CLASS::getEvaluatorAt(const time_t & t) const {
		return typename _CLASS::spline_t::template Evaluator<IMaximalDerivativeOrder>(this->getDerived(), t);
	}

#undef _TEMPLATE
#undef _CLASS
}

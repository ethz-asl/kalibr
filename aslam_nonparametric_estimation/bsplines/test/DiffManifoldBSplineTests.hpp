#ifndef RIEMANNIANBSPLINETESTS_HPP_
#define RIEMANNIANBSPLINETESTS_HPP_

// Bring in my package's API, which is what I'm testing
#include <bsplines/BSpline.hpp>
#include <bsplines/DiffManifoldBSpline.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <bsplines/BSplineFitter.hpp>
#include <bsplines/NsecTimePolicy.hpp>

#include <iostream>

// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Helpful functions from libsm
#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/timing/Timer.hpp>

#include <boost/tuple/tuple.hpp>

#include <ctime>

namespace bsplines
{

const unsigned int splineOrder = 4;
const unsigned int rows = 3;
const double minTime = 0;
const double maxTime = 1;
const double duration = maxTime - minTime;

const unsigned int numberOfSegments = 16;
const unsigned int numberOfTimeSteps = 32;


typedef EuclideanBSpline<>::TYPE TestSplineDD;
typedef EuclideanBSpline<splineOrder, Eigen::Dynamic>::TYPE TestSplineSD;
typedef EuclideanBSpline<Eigen::Dynamic, rows>::TYPE TestSplineDS;
typedef EuclideanBSpline<splineOrder, rows>::TYPE TestSpline;
typedef EuclideanBSpline<splineOrder, rows, NsecTimePolicy>::TYPE TestSplineNsecTime;

typedef UnitQuaternionBSpline<>::TYPE UQTestSplineD;
typedef UnitQuaternionBSpline<splineOrder>::TYPE UQTestSpline;


struct LongTime {
	long v;
	LongTime(long v) : v(v){

	}
};

struct LongDuration {
	long v;
	LongDuration(long v) : v(v){

	}
};


inline bool operator < (const struct LongTime a, const struct LongTime b){
	return a.v < b.v;
}
inline bool operator <= (const struct LongTime a, const struct LongTime b){
	return a.v <= b.v;
}
inline bool operator > (const struct LongTime a, const struct LongTime b){
	return a.v > b.v;
}
inline bool operator >= (const struct LongTime a, const struct LongTime b){
	return a.v >= b.v;
}
inline bool operator==(const struct LongTime & a,  const struct LongTime & b){
	return a.v == b.v;
}
inline std::ostream& operator<<(std::ostream& out, const LongDuration x)
{
	return out << x.v;
}

inline bool operator < (const struct LongDuration a, const struct LongDuration b){
	return a.v < b.v;
}
inline bool operator <= (const struct LongDuration a, const struct LongDuration b){
	return a.v <= b.v;
}
inline bool operator > (const struct LongDuration a, const struct LongDuration b){
	return a.v > b.v;
}
inline bool operator >= (const struct LongDuration a, const struct LongDuration b){
	return a.v >= b.v;
}
inline bool operator==(const struct LongDuration a, const struct LongDuration b){
	return a.v == b.v;
}
inline std::ostream& operator<<(std::ostream& out, const LongTime x)
{
	return out << x.v;
}


typedef struct {
	typedef struct LongTime time_t;
	typedef struct LongDuration duration_t;
	static duration_t computeDuration(time_t from, time_t till){
		return LongDuration(till.v - from.v);
	}

	static double divideDurations(duration_t a, duration_t b){
		return (double) a.v/b.v;
	}

	inline static time_t linearlyInterpolate(time_t from, time_t till, int segments, int pos)
	{
		return LongTime((till.v - from.v) / segments * pos);
	}

	inline static int getSegmentNumber(time_t from, time_t till, int segments, time_t t)
	{
		double dt = (till.v - from.v) / segments;
		return std::floor((double)(t.v - from.v) / dt);
	}

	inline static double getDurationAsDouble(duration_t d){
		return (double) d.v /getOne().v;
	}

	inline static duration_t getZero(){
		return LongDuration(0);
	}

	inline static duration_t getOne(){
		return LongDuration(1000000);
	}

} LongTimePolicy;

typedef EuclideanBSpline<splineOrder, rows, LongTimePolicy>::TYPE TestSplineLongTime;

const LongTime minTimeLong = LongTime(0);
const LongTime maxTimeLong = LongTime(1000000);
const LongDuration durationLong = LongTimePolicy::computeDuration(maxTimeLong, minTimeLong);

const Eigen::VectorXd zero = Eigen::VectorXd::Zero(rows);
const Eigen::VectorXd ones = Eigen::VectorXd::Ones(rows);


template<typename InputT>
struct InputTypeTraits {
	inline static InputT ones(){
		return InputT::Ones();
	}

	inline static InputT random(){
		return InputT::Random();
	}

	inline static void update(InputT & input, int c, double delta){
		input[c] += delta;
 	}
};

template <typename TSpline>
void initMinimalSpline(TSpline & spline){
	for(int i = 0, n = spline.getSplineOrder() * 2; i < n; i ++){
		spline.addKnot(i);
	}
	spline.init();
}



template <>
struct InputTypeTraits<double> {
	inline static double random(){
		return std::rand();
	}
	inline static double ones(){
		return 1;
	}
  inline static void update(double & input, int /* c */, double delta){
		input += delta;
 	}
};

template<typename TInput, typename TValue, typename TJacobian, typename TEvaluator>
struct JacobianTester{
	struct JacobianFunctor {
		typedef TInput input_t;
		typedef TJacobian jacobian_t;
		typedef TValue value_t;
		typedef double scalar_t;

		TEvaluator & _evaluator;

		JacobianFunctor(TEvaluator & evaluator) : _evaluator(evaluator){
		}

		input_t update(const input_t & x, int c, scalar_t delta)
		{
			input_t xnew = x;
			InputTypeTraits<TInput>::update(xnew, c, delta);
			return xnew;
		}

		inline value_t operator()(const input_t & c)
		{
			value_t v;
			_evaluator.eval(c, v);
			return v;
		}
	};

	static void testFunc(int numberOfPoints, int numberOfDirections, TEvaluator evaluator = TEvaluator()){
		for(int i = 0 ; i < numberOfPoints; i++){
			evaluator.randomParams();
			JacobianFunctor f(evaluator);
			sm::eigen::NumericalDiff<JacobianFunctor> nd(f);

			TJacobian estVal, val;
			TInput v;
			for(int j = 0 ; j < numberOfDirections; j ++){
				evaluator.randomizeInput(v);
				evaluator.evalJac(v, val);
				estVal = nd.estimateJacobian(v);
//			 		std::cout << "\nval:" << val << "\nestVal:" << estVal <<"\n";
				sm::eigen::assertNear(val, estVal, 1E-6, SM_SOURCE_FILE_POS);
			}
		}
	}
};

template <typename TDiffManifold>
struct DExpTester : public JacobianTester<typename TDiffManifold::tangent_vector_t, typename TDiffManifold::point_t, typename TDiffManifold::dmatrix_t, DExpTester<TDiffManifold> > {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typename TDiffManifold::point_t _pt;
	TDiffManifold _manifold;
	inline void eval(const typename TDiffManifold::tangent_vector_t & vec, typename TDiffManifold::point_t & result) const {
		_manifold.expInto(_pt, vec, result);
	}
	inline void evalJac(const typename TDiffManifold::tangent_vector_t & vec, typename TDiffManifold::dmatrix_t & result) const {
		_manifold.dexpInto(_pt, vec, result);
	}
	inline void randomParams(){
		_manifold.randomizePoint(_pt);
	}
	inline void randomizeInput(typename TDiffManifold::tangent_vector_t & vec){
		vec = 2 * TDiffManifold::tangent_vector_t::Random() -TDiffManifold::tangent_vector_t::Ones();
	}

};

template <typename TSpline, int IDerivativeOrder>
struct SplineEvalDTester : public JacobianTester<Eigen::Matrix<double, 1, 1>, typename TSpline::point_t, typename TSpline::point_t, SplineEvalDTester<TSpline, IDerivativeOrder> > {
	TSpline spline;
	SplineEvalDTester(){
		initMinimalSpline(spline);
	}

	inline void eval(Eigen::Matrix<double, 1, 1> t, typename TSpline::point_t & result) const {
		result = spline.template getEvaluatorAt<IDerivativeOrder - 1>(t[0]).evalD(IDerivativeOrder - 1);
	}
	inline void evalJac(Eigen::Matrix<double, 1, 1> t, typename TSpline::point_t & result) const {
		result = spline.template getEvaluatorAt<IDerivativeOrder>(t[0]).evalD(IDerivativeOrder);
	}

	inline void randomParams(){
		typename TSpline::SegmentIterator it = spline.getAbsoluteBegin();
		for(int i = 0, n = spline.getNumControlVertices(); i < n; i++){
			spline.getManifold().randomizePoint(it->getControlVertex());
			it++;
		}
	}
	inline void randomizeInput(Eigen::Matrix<double, 1, 1> & t){
		double r = (double)std::rand()/RAND_MAX;
		t[0] = spline.getMinTime() + (spline.getMaxTime() - spline.getMinTime()) * r;
	}
};

template <typename TSpline, int IDerivativeOrder>
struct SplineEvalRiDTester : public JacobianTester<Eigen::Matrix<double, 1, 1>, typename TSpline::point_t, typename TSpline::point_t, SplineEvalRiDTester<TSpline, IDerivativeOrder> > {
	TSpline spline;
	int vectorPos;
	typename TSpline::tangent_vector_t vec;

	SplineEvalRiDTester() : vectorPos(0){
		initMinimalSpline(spline);
	}

	template<int IDerivativeOrder2> inline void e(Eigen::Matrix<double, 1, 1> t, typename TSpline::point_t & result) const {
		BOOST_AUTO(eval, spline.template getEvaluatorAt<IDerivativeOrder2>(t[0]));
		typename TSpline::template Evaluator<IDerivativeOrder2>::CalculationCache cache(&eval);
		cache.localPhiVectors[vectorPos] = vec;
		cache.localRiPoints[vectorPos] = spline.getManifold().expAtId(vec * eval.getLocalCumulativeBi(0)[vectorPos + 1]);
		result = eval.evalRiD(cache, IDerivativeOrder2, vectorPos + 1);
	}

	inline void eval(Eigen::Matrix<double, 1, 1> t, typename TSpline::point_t & result) const {
		e<IDerivativeOrder - 1>(t, result);
	}
	inline void evalJac(Eigen::Matrix<double, 1, 1> t, typename TSpline::point_t & result) const {
		e<IDerivativeOrder>(t, result);
	}

	inline void randomParams(){
		vectorPos = rand() % (spline.getSplineOrder() - 1);
		vec.setRandom(vec.rows());
	}
	inline void randomizeInput(Eigen::Matrix<double, 1, 1> & t){
		Eigen::Matrix<double, 1, 1> ret;
		double r = (double)std::rand()/RAND_MAX;
		t[0] = spline.getMinTime() + (spline.getMaxTime() - spline.getMinTime()) * r;
	}
};

template <typename TManifold>
inline void disturbePointInto(const TManifold & manifold, const typename TManifold::tangent_vector_t & vec, const typename TManifold::point_t & p, typename TManifold::point_t & into){
	into = p;
	manifolds::internal::DiffManifoldPointUpdateTraits<typename TManifold::configuration_t>::update(manifold, into, vec);
}

template <typename TSpline, int IDerivativeOrder>
struct SplineEvalRiDJacobianTester : public JacobianTester<typename TSpline::tangent_vector_t, typename TSpline::point_t, typename TSpline::dmatrix_t, SplineEvalRiDJacobianTester<TSpline, IDerivativeOrder> > {
	TSpline spline;
	typedef typename TSpline::tangent_vector_t input_t;
	int vectorPos, controlVertexPos;
	double _t;
	typename TSpline::SegmentIterator _controlVertexIt;
	typename TSpline::point_t toBeDisturbedPoint;

	SplineEvalRiDJacobianTester() : vectorPos(0), controlVertexPos(0), _t(0) {
		initMinimalSpline(spline);
	}

	inline void updateControlVertex(const input_t & vec) const
	{
		disturbePointInto(spline.getManifold(), vec, toBeDisturbedPoint, _controlVertexIt->getControlVertex());
	}

	inline void eval(const input_t & vec, typename TSpline::point_t & result) {
		updateControlVertex(vec);
		BOOST_AUTO(eval, spline.template getEvaluatorAt<IDerivativeOrder>(_t));
		typename TSpline::template Evaluator<IDerivativeOrder>::CalculationCache cache(&eval);
		result = eval.evalRiD(cache, IDerivativeOrder, vectorPos);
	}
	inline void evalJac(const input_t & vec, typename TSpline::dmatrix_t & result) {
		updateControlVertex(vec);
		BOOST_AUTO(eval, spline.template getEvaluatorAt<IDerivativeOrder>(_t));
		typename TSpline::template Evaluator<IDerivativeOrder>::CalculationCache cache(&eval);
		result = eval.getRiDJacobian(cache, IDerivativeOrder, vectorPos) * (vectorPos == controlVertexPos ? 1 : -1);
	}

	inline void randomParams(){
		double r = (double)std::rand()/RAND_MAX;
		_t = spline.getMinTime() + (spline.getMaxTime() - spline.getMinTime()) * r;
		_controlVertexIt  = spline.getSegmentIterator(_t);

		controlVertexPos = rand() % (spline.getSplineOrder() - 1);
		if(controlVertexPos == 0)
			vectorPos = controlVertexPos + 1;
		else
			vectorPos = controlVertexPos + rand() % 2;

		for(int i = 0, n = spline.getSplineOrder() - controlVertexPos - 1; i < n ; i++){
			_controlVertexIt--;
		}

		typename TSpline::SegmentIterator it = spline.getAbsoluteBegin();
		for(int i = 0, n = spline.getNumControlVertices(); i < n; i++){
			spline.getManifold().randomizePoint(it->getControlVertex());

			it->getControlVertex().setZero(spline.getPointSize());
			it->getControlVertex()[(i+3) % spline.getPointSize()] = 1;

			it++;
		}

		toBeDisturbedPoint = _controlVertexIt->getControlVertex();
	}

	inline void randomizeInput(input_t & vec){
		vec.setZero();
	}
};

template <typename TSpline, int IDerivativeOrder>
struct BSplineJacobianEvaluator{
	TSpline spline;
	typedef Eigen::Matrix<double, multiplyEigenSize(TSpline::Dimension, TSpline::SplineOrder), 1> input_t;
	double _t;
	typename TSpline::SegmentIterator _it;

	typedef typename TSpline::point_t point_t;

	std::vector<point_t> toBeDisturbedPoints;

	BSplineJacobianEvaluator() :  _t(0), toBeDisturbedPoints(spline.getSplineOrder()) {
		initMinimalSpline(spline);
	}

	inline void updateSpline(const input_t & input) {
		typename TSpline::SegmentIterator it = _it;
		int dim = spline.getDimension();
		typename TSpline::tangent_vector_t vec;
		for(int o = spline.getSplineOrder(), i = o - 1; i >= 0 ; i--){
			vec = input.segment(i * dim, dim);
			disturbePointInto(spline.getManifold(), vec, toBeDisturbedPoints[i], it->getControlVertex());
			it--;
		}
	}

	inline void eval(const input_t & input, typename TSpline::point_t & result) {
		updateSpline(input);
		typename TSpline::template Evaluator<IDerivativeOrder> eval = spline.template getEvaluatorAt<IDerivativeOrder>(_t);
		result = eval.evalD(IDerivativeOrder);
	}

	inline void evalJac(const input_t & input, Eigen::Matrix<double, TSpline::PointSize , multiplyEigenSize(TSpline::Dimension, TSpline::SplineOrder)> & result) {
		updateSpline(input);
		typename TSpline::template Evaluator<IDerivativeOrder> eval = spline.template getEvaluatorAt<IDerivativeOrder>(_t);
		eval.evalJacobian(IDerivativeOrder, result);
	}

	inline void randomParams(){
		double r = (double)std::rand()/RAND_MAX;
		_t = spline.getMinTime() + (spline.getMaxTime() - spline.getMinTime()) * r;

		_it  = spline.getSegmentIterator(_t);

		typename TSpline::SegmentIterator it = spline.getAbsoluteBegin();
		for(int i = 0, n = spline.getNumControlVertices(); i < n; i++){
			spline.getManifold().randomizePoint(it->getControlVertex());
			it++;
		}

		it = _it;
		for(int o = spline.getSplineOrder(), i = o - 1; i >= 0 ; i--){
			toBeDisturbedPoints[i] = it->getControlVertex();
			it--;
		}
	}

	inline void randomizeInput(input_t  & ret){
		ret.setZero();
	}
};

template <typename TSpline, int IDerivativeOrder>
struct BSplineJacobianTester :
	public JacobianTester<
		Eigen::Matrix<double, multiplyEigenSize(TSpline::Dimension, TSpline::SplineOrder), 1>,
		typename TSpline::point_t,
		Eigen::Matrix<double, TSpline::PointSize , multiplyEigenSize(TSpline::Dimension, TSpline::SplineOrder)>,
		BSplineJacobianEvaluator<TSpline, IDerivativeOrder>
	>
{
};

template <typename TSpline, int IDerivativeOrder>
struct AngularDerivativeJacobianEvaluator : public BSplineJacobianEvaluator<TSpline, IDerivativeOrder> {
	typedef Eigen::Matrix<double, multiplyEigenSize(TSpline::Dimension, TSpline::SplineOrder), 1> input_t;

	inline void eval(const input_t & input, typename Eigen::Vector3d & result) {
		this->updateSpline(input);
		typename TSpline::template Evaluator<IDerivativeOrder> eval = this->spline.template getEvaluatorAt<IDerivativeOrder>(this->_t);
		result = eval.template evalAngularDerivative<IDerivativeOrder>();
	}

	inline void evalJac(const input_t & input, Eigen::Matrix<double, TSpline::Dimension , multiplyEigenSize(TSpline::Dimension, TSpline::SplineOrder)> & result) {
		this->updateSpline(input);
		typename TSpline::template Evaluator<IDerivativeOrder> eval = this->spline.template getEvaluatorAt<IDerivativeOrder>(this->_t);
		eval.template evalAngularDerivativeJacobian<IDerivativeOrder>(result);
	}
};


template <typename TSpline, int IDerivativeOrder>
struct AngularDerivativeJacobianTestser :
	public JacobianTester<
		Eigen::Matrix<double, multiplyEigenSize(TSpline::Dimension, TSpline::SplineOrder), 1>,
		Eigen::Vector3d,
		Eigen::Matrix<double, TSpline::Dimension, multiplyEigenSize(TSpline::Dimension, TSpline::SplineOrder)>,
		AngularDerivativeJacobianEvaluator<TSpline, IDerivativeOrder>
	>
{
};

template<typename Iterator> int getIteratorDistance(const Iterator & begin, const Iterator & end, const Iterator & limit){
	int c = 0;
	Iterator i = begin;
	for(; i != end && i != limit; i++){
		c++;
	}
	if(i == limit && i != end){
		return -1;
	}
	else {
		return c;
	}
}

template<typename TSpline> void copyKnots(TSpline & src, BSpline & dest){
	Eigen::VectorXd knots = dest.knotVector();
	typename TSpline::SegmentIterator it = src.getAbsoluteBegin();
	for(int i = 0, n = knots.rows(); i < n; i++ ){
		knots[i] = (it++)->first;
	}
	dest.setKnotVectorAndCoefficients(knots, dest.coefficients());
}

template <typename TSpline, typename Container>
void setControlVerticesFromContainer(TSpline & spline, const Container & controlVertices){
	auto it = controlVertices.begin();
	spline.manipulateControlVertices([&it, &controlVertices, &spline](int, typename TSpline::point_t & vertex){ vertex = *it; it++;}, controlVertices.size());
}



template<typename TSpline>
void testDiffManifoldBSplineFitting(int numberOfSegments, double tolerance = 1E-9)
{
	typedef TSpline TestSpline;
	typedef typename TestSpline::time_t time_t;

	const time_t minTime = 0, duration = TSpline::TimePolicy::getOne(), maxTime = minTime + duration;

	const int numberOfInterpolationPoints = numberOfSegments * 2 + 3;

	Eigen::Matrix<time_t, Eigen::Dynamic, 1> times = Eigen::Matrix<time_t, Eigen::Dynamic, 1>::LinSpaced(numberOfInterpolationPoints, minTime, maxTime);
	const double lambda = 0.1;

	TestSpline rbspline(splineOrder);
	TestSpline rbspline2(splineOrder);

	auto identity = rbspline.getManifold().getIdentity();

	std::vector<time_t> timesV;
	std::vector<typename TestSpline::point_t> interpolationPointsV;

	const auto & manifold = rbspline.getManifold();
	auto p = manifold.getIdentity();
	for(int i = 0; i < numberOfInterpolationPoints; i ++){
		timesV.push_back((time_t)times[i]);
		p = manifold.exp(p, 0.05 * TestSpline::tangent_vector_t::Random((int)manifold.getDimension()));
		interpolationPointsV.push_back(p);
	}

	BSplineFitter<TestSpline>::initUniformSplineWithKnotDelta(rbspline, timesV, interpolationPointsV, duration / numberOfSegments, lambda);
	auto knotGeneratorOrg = rbspline2.initConstantUniformSplineWithKnotDelta(minTime, maxTime, duration / numberOfSegments, identity);
	decltype(knotGeneratorOrg) knotGenerator;
	knotGenerator = knotGeneratorOrg; // test knotGenerator copying.

	ASSERT_EQ(rbspline2.getNumValidTimeSegments(), numberOfSegments);
	BSplineFitter<TestSpline>::fitSpline(rbspline2, timesV, interpolationPointsV, lambda);

	for(unsigned int i = 0; i <= numberOfTimeSteps; i ++) {
		time_t t = minTime + duration / numberOfTimeSteps * (time_t) i;
		Eigen::VectorXd rval = rbspline.template getEvaluatorAt<0>(t).eval();
		Eigen::VectorXd rval2 = rbspline2.template getEvaluatorAt<0>(t).eval();
		sm::eigen::assertEqual(rval, rval2, SM_SOURCE_FILE_POS, "");
	}

	timesV.clear();
	timesV.push_back(maxTime);

	interpolationPointsV.clear();
	typename TestSpline::point_t goal;
	typename TestSpline::tangent_vector_t translation((int)rbspline.getManifold().getDimension());
	translation.setZero();
	const double goalDist = 0.01;
	translation[0] = goalDist;
	rbspline.getManifold().expInto(p, translation, goal);
	interpolationPointsV.push_back(goal);

	BSplineFitter<TestSpline>::fitSpline(rbspline2, timesV, interpolationPointsV, 0, 0, std::function<double(int)>(), FittingBackend::DENSE, true);

	sm::eigen::assertNear(goal, rbspline2.template getEvaluatorAt<0>(maxTime).eval(), tolerance, SM_SOURCE_FILE_POS, "");
	time_t newMaxTime = maxTime + duration / numberOfSegments;
	timesV.push_back(newMaxTime);
	interpolationPointsV.push_back(goal);

	ASSERT_EQ(rbspline2.getMaxTime(), maxTime);
	BSplineFitter<TestSpline>::extendAndFitSpline(rbspline2, knotGenerator, timesV, interpolationPointsV, 0, 0, FittingBackend::DENSE, true);
	ASSERT_EQ(rbspline2.getMaxTime(), newMaxTime);

	sm::eigen::assertNear(goal, rbspline2.template getEvaluatorAt<0>(maxTime).eval(), tolerance, SM_SOURCE_FILE_POS, "");
	sm::eigen::assertNear(goal, rbspline2.template getEvaluatorAt<0>(newMaxTime).eval(), tolerance, SM_SOURCE_FILE_POS, "");

	interpolationPointsV[0] = p;
	BSplineFitter<TestSpline>::extendAndFitSpline(rbspline2, knotGenerator, timesV, interpolationPointsV, 0, 100, FittingBackend::DENSE, true);

	ASSERT_EQ(rbspline2.getMaxTime(), newMaxTime);
	sm::eigen::assertNear(goal, rbspline2.template getEvaluatorAt<0>(maxTime).eval(), tolerance, SM_SOURCE_FILE_POS, "");
	sm::eigen::assertNear(goal, rbspline2.template getEvaluatorAt<0>(newMaxTime).eval(), tolerance, SM_SOURCE_FILE_POS, "");

	BSplineFitter<TestSpline>::extendAndFitSpline(rbspline2, knotGenerator, timesV, interpolationPointsV, 0, 50, FittingBackend::DENSE, true);

	sm::eigen::assertNear(p, rbspline2.template getEvaluatorAt<0>(maxTime).eval(), goalDist * 0.7, SM_SOURCE_FILE_POS, "");
	sm::eigen::assertNear(goal, rbspline2.template getEvaluatorAt<0>(maxTime).eval(), goalDist * 0.7, SM_SOURCE_FILE_POS, "");
	sm::eigen::assertNear(goal, rbspline2.template getEvaluatorAt<0>(newMaxTime).eval(), tolerance, SM_SOURCE_FILE_POS, "");

	BSplineFitter<TestSpline>::extendAndFitSpline(rbspline2, knotGenerator, timesV, interpolationPointsV, 0, 0, FittingBackend::DENSE, true);

	sm::eigen::assertNear(p, rbspline2.template getEvaluatorAt<0>(maxTime).eval(), tolerance, SM_SOURCE_FILE_POS, "");
	sm::eigen::assertNear(goal, rbspline2.template getEvaluatorAt<0>(newMaxTime).eval(), tolerance, SM_SOURCE_FILE_POS, "");
}

}// namespace bsplines
#endif

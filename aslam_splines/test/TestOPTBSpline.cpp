/*
 * TestOPTBSpline.cpp
 *
 *  Created on: 05.08.2012
 *      Author: hannes
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include <type_traits>
#include <algorithm>
#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>
#include <bsplines/BSplineFitter.hpp>

#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/backend/JacobianContainerSparse.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/backend/Vector2RotationQuaternionExpressionAdapter.hpp>
#include <aslam/backend/ErrorTermTransformation.hpp>
#include <aslam/backend/DesignVariableMinimalDifferenceExpression.hpp>
#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/backend/test/ExpressionTests.hpp>
#include <aslam/backend/test/RotationExpressionTests.hpp>
#include <aslam/backend/GenericScalar.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/ExpressionErrorTerm.hpp>
#include <aslam/backend/VectorExpressionToGenericMatrixTraits.hpp>


//#define NO_T1
//#define NO_T2
//#define COMPILE_FAST

using namespace aslam::backend;
using namespace sm::kinematics;
using namespace aslam::splines;
using namespace bsplines;
using namespace std;

const int numSegments = 10, numberOfTimesToProbe = 30;
const double duration = 10;
const bool randomizeControlVertices = true;

template <typename TConf, int ISplineOrder, int IDim, bool BDimRequired> struct ConfCreator {
	static inline TConf create(){
		return TConf(typename TConf::ManifoldConf(IDim), ISplineOrder);
	}
};

template <typename TConf, int ISplineOrder, int IDim> struct ConfCreator<TConf, ISplineOrder, IDim, false> {
	static inline TConf create(){
		BOOST_STATIC_ASSERT_MSG(IDim == TConf::Dimension::VALUE, "impossible dimension selected!");
		return TConf(typename TConf::ManifoldConf(), ISplineOrder);
	}
};

template <typename TConf, int ISplineOrder, int IDim> inline TConf createConf(){
	return ConfCreator<TConf, ISplineOrder, IDim, TConf::Dimension::IS_DYNAMIC>::create();
}


template <typename TSplineConfig, int ISplineOrder, int IDim>
struct OPTSplineSpecializationTester
{
	typedef typename OPTBSpline<TSplineConfig>::BSpline TestBSpline;
	typedef typename TestBSpline::TimeExpression TimeExpression;
	typedef GenericScalar<typename TimeExpression::Scalar> TimeDesignVariable;
	static void test(TestBSpline & /* spline */, double /* t */, typename TestBSpline::TimeExpression /* timeExpression */, time_t /* timeExpLowerBound */, time_t /* timeExpUpperBound */){}
};


template <typename TSplineConfig, int ISplineOrder>
struct MaxDerivative {
	enum { VALUE = ISplineOrder };
};

template <typename TSpline, typename TExpression>
bool isNumericallyDifferentiableAt(const TSpline & spline, const TExpression & /* expression */, int derivativeOrder, typename TSpline::time_t t){
	if(test::ExpressionTraits<TExpression>::defaulEps() * 2 < spline.getMinimalDistanceToNeighborKnots(t)){
		return true;
	}
	else {
		return derivativeOrder < spline.getSplineOrder() - 1;
	}
}

template<typename TTime, typename TExpression>
inline void testJacobian(TExpression expression, int expectedNumberOfDesignVariables = -1, double toleranceFactor = 1.0) {
  aslam::backend::testJacobian(expression, expectedNumberOfDesignVariables, false, test::ExpressionTraits<TExpression>::defaultTolerance() * toleranceFactor, std::numeric_limits<TTime>::is_integer ?  1E-7 : test::ExpressionTraits<TExpression>::defaulEps());
}

template <typename TSplineMap, int ISplineOrder, int IDim>
struct OPTSplineTester{
	typedef typename OPTBSpline<typename TSplineMap::CONF>::BSpline TestBSpline;
	typedef typename TestBSpline::TimePolicy TimePolicy;
	typedef typename TestBSpline::time_t time_t;
	typedef typename TestBSpline::point_t point_t;
	typedef typename TestBSpline::tangent_vector_t tangent_vector_t;
	typedef typename TestBSpline::scalar_t scalar_t;
	typedef typename TestBSpline::TimeExpression TimeExpression;
	typedef GenericScalar<typename TestBSpline::TimeExpression::Scalar> TimeDesignVariable;

	constexpr static scalar_t eps = std::numeric_limits<scalar_t>::epsilon();

	inline static typename TSplineMap::CONF createMyConf() {
		return createConf<typename TSplineMap::CONF, ISplineOrder, IDim>();
	}

	OPTSplineTester() : bspline(createMyConf()) {
		bspline.initConstantUniformSpline(0, duration * TimePolicy::getOne(), numSegments, bspline.getManifold().getDefaultPoint());
		for (auto & i : bspline){
			point_t & p = i.getControlVertex();
			if(randomizeControlVertices){
				bspline.getManifold().randomizePoint(p);
			}
		}
	}

 protected:
	TestBSpline bspline;

 public:
	void testCopyAssignAndBasics(){
		TestBSpline bsplineCopyConstruct(bspline), bsplineCopyAssign(createMyConf());
		bsplineCopyAssign = bspline;

		SM_ASSERT_EQ(std::runtime_error, ISplineOrder, (int)bspline.getSplineOrder(), "");
		SM_ASSERT_EQ(std::runtime_error, IDim, (int)bspline.getDimension(), "");
	}

	void testUpdate(){
		Eigen::Matrix<double, IDim, 1> updateVec;
		updateVec.setRandom();
		double * update = &updateVec[0];

		for (typename TestBSpline::SegmentIterator i = bspline.begin(), end = bspline.end(); i != end; i++) {
			point_t & p = i->getControlVertex();
			point_t op = p;

			typename TestBSpline::dv_t & dv = *bspline.getDesignVariables(i.getKnot())[ISplineOrder - 1];
			SM_ASSERT_EQ(std::runtime_error, &dv, &i->getDesignVariable(), "");

			dv.update(update, IDim);

			point_t opu = op;
			::manifolds::internal::DiffManifoldPointUpdateTraits<typename TSplineMap::CONF::ManifoldConf>::update(bspline.getManifold(), opu, Eigen::Map<const tangent_vector_t>(update, IDim));

			sm::eigen::assertEqual(opu, p, SM_SOURCE_FILE_POS);

			dv.revertUpdate();
			sm::eigen::assertEqual(op, p, SM_SOURCE_FILE_POS);
		}
	}

	constexpr static std::uintmax_t getTimeUnit() {
		return (std::uintmax_t)TimePolicy::getOne();
	}

	void testExpressions(){
		const int pointSize = bspline.getPointSize();

		for(int k = 0; k < numberOfTimesToProbe; k++){
			static_assert(numberOfTimesToProbe > 2, "use at lease three numberOfTimesToProbe");
			time_t t = ((k % 2 == 0) ? ((double) (k / 2) / ((numberOfTimesToProbe - 1)/ 2)) : ((double) rand() / RAND_MAX)) * (duration * TimePolicy::getOne());
			const int maxDerivativeOrder = MaxDerivative<typename TSplineMap::configuration_t, ISplineOrder>::VALUE;

			TimeDesignVariable offsetVar(1000000L);
			offsetVar.setActive(true);

			typename TestBSpline::TimeExpression
					variableOffset(offsetVar.toExpression()),
					constOffset(variableOffset.evaluate()),
					timePoint(t),
					timeExpression(variableOffset - constOffset + timePoint);

			ASSERT_DOUBLE_EQ(typename TestBSpline::TimeExpression::Scalar(t), timeExpression.evaluate());

			auto fact = bspline.template getExpressionFactoryAt<maxDerivativeOrder> (t);

			time_t timeExpLowerBound = t / 2;
			time_t timeExpUpperBound = (t + bspline.getMaxTime()) / 2;

			auto factTimeExp = bspline.template getExpressionFactoryAt<maxDerivativeOrder> (timeExpression, timeExpLowerBound, timeExpUpperBound);

			typename TestBSpline::template Evaluator<maxDerivativeOrder> eval = bspline.template getEvaluatorAt <maxDerivativeOrder > (t);

			for(int derivativeOrder = 0; derivativeOrder <= maxDerivativeOrder; derivativeOrder++){
				auto expression = fact.getValueExpression(derivativeOrder);
				auto expressionTimeExp = factTimeExp.getValueExpression(derivativeOrder);

				sm::eigen::assertEqual(fact.getEvaluator().evalD(derivativeOrder), expression.evaluate(), SM_SOURCE_FILE_POS);
				sm::eigen::assertEqual(eval.evalD(derivativeOrder), expression.evaluate(), SM_SOURCE_FILE_POS);
				sm::eigen::assertEqual(factTimeExp.getEvaluator().evalD(derivativeOrder), expressionTimeExp.evaluate(), SM_SOURCE_FILE_POS);
				sm::eigen::assertNear(expressionTimeExp.evaluate(), expression.evaluate(), 16 * eps, SM_SOURCE_FILE_POS);

				JacobianContainerSparse<> jac(pointSize);
				DesignVariable::set_t set;
				expression.getDesignVariables(set);

				std::vector<DesignVariable *> varVec = bspline.getDesignVariables(t);
				ASSERT_EQ(set.size(), varVec.size());

				DesignVariable::set_t setTimeExp;
				expressionTimeExp.getDesignVariables(setTimeExp);

				ASSERT_LT(set.size(), setTimeExp.size()); // at least the time variable is also there

				int dvIndexCounter = 0;
				for(auto dv : varVec){
					dv->setActive(true);
					dv->setBlockIndex(dvIndexCounter++);
					ASSERT_EQ(set.count(dv), 1);
					ASSERT_EQ(setTimeExp.count(dv), 1);
				}

				ASSERT_EQ(setTimeExp.count(&offsetVar), 1);

				expression.evaluateJacobians(jac);

				typename TestBSpline::full_jacobian_t J;
				eval.evalJacobian(derivativeOrder, J);
				sm::eigen::assertEqual(J, jac.asDenseMatrix(), SM_SOURCE_FILE_POS);
				fact.getEvaluator().evalJacobian(derivativeOrder, J);
				sm::eigen::assertEqual(J, jac.asDenseMatrix(), SM_SOURCE_FILE_POS);

				if(isNumericallyDifferentiableAt(bspline, expression, derivativeOrder, t))
				{
					SCOPED_TRACE(::testing::Message() << "derivativeOrder=" << derivativeOrder << ", t=" << t);
					testJacobian(expression, dvIndexCounter);
				}

				for(auto dv : setTimeExp){
					if(!set.count(dv)){
						dv->setActive(true);
						dv->setBlockIndex(dvIndexCounter++);
						varVec.push_back(dv);
					}
				}

				if(isNumericallyDifferentiableAt(bspline, expressionTimeExp, derivativeOrder + 1, t) && t != bspline.getMaxTime() && t != bspline.getMinTime()){
					SCOPED_TRACE(::testing::Message() << "derivativeOrder=" << derivativeOrder << ", t=" << t);
					testJacobian<time_t>(expressionTimeExp, dvIndexCounter, 2);
				}

				{
					SCOPED_TRACE(::testing::Message() << "derivativeOrder=" << derivativeOrder << ", t=" << t);
					OPTSplineSpecializationTester<typename TSplineMap::CONF, ISplineOrder, IDim>::test(bspline, t, timeExpression, timeExpLowerBound, timeExpUpperBound);
				}

				for(auto dv : varVec){
					dv->setActive(false);
					dv->setBlockIndex(-1);
				}
			}
		}
	}

	void testMinimalDifference(){
		auto manifold = bspline.getManifold();
		DesignVariable& dv = *bspline.getDesignVariables()[0];
		dv.setActive(true);
		dv.setBlockIndex(0);

		for(int i = 0; i < 10; i ++){
			Eigen::MatrixXd startPoint = manifold.getRandomPoint();
			Eigen::VectorXd updateVector = Eigen::VectorXd::Random((int)manifold.getDimension());

			dv.setParameters(startPoint);
			dv.update(&updateVector[0], updateVector.size());
			Eigen::VectorXd vector;
			dv.minimalDifference(startPoint, vector);
			sm::eigen::assertNear(vector, updateVector, eps * 4, SM_SOURCE_FILE_POS);

			DesignVariableMinimalDifferenceExpression<IDim> dvMDE(dv, startPoint);
			sm::eigen::assertEqual(dvMDE.evaluate(), vector, SM_SOURCE_FILE_POS);
			testJacobian(dvMDE);
		}
	}
};

template <typename TDiffManifoldConfiguration, int IEigenSplineOrder, typename TTimePolicy, int ISplineOrder>
struct MaxDerivative<UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, IEigenSplineOrder, TTimePolicy>, ISplineOrder> {
	enum { LIMIT=UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, IEigenSplineOrder, TTimePolicy>::BSpline::MaxSupportedDerivativeOrderJacobian, VALUE = LIMIT < ISplineOrder ? LIMIT : ISplineOrder };
};

template <typename TDiffManifoldConfiguration, int IEigenSplineOrder, typename TTimePolicy, int ISplineOrder, int IDim>
struct OPTSplineSpecializationTester<UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, IEigenSplineOrder, TTimePolicy>, ISplineOrder, IDim>
{
	typedef UnitQuaternionBSplineConfiguration<TDiffManifoldConfiguration, IEigenSplineOrder, TTimePolicy> CONF;
	typedef typename OPTBSpline<CONF>::BSpline TestBSpline;
	typedef typename TestBSpline::time_t time_t;
	typedef GenericScalar<typename TestBSpline::TimeExpression::Scalar> TimeDesignVariable;

	static void test(TestBSpline & bspline, time_t t, typename TestBSpline::TimeExpression timeExpression, time_t timeExpLowerBound, time_t timeExpUpperBound){

		auto fact = bspline.template getExpressionFactoryAt < 2 > (t);
		auto factTimeExp = bspline.template getExpressionFactoryAt<3> (timeExpression, timeExpLowerBound, timeExpUpperBound);

		auto avexpression = fact.getAngularVelocityExpression();
		auto aaexpression = fact.getAngularAccelerationExpression();
		auto avexpressionTE = factTimeExp.getAngularVelocityExpression();
		auto aaexpressionTE = factTimeExp.getAngularAccelerationExpression();

		typename TestBSpline::template Evaluator<2> eval = bspline.template getEvaluatorAt < 2 > (t);
		sm::eigen::assertEqual(eval.evalAngularVelocity(), avexpression.evaluate(), SM_SOURCE_FILE_POS);
		sm::eigen::assertEqual(eval.evalAngularAcceleration(), aaexpression.evaluate(), SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(eval.evalAngularVelocity(), avexpressionTE.evaluate(), 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(eval.evalAngularAcceleration(), aaexpressionTE.evaluate(), 1E-9, SM_SOURCE_FILE_POS);

		JacobianContainerSparse<> jac(3);
		typename TestBSpline::angular_jacobian_t J;

		avexpression.evaluateJacobians(jac);
		eval.evalAngularVelocityJacobian(J);
		sm::eigen::assertEqual(J, jac.asDenseMatrix(), SM_SOURCE_FILE_POS);

		jac.clear();
		aaexpression.evaluateJacobians(jac);
		eval.evalAngularAccelerationJacobian(J);
		sm::eigen::assertEqual(J, jac.asDenseMatrix(), SM_SOURCE_FILE_POS);

		if(isNumericallyDifferentiableAt(bspline, avexpression, 1, t))
		{
			SCOPED_TRACE(""); testJacobian(avexpression, bspline.getSplineOrder());
		}

		if(isNumericallyDifferentiableAt(bspline, aaexpression, 2, t))
		{
			SCOPED_TRACE(""); testJacobian(aaexpression, bspline.getSplineOrder());
		}

		if(t != bspline.getMaxTime() && t != bspline.getMinTime()){
			int expectedNumberOfDVs = bspline.getSplineOrder() + std::distance(bspline.getSegmentIterator(timeExpLowerBound), bspline.getSegmentIterator(timeExpUpperBound)) + 1; // number of control vertices relevant for the interval + 1 for the time design variable!
			if(isNumericallyDifferentiableAt(bspline, avexpressionTE, 2, t))
			{
				SCOPED_TRACE(""); testJacobian<time_t>(avexpressionTE, expectedNumberOfDVs, 2);
			}
			if(isNumericallyDifferentiableAt(bspline, aaexpressionTE, 3, t))
			{
				SCOPED_TRACE(""); testJacobian<time_t>(aaexpressionTE, expectedNumberOfDVs, 2);
			}
		}
	}

};


#ifndef NO_T1

template <typename Tester>
class OPTBSplineTestSuiteT : public ::testing::Test  {
 protected:
	Tester tester;
};

typedef ::testing::Types<
#ifndef COMPILE_FAST
	OPTSplineTester<EuclideanBSpline<2, Eigen::Dynamic>, 2, 3> ,
	OPTSplineTester<EuclideanBSpline<3, 2>, 3, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 3, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 3, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 4, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 4, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 5, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 5, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 6, 2>,
#endif
//	OPTSplineTester<EuclideanBSpline<2, 1>, 2, 1>
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 6, 2>,
	OPTSplineTester<UnitQuaternionBSpline<2>, 2, 3>
#ifndef COMPILE_FAST
	,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic>, 2, 3>,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>, 2, 3>,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic>, 3, 3>,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>, 3, 3>,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic>, 4, 3>,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>, 4, 3>,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic>, 5, 3>,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>, 5, 3>,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic>, 8, 3>,
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic>, 9, 3>
#endif
> Testers;

TYPED_TEST_CASE(OPTBSplineTestSuiteT, Testers);

TYPED_TEST(OPTBSplineTestSuiteT, update) {
	try{
		SCOPED_TRACE(""); this->tester.testUpdate();
	}
	catch(const std::exception & e)
	{
		FAIL() << e.what();
	}
}

TYPED_TEST(OPTBSplineTestSuiteT, expressions) {
	try{
		SCOPED_TRACE("");this->tester.testExpressions();
	}
	catch(const std::exception & e)
	{
		FAIL() << e.what();
	}
}

TYPED_TEST(OPTBSplineTestSuiteT, minimalDifferences) {
	try{
		SCOPED_TRACE(""); this->tester.testMinimalDifference();
	}
	catch(const std::exception & e)
	{
		FAIL() << e.what();
	}

}

TEST(OPTBSplineTestSuite, testPoseErrorWithOPTSplines)
{
	try {
		using namespace aslam::backend;
		sm::kinematics::Transformation T_random;
		T_random.setRandom(0.05, 0.01);
		sm::kinematics::Transformation T_prior;

		OPTBSpline<typename UnitQuaternionBSpline<2>::CONF>::BSpline testSpline;

		testSpline.initConstantUniformSpline(0, 10, 2, quatRandom());

		double t = 5;
		auto valueExpression = testSpline.getExpressionFactoryAt<0>(t).getValueExpression();
		{
			SCOPED_TRACE("");
			testExpression(valueExpression, testSpline.getSplineOrder());
		}

		RotationExpression rexp(Vector2RotationQuaternionExpressionAdapter::adapt(VectorExpression<4>(valueExpression)));
		sm::eigen::assertEqual(sm::kinematics::quat2r(valueExpression.toValue()), rexp.toRotationMatrix(), SM_SOURCE_FILE_POS);

		for(int i = 0 ; i < 3 ; i++)
		{
			SCOPED_TRACE("");
			Eigen::Vector3d v;
			v.setZero();
			v(i) = 1;
			testJacobian(rexp * EuclideanExpression(v));
		}

		{
			SCOPED_TRACE("");
			testJacobian(rexp);
		}

		EuclideanPoint ep(T_prior.t());
		EuclideanExpression eexp(&ep);

		TransformationBasic Tb(rexp, eexp);
		TransformationExpression T(&Tb);

		HomogeneousExpression he(Eigen::Vector4d::Random().eval());
		{
			SCOPED_TRACE(""); testExpression(T * he, testSpline.getSplineOrder() + 1);
		}

		Eigen::MatrixXd N = Eigen::MatrixXd::Zero(6,6);
		N(0,0) = 1e-3;
		N(1,1) = 1e-3;
		N(2,2) = 1e-3;
		N(3,3) = 1e0;
		N(4,4) = 1e0;
		N(5,5) = 1e0;

		// Create the ErrorTerm
		ErrorTermTransformation ett(T, T_random, N);
		// Create the test harness
		aslam::backend::ErrorTermTestHarness<6> harness(&ett);

		// Run the unit tests.
		{
			SCOPED_TRACE("");
			harness.testAll(1e-5);
		}
	}
	catch(const std::exception & e)
	{
		FAIL() << e.what();
	}
}

TEST(OPTBSplineTestSuite, testAppendingUpdatesDesignVariableList)
{
	try {
		OPTBSpline<EuclideanBSpline<3, 2>::CONF>::BSpline testSpline;

		auto zero = Eigen::Vector2d::Zero();
		testSpline.initConstantUniformSpline(0, 1, 2, zero);
		const int before = testSpline.getDesignVariables().size();
		testSpline.appendSegmentsUniformly(1);

		ASSERT_EQ(before + 1, testSpline.getDesignVariables().size());
	}
	catch(const std::exception & e)
	{
		FAIL() << e.what();
	}
}

#endif

#ifndef NO_T2

template <typename Tester>
class OPTBSplineTestSuiteT2 : public ::testing::Test  {
 public:
	void testOptimizingEvaluationTimeSmall(){
		typedef typename Tester::TestBSpline TestBSpline;
		typedef typename TestBSpline::TimePolicy TimePolicy;
		TestBSpline testSpline(Tester::createMyConf());
		typedef typename TimePolicy::time_t time_t;
		const time_t endTime = 4 * TimePolicy::getOne(), halfTime = endTime / 2;

		auto zero = Eigen::Vector2d::Zero();
		auto one = Eigen::Vector2d::Ones();


		testSpline.initConstantUniformSpline(0, endTime, testSpline.getSplineOrder(), zero);
		auto seg = testSpline.getSegmentIterator(0);
		seg->setControlVertex(one);
		ASSERT_EQ(0, seg->getKnot());
		sm::eigen::assertNear(zero, testSpline.template getEvaluatorAt<1>(halfTime).evalD(1), 1E-8, SM_SOURCE_FILE_POS);

		OptimizationProblem problem;
		testSpline.addDesignVariables(problem);

		typename Tester::TimeDesignVariable timeVar(endTime/4);
		problem.addDesignVariable(&timeVar, false);
		timeVar.setActive(true);

		problem.addErrorTerm(toErrorTerm(convertToGME(testSpline.template getExpressionFactoryAt<0>(timeVar.toExpression(), 0, endTime).getValueExpression(0)) - GenericMatrixExpression<2, 1>(testSpline.template getEvaluatorAt<0>(halfTime).eval())));

		Optimizer opt;
		opt.setProblem(&problem, false);
		opt.options().verbose = false;
		opt.options().convergenceDeltaJ = 0;
		opt.options().convergenceDeltaX = 1E-9;
		opt.optimize();

		ASSERT_NEAR(typename Tester::TimeDesignVariable::Scalar(halfTime), timeVar.toScalar(), 1E-5);
	}

	void testOptimizingEvaluationTimeLarge(){
		typedef typename Tester::TestBSpline TestBSpline;
		typedef typename TestBSpline::TimePolicy TimePolicy;
		TestBSpline groundTrouth(Tester::createMyConf());
		TestBSpline estimate(Tester::createMyConf());
		typedef typename TimePolicy::time_t time_t;
		typedef typename Tester::TimeDesignVariable::Scalar TimeScalar;

		typedef Eigen::Vector2d Vec;
		const double duration = 10;

		const time_t endTime = duration * TimePolicy::getOne();

		auto zero = Vec::Zero();

		groundTrouth.initConstantUniformSpline(0, endTime, groundTrouth.getSplineOrder(), zero);
		estimate.initConstantUniformSpline(0, endTime, estimate.getSplineOrder(), zero);

		const int numPoints = 1000;
		const double initNoiseMagnitude = 0.1;
		const double velNoiseMagnitude = 0.0001;
		const double posNoiseMagnitude = 0.001;

		std::vector<time_t> times;
		std::vector<Vec> points;
		std::vector<Vec> velMeasurements;
		for(int i = 0; i < numPoints; i++){
			times.push_back(TestBSpline::TimePolicy::linearlyInterpolate(0, endTime, numPoints, i));
			double phase = M_PI * i / (numPoints -1);
			points.push_back(Vec(cos(phase), sin(phase)));
		}
		BSplineFitter<TestBSpline>::fitSpline(groundTrouth, times, points, 0.01);

		OptimizationProblem problem;

		const double realDelay = 0.1;
		typename Tester::TimeDesignVariable timeVar(TimeScalar(0.0));

		problem.addDesignVariable(&timeVar, false);
		timeVar.setActive(true);

		estimate.addDesignVariables(problem);
		for(auto & dv : estimate.getDesignVariables()){
			dv->setActive(true);
		}

		int numPointsTooCloseToEachBound = std::ceil(realDelay/ (duration / numPoints));

		for(int i = 0; i < numPoints; i++){
			points[i] += Vec::Random() * initNoiseMagnitude;
			if(i > numPointsTooCloseToEachBound && i < numPoints - numPointsTooCloseToEachBound){ // error terms too close to the bounds might hit the spline bounds
				Vec posMeasurement = groundTrouth.template getEvaluatorAt<0>(times[i]).evalD(0) + Vec::Random() * posNoiseMagnitude;
				problem.addErrorTerm(toErrorTerm(convertToGME(estimate.template getExpressionFactoryAt<0>(times[i]).getValueExpression(0)) - GenericMatrixExpression<2, 1>(posMeasurement), Eigen::Matrix2d::Identity() / posNoiseMagnitude));

				Vec velMeasurement = groundTrouth.template getEvaluatorAt<1>(times[i]).evalD(1) + Vec::Random() * velNoiseMagnitude;
				problem.addErrorTerm(toErrorTerm(convertToGME(estimate.template getExpressionFactoryAt<1>(timeVar.toExpression() + GenericScalarExpression<TimeScalar>(TimeScalar(times[i]) - TimeScalar(realDelay)), 0, endTime).getValueExpression(1)) - GenericMatrixExpression<2, 1>(velMeasurement), Eigen::Matrix2d::Identity() / velNoiseMagnitude));
			}
		}
		BSplineFitter<TestBSpline>::fitSpline(estimate, times, points, 0.01);

		auto exp = timeVar.toExpression() - GenericScalarExpression<TimeScalar>(TimeScalar(0.0));
		auto timeModelError = toErrorTerm(exp, Eigen::Matrix<double, 1, 1>::Ones() / realDelay);
		problem.addErrorTerm(timeModelError);


		Optimizer opt;
		opt.setProblem(&problem, false);
		opt.options().verbose = false;
		opt.options().convergenceDeltaJ = 0;
		opt.options().convergenceDeltaX = 1E-9;
		opt.options().maxIterations = 20;
		opt.optimize();

		ASSERT_NEAR(TimeScalar(realDelay), timeVar.toScalar(), 1E-3);
	}
};

typedef ::testing::Types<
#ifndef COMPILE_FAST
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 4, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 4, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 5, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 5, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 6, 2>,
#endif
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 6, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 6, 2>
> Testers2;
TYPED_TEST_CASE(OPTBSplineTestSuiteT2, Testers2);

TYPED_TEST(OPTBSplineTestSuiteT2, testOptimizingEvaluationTimeSmall)
{
	try {
		SCOPED_TRACE(""); this->testOptimizingEvaluationTimeSmall();
	}
	catch(const std::exception & e)
	{
		FAIL() << e.what();
	}
}

TYPED_TEST(OPTBSplineTestSuiteT2, testOptimizingEvaluationTimeLarge)
{
	try {
		SCOPED_TRACE(""); this->testOptimizingEvaluationTimeLarge();
	}
	catch(const std::exception & e)
	{
		FAIL() << e.what();
	}
}

#endif

#if __cplusplus >= 201103L
TEST(OPTBSplineTestSuite, cpp11ConfortAndBackCompatibility)
{
	typedef OPTBSpline<typename EuclideanBSpline<1, 3>::CONF> Spline;
	Spline::BSpline(Spline::CONF::Conf(Spline::CONF::ManifoldConf(3), 1)); // old way needs to be still possible
	Spline::BSpline(Spline::CONF(Spline::CONF::ManifoldConf(3), 1)); // better thanks to constructor inheritance in DesignVariableSegmentBSplineConf
	Spline::BSpline(1, 3); // better thanks to constructor inheritance in OPTBSpline

	// awesome: thanks to the new type aliasing nature of OPTBSpline
	Spline(1, 3);
	Spline(1); // even inheriting default constructor arguments
}
#endif

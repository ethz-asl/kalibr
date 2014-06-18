/*
 * TestOPTBSpline.cpp
 *
 *  Created on: 05.08.2012
 *      Author: hannes
 */

#include <type_traits>
#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <bsplines/NsecTimePolicy.hpp>

#include <aslam/backend/TransformationBasic.hpp>
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


using namespace aslam::backend;
using namespace sm::kinematics;
using namespace aslam::splines;
using namespace bsplines;
using namespace std;

const int numSegments = 3, numberOfTimesToProbe = 10;
const double duration = numSegments / 2.0;
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


template <typename TSplineMap, int ISplineOrder, int IDim>
struct OPTSplineSpecializationTester
{
	typedef typename OPTBSpline<typename TSplineMap::CONF>::BSpline TestBSpline;
	typedef typename TestBSpline::TimeExpression TimeExpression;
	typedef GenericScalar<typename TimeExpression::Scalar> TimeDesignVariable;
	static void test(TestBSpline & /* spline */, double /* t */, typename TestBSpline::TimeExpression /* timeExpression */){}
};


template <typename TSplineMap, int ISplineOrder>
struct MaxDerivative {
	enum { VALUE = ISplineOrder };
};


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
			time_t t = ((k % 2 == 0) ? ((double) (k / 2) / ((numberOfTimesToProbe - 1)/ 2)) : ((double) rand() / RAND_MAX)) * (duration * TimePolicy::getOne());
			const int maxDerivativeOrder = MaxDerivative<TSplineMap, ISplineOrder>::VALUE;

			TimeDesignVariable offsetVar(1000000);
			offsetVar.setActive(true);

			typename TestBSpline::TimeExpression
					variableOffset(offsetVar.toExpression()),
					constOffset(variableOffset.evaluate()),
					timePoint(t),
					timeExpression(variableOffset - constOffset + timePoint);

			ASSERT_DOUBLE_EQ(typename TestBSpline::TimeExpression::Scalar(t), timeExpression.evaluate());

			auto fact = bspline.template getExpressionFactoryAt<maxDerivativeOrder> (t);
			auto factTimeExp = bspline.template getExpressionFactoryAt<maxDerivativeOrder> (timeExpression, t / 2,  t == bspline.getMaxTime() ? bspline.getMaxTime() + 1: (t + bspline.getMaxTime()) / 2);

			typename TestBSpline::template Evaluator<maxDerivativeOrder> eval = bspline.template getEvaluatorAt <maxDerivativeOrder > (t);

			for(int derivativeOrder = 0; derivativeOrder <= maxDerivativeOrder; derivativeOrder++){
				auto expression = fact.getValueExpression(derivativeOrder);
				auto expressionTimeExp = factTimeExp.getValueExpression(derivativeOrder);

				sm::eigen::assertEqual(fact.getEvaluator().evalD(derivativeOrder), expression.evaluate(), SM_SOURCE_FILE_POS);
				sm::eigen::assertEqual(eval.evalD(derivativeOrder), expression.evaluate(), SM_SOURCE_FILE_POS);
				sm::eigen::assertEqual(factTimeExp.getEvaluator().evalD(derivativeOrder), expressionTimeExp.evaluate(), SM_SOURCE_FILE_POS);
				sm::eigen::assertNear(expressionTimeExp.evaluate(), expression.evaluate(), 16 * eps, SM_SOURCE_FILE_POS);

				JacobianContainer jac(pointSize);
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

				{
					SCOPED_TRACE("");
					testJacobian(expression, dvIndexCounter);
				}

				if(t != bspline.getMaxTime() && t != bspline.getMinTime()){
					for(auto dv : setTimeExp){
						if(!set.count(dv)){
							dv->setActive(true);
							dv->setBlockIndex(dvIndexCounter++);
							varVec.push_back(dv);
						}
					}
					SCOPED_TRACE("");
					testJacobian(expressionTimeExp, dvIndexCounter, false, test::ExpressionTraits<decltype(expressionTimeExp)>::defaultTolerance(), std::numeric_limits<time_t>::is_integer ?  1E-7 : test::ExpressionTraits<decltype(expressionTimeExp)>::defaulEps());
				}

				{
					SCOPED_TRACE("");
					OPTSplineSpecializationTester<TSplineMap, ISplineOrder, IDim>::test(bspline, t, timeExpression);
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

template <int IEigenSplineOrder, int ISplineOrder, typename TTimePolicy, typename TScalar>
struct MaxDerivative<UnitQuaternionBSpline<IEigenSplineOrder, TTimePolicy, TScalar>, ISplineOrder> {
	enum { LIMIT=UnitQuaternionBSpline<IEigenSplineOrder>::TYPE::MaxSupportedDerivativeOrderJacobian, VALUE = LIMIT < ISplineOrder ? LIMIT : ISplineOrder };
};

template <int IEigenSplineOrder, int ISplineOrder, int IDim>
struct OPTSplineSpecializationTester<UnitQuaternionBSpline<IEigenSplineOrder>, ISplineOrder, IDim>
{
	typedef typename OPTBSpline<typename UnitQuaternionBSpline<IEigenSplineOrder>::CONF>::BSpline TestBSpline;
	typedef GenericScalar<typename TestBSpline::TimeExpression::Scalar> TimeDesignVariable;

	static void test(TestBSpline & bspline, double t, typename TestBSpline::TimeExpression timeExpression){

		auto fact = bspline.template getExpressionFactoryAt < 2 > (t);
		auto factTimeExp = bspline.template getExpressionFactoryAt<3> (timeExpression, t / 2,  t == bspline.getMaxTime() ? bspline.getMaxTime() + 1: (t + bspline.getMaxTime()) / 2);

		auto avexpression = fact.getAngularVelocityExpression();
		auto aaexpression = fact.getAngularAccelerationExpression();
		auto avexpressionTE = factTimeExp.getAngularVelocityExpression();
		auto aaexpressionTE = factTimeExp.getAngularAccelerationExpression();

		typename TestBSpline::template Evaluator<2> eval = bspline.template getEvaluatorAt < 2 > (t);
		sm::eigen::assertEqual(eval.evalAngularVelocity(), avexpression.evaluate(), SM_SOURCE_FILE_POS);
		sm::eigen::assertEqual(eval.evalAngularAcceleration(), aaexpression.evaluate(), SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(eval.evalAngularVelocity(), avexpressionTE.evaluate(), 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(eval.evalAngularAcceleration(), aaexpressionTE.evaluate(), 1E-9, SM_SOURCE_FILE_POS);

		JacobianContainer jac(3);
		typename TestBSpline::angular_jacobian_t J;

		avexpression.evaluateJacobians(jac);
		eval.evalAngularVelocityJacobian(J);
		sm::eigen::assertEqual(J, jac.asDenseMatrix(), SM_SOURCE_FILE_POS);

		jac.clear();
		aaexpression.evaluateJacobians(jac);
		eval.evalAngularAccelerationJacobian(J);
		sm::eigen::assertEqual(J, jac.asDenseMatrix(), SM_SOURCE_FILE_POS);

		{
			SCOPED_TRACE("");
			testJacobian(avexpression);
		}
		{
			SCOPED_TRACE("");
			testJacobian(aaexpression);
		}

		if(t != bspline.getMaxTime() && t != bspline.getMinTime()){
			{
				SCOPED_TRACE("");
				testJacobian(avexpressionTE);
			}
			{
				SCOPED_TRACE("");
				testJacobian(aaexpressionTE);
			}
		}
	}
};

template <typename Tester>
class OPTBSplineTestSuiteT : public ::testing::Test  {
 protected:
	Tester tester;
};

//#define COMPILE_FAST

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
	OPTSplineTester<UnitQuaternionBSpline<Eigen::Dynamic, NsecTimePolicy>, 5, 3>
#endif
> Testers;

TYPED_TEST_CASE(OPTBSplineTestSuiteT, Testers);

TYPED_TEST(OPTBSplineTestSuiteT, update) {
	SCOPED_TRACE("");
	this->tester.testUpdate();
}

TYPED_TEST(OPTBSplineTestSuiteT, expressions) {
	SCOPED_TRACE("");
	this->tester.testExpressions();
}

TYPED_TEST(OPTBSplineTestSuiteT, minimalDifferences) {
	SCOPED_TRACE("");
	this->tester.testMinimalDifference();
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

template <typename Tester>
class OPTBSplineTestSuiteT2 : public ::testing::Test  {
};

typedef ::testing::Types<
#ifndef COMPILE_FAST
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 4, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 4, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 5, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 5, 2>,
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2>, 6, 2>,
#endif
	OPTSplineTester<EuclideanBSpline<Eigen::Dynamic, 2, NsecTimePolicy>, 6, 2>
> Testers2;
TYPED_TEST_CASE(OPTBSplineTestSuiteT2, Testers2);

TYPED_TEST(OPTBSplineTestSuiteT2, testOptimizingEvaluationTime)
{
	try {

		typedef typename TypeParam::TestBSpline TestBSpline;
		typedef typename TestBSpline::TimePolicy TimePolicy;
		TestBSpline testSpline(TypeParam::createMyConf());
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
		for(auto & dv : testSpline.getDesignVariables()){
			problem.addDesignVariable(dv, false);
		}

		typename TypeParam::TimeDesignVariable timeVar(typename TypeParam::TimeDesignVariable(endTime/4));
		problem.addDesignVariable(&timeVar, false);
		timeVar.setActive(true);

		problem.addErrorTerm(toErrorTerm(convertToGME(testSpline.template getExpressionFactoryAt<0>(timeVar.toExpression(), 0, endTime).getValueExpression(0)) - GenericMatrixExpression<2, 1>(testSpline.template getEvaluatorAt<0>(halfTime).eval())));

		Optimizer opt;
		opt.setProblem(&problem, false);
		opt.options().verbose = false;
		opt.options().convergenceDeltaJ = 0;
		opt.options().convergenceDeltaX = 1E-9;
		opt.optimize();

		ASSERT_NEAR(typename TypeParam::TimeDesignVariable::Scalar(halfTime), timeVar.toScalar(), 1E-5);
	}
	catch(const std::exception & e)
	{
		FAIL() << e.what();
	}
}

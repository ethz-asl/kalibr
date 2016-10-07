#include "DiffManifoldBSplineTests.hpp"
#include <algorithm>

namespace bsplines {


TEST(UnitQuaternionBSplineTestSuite, testQuaternionBSplineCompilation)
{
	UnitQuaternionBSpline<splineOrder>::TYPE rbspline;
	const UnitQuaternionBSpline<splineOrder>::TYPE::point_t p = UnitQuaternionBSpline<splineOrder>::TYPE::point_t(1, 0, 0, 0);
	rbspline.initConstantUniformSpline(minTime, maxTime, numberOfSegments, p);
	sm::eigen::assertEqual(rbspline.getEvaluatorAt<0>(minTime).eval(), p, SM_SOURCE_FILE_POS);
}

TEST(UnitQuaternionBSplineTestSuite, differentEvalMethodsEvalTheSame)
{
	UQTestSpline spline;
	UQTestSplineD splineD(splineOrder);

	initMinimalSpline(spline);
	initMinimalSpline(splineD);

	UQTestSpline::point_t p;
	for(int i = 0, n = knot_arithmetics::getNumControlVerticesRequired(2, splineOrder) ; i < n; i ++){
		spline.getManifold().randomizePoint(p);
		spline.addControlVertex(i, p);
		splineD.addControlVertex(i, p);
	}

	UQTestSpline::full_jacobian_t jac1, jac2;
	UQTestSplineD::full_jacobian_t jac3;

	for(int i = 0, n = 10; i< n; i ++){
		double t = (spline.getMaxTime() - spline.getMinTime()) / (n - 1) * i + spline.getMinTime();
		UQTestSpline::Evaluator<splineOrder + 1> eval = spline.getEvaluatorAt<splineOrder + 1>(t);
		UQTestSplineD::Evaluator<splineOrder + 1> eval2 = splineD.getEvaluatorAt<splineOrder + 1>(t);

//		std::cout << std::endl << "t = "<< t << std::endl;
		sm::eigen::assertNear(eval.evalDRecursive(0), eval.evalGeneric(), 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(eval.evalDRecursive(1), eval.evalD1Special(), 1E-9, SM_SOURCE_FILE_POS);

		for(unsigned i = 0 ;  i < std::min(splineOrder + 1, unsigned(4)); i ++){
			sm::eigen::assertNear(eval.evalDRecursive(i), eval2.evalDRecursive(i), 0, SM_SOURCE_FILE_POS);
		}
		for(unsigned i = 0 ;  i < std::min(splineOrder + 1, unsigned(3)); i ++){
		//compare generic recursive computation of Jacobian with the optimized one.s
			eval.evalJacobianDRecursive(i, jac1);
			eval.evalJacobian(i, jac2);
			eval2.evalJacobian(i, jac3);
			sm::eigen::assertNear(jac1, jac2, 0, SM_SOURCE_FILE_POS);
			sm::eigen::assertNear(jac2, jac3, 0, SM_SOURCE_FILE_POS);
		}
	}
}

TEST(UnitQuaternionBSplineTestSuite, testDExp)
{
	DExpTester<UnitQuaternionManifoldConf<>::Manifold >::testFunc(100, 10);
}

TEST(UnitQuaternionBSplineTestSuite, evalRiD1)
{
	SplineEvalRiDTester<UQTestSpline, 1>::testFunc(10, 10);
}
TEST(UnitQuaternionBSplineTestSuite, evalRiD2)
{
	SplineEvalRiDTester<UQTestSpline, 2>::testFunc(10, 10);
}
TEST(UnitQuaternionBSplineTestSuite, evalRiD3)
{
	SplineEvalRiDTester<UQTestSpline, 3>::testFunc(10, 10);
}

TEST(UnitQuaternionBSplineTestSuite, evalD1)
{
	SplineEvalDTester<UQTestSpline, 1>::testFunc(10, 10);
}

TEST(UnitQuaternionBSplineTestSuite, evalD2)
{
	SplineEvalDTester<UQTestSpline, 2>::testFunc(10, 10);
}

TEST(UnitQuaternionBSplineTestSuite, evalD3)
{
	SplineEvalDTester<UQTestSpline, 3>::testFunc(10, 10);
}

// TODO implement generic angular velocity tests
TEST(UnitQuaternionBSplineTestSuite, evalAngularVelocityAndAcceleration)
{
	UQTestSpline spline;
	Eigen::Vector3d direction(asin(1) / 2, 0, 0);

	const int numPoints = 4 + splineOrder - 1;

	Eigen::MatrixXd interpolationPointsE = Eigen::MatrixXd::Zero(4, numPoints);
	Eigen::VectorXd timesE = Eigen::VectorXd::Zero(numPoints);

	for(int i = -splineOrder + 1; i < numPoints; i++){
		UQTestSpline::point_t p = spline.getManifold().expAtId(direction * (i + splineOrder - 1));
		spline.addControlVertex(i, p);
	}

	spline.init();
	assert(spline.getMinTime() == 0.0);

	Eigen::VectorXd accel = Eigen::VectorXd::Zero(3);

	const int numTimes = numPoints * 5;
	for(int i = 0; i < numTimes; i ++){
		double t = spline.getMinTime() + (spline.getMaxTime() - spline.getMinTime()) * (double) i / (numTimes - 1);
		UQTestSpline::Evaluator<2> eval = spline.getEvaluatorAt<2>(t);

		if(splineOrder == 2){
			sm::eigen::assertNear(eval.eval(), spline.getManifold().expAtId(direction * t), 1E-9, SM_SOURCE_FILE_POS);
			sm::eigen::assertNear(spline.getEvaluatorAt<0>(t).eval(), spline.getManifold().expAtId(direction * t), 1E-9, SM_SOURCE_FILE_POS);
		}

		sm::eigen::assertNear(eval.evalAngularVelocity(), direction, 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(eval.evalAngularAcceleration(), accel, 1E-9, SM_SOURCE_FILE_POS);
	}
}


TEST(UnitQuaternionBSplineTestSuite, SplineEvalRiDJacobianTesterD0)
{
	SplineEvalRiDJacobianTester<UQTestSpline, 0>::testFunc(10, 1);
}
TEST(UnitQuaternionBSplineTestSuite, SplineEvalRiDJacobianTesterD1)
{
	SplineEvalRiDJacobianTester<UQTestSpline, 1>::testFunc(10, 1);
}
TEST(UnitQuaternionBSplineTestSuite, SplineEvalRiDJacobianTesterD2)
{
	SplineEvalRiDJacobianTester<UQTestSpline, 2>::testFunc(10, 1);
}

// Check the Jacobian calculation.
TEST(UnitQuaternionBSplineTestSuite, testBSplineJacobianD0)
{
	BSplineJacobianTester<UQTestSpline, 0>::testFunc(10, 1);
}
TEST(UnitQuaternionBSplineTestSuite, testBSplineJacobianD1)
{
	BSplineJacobianTester<UQTestSpline, 1>::testFunc(10, 1);
}
TEST(UnitQuaternionBSplineTestSuite, testBSplineJacobianD2)
{
	BSplineJacobianTester<UQTestSpline, 2>::testFunc(10, 1);
}

TEST(UnitQuaternionBSplineTestSuite, testAngularVelocitiyJacobian)
{
	AngularDerivativeJacobianTestser<UQTestSpline, 1>::testFunc(10, 1);
}
TEST(UnitQuaternionBSplineTestSuite, testAngularAccelerationJacobian)
{
	AngularDerivativeJacobianTestser<UQTestSpline, 2>::testFunc(10, 1);
}

TEST(UnitQuaternionBSplineTestSuite, testDiffManifoldBSplineFitting)
{
	double tolerance = 0.3; //TODO improve : the unit quaternion fitting is quite bad. this huge tolerance actually checks almost nothing apart from compilation and running without exceptions.
	testDiffManifoldBSplineFitting<UQTestSpline>(numberOfSegments, tolerance);
	testDiffManifoldBSplineFitting<UQTestSpline>(numberOfSegments * 2, tolerance);
}

} // namespace bsplines

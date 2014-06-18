/*
 * EuclideanBSplineTests.cpp
 *
 *  Created on: May 22, 2012
 *      Author: hannes
 */

#include "DiffManifoldBSplineTests.hpp"

namespace bsplines {
//TODO implement: test and implement EuclideanSpace<float>

// Check that the the manifold bspline with the EuclideanSpace as manifold evaluates to the same as the former Euclidean space only bspline.
TEST(EuclideanBSplineTestSuite, testEuclideanDiffManifoldBSplineBehavesAsVectorSpaceBSpline)
{
	BSpline bspline = BSpline(splineOrder);
	bspline.initConstantSpline(minTime, maxTime, numberOfSegments, ones);

	Eigen::MatrixXd coefficients = Eigen::MatrixXd::Random(rows, bspline.numVvCoefficients());
	// coefficients.block(0, 0, 1, bspline.numVvCoefficients()) = Eigen::VectorXd::LinSpaced(bspline.numVvCoefficients(), 0, 1).transpose();

	bspline.setCoefficientMatrix(coefficients);

	TestSplineDD rbsplineDD(splineOrder, rows);
	TestSplineSD rbsplineSD(splineOrder, rows);
	TestSplineDS rbsplineDS(splineOrder, rows);
	TestSpline rbspline(splineOrder, rows);

	rbspline.initConstantUniformSpline(minTime, maxTime, numberOfSegments, zero);
	rbsplineDS.initConstantUniformSpline(minTime, maxTime, numberOfSegments, zero);
	rbsplineSD.initConstantUniformSpline(minTime, maxTime, numberOfSegments, zero);
	rbsplineDD.initConstantUniformSpline(minTime, maxTime, numberOfSegments, zero);

	copyKnots(rbspline, bspline);

	SM_ASSERT_EQ(std::runtime_error, rbspline.getNumControlVertices(), bspline.numVvCoefficients(), "");

	rbspline.setControlVertices(coefficients);
	rbsplineDS.setControlVertices(coefficients);
	rbsplineSD.setControlVertices(coefficients);
	rbsplineDD.setControlVertices(coefficients);

	for(unsigned int i = 0; i <= numberOfTimeSteps; i ++) {
		double t = minTime + duration * ((double) i / numberOfTimeSteps);
		TestSpline::point_t val = bspline.eval(t);
		TestSpline::point_t rval = rbspline.getEvaluatorAt<0>(t).eval();
		TestSpline::point_t rvalDS = rbsplineDS.getEvaluatorAt<0>(t).eval();
		TestSpline::point_t rvalSD = rbsplineSD.getEvaluatorAt<0>(t).eval();
		TestSpline::point_t rvalDD = rbsplineDD.getEvaluatorAt<0>(t).eval();
		TestSpline::point_t rvalG = rbspline.getEvaluatorAt<0>(t).evalGeneric();
		TestSpline::point_t rvalDG = rbsplineDS.getEvaluatorAt<0>(t).evalGeneric();
		TestSpline::point_t rvalDDG = rbsplineDD.getEvaluatorAt<0>(t).evalGeneric();
		sm::eigen::assertNear(rval, rvalDS, 1E-9, SM_SOURCE_FILE_POS, "");
		sm::eigen::assertNear(rval, rvalSD, 1E-9, SM_SOURCE_FILE_POS, "");
		sm::eigen::assertNear(rval, rvalDD, 1E-9, SM_SOURCE_FILE_POS, "");
		sm::eigen::assertNear(rval, rvalG, 1E-9, SM_SOURCE_FILE_POS, "");
		sm::eigen::assertNear(rval, rvalDG, 1E-9, SM_SOURCE_FILE_POS, "");
		sm::eigen::assertNear(rval, rvalDDG, 1E-9, SM_SOURCE_FILE_POS, "");
		sm::eigen::assertNear(rval, val, 1E-9, SM_SOURCE_FILE_POS, "");

		for(unsigned int derivativeOrder = 0; derivativeOrder <= splineOrder + 1; derivativeOrder++){
			std::stringstream atS;
			atS << "t=" << t<< ", derivativeOrder=" << derivativeOrder;
			std::string at = atS.str();

			Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(rows, rows * splineOrder);
			TestSpline::point_t dval = bspline.evalDAndJacobian(t, derivativeOrder, &jac, NULL);
			TestSpline::point_t drval = rbspline.getEvaluatorAt<splineOrder + 1>(t).evalD(derivativeOrder);
			TestSpline::point_t drvalDS = rbsplineDS.getEvaluatorAt<splineOrder + 1>(t).evalD(derivativeOrder);
			TestSpline::point_t drvalSD = rbsplineSD.getEvaluatorAt<splineOrder + 1>(t).evalD(derivativeOrder);
			TestSpline::point_t drvalDD = rbsplineDD.getEvaluatorAt<splineOrder + 1>(t).evalD(derivativeOrder);

			if(derivativeOrder == 0){
				sm::eigen::assertNear(rval, drval, 1E-9, SM_SOURCE_FILE_POS, at);
			}
			else if(derivativeOrder >= splineOrder){
				sm::eigen::assertNear(drval, zero, 1E-9, SM_SOURCE_FILE_POS, at);
			}
			sm::eigen::assertNear(drval, drvalDS, 1E-9, SM_SOURCE_FILE_POS, at);
			sm::eigen::assertNear(drval, drvalSD, 1E-9, SM_SOURCE_FILE_POS, at);
			sm::eigen::assertNear(drval, drvalDD, 1E-9, SM_SOURCE_FILE_POS, at);
			sm::eigen::assertNear(drval, dval, 1E-9, SM_SOURCE_FILE_POS, at);

			typedef TestSpline::full_jacobian_t JacobianT;
			JacobianT rjac = JacobianT::Zero(rows, rows * splineOrder);
			typedef TestSplineDS::full_jacobian_t JacobianTDS;
			JacobianTDS rjacDS = JacobianTDS::Zero(rows, rows * splineOrder);
			typedef TestSplineSD::full_jacobian_t JacobianTSD;
			JacobianTSD rjacSD = JacobianTSD::Zero(rows, rows * splineOrder);
			Eigen::MatrixXd rjacDD = Eigen::MatrixXd::Zero(rows, rows * splineOrder);
			rbspline.getEvaluatorAt<splineOrder + 1>(t).evalJacobian(derivativeOrder, rjac);
			rbsplineDS.getEvaluatorAt<splineOrder + 1>(t).evalJacobian(derivativeOrder, rjacDS);
			rbsplineSD.getEvaluatorAt<splineOrder + 1>(t).evalJacobian(derivativeOrder, rjacSD);
			rbsplineDD.getEvaluatorAt<splineOrder + 1>(t).evalJacobian(derivativeOrder, rjacDD);

			if(derivativeOrder == 0){
				sm::eigen::assertNear(rval, drval, 1E-9, SM_SOURCE_FILE_POS, at);
			}
			sm::eigen::assertNear(rjac, rjacDS, 1E-9, SM_SOURCE_FILE_POS, at);
			sm::eigen::assertNear(rjac, rjacSD, 1E-9, SM_SOURCE_FILE_POS, at);
			sm::eigen::assertNear(rjac, rjacDD, 1E-9, SM_SOURCE_FILE_POS, at);
			sm::eigen::assertNear(rjac, jac, 1E-9, SM_SOURCE_FILE_POS, at);
		}

		for(unsigned int j = 0; j <= numberOfTimeSteps; j ++) {
			double t2 = minTime + duration * ((double) j / numberOfTimeSteps);
			Eigen::VectorXd val = bspline.evalIntegral(t, t2);
			Eigen::VectorXd rval = rbspline.evalIntegral(t, t2);
			Eigen::VectorXd rvalN = rbspline.evalIntegralNumerically(t, t2);
			Eigen::VectorXd rvalDS = rbsplineDS.evalIntegral(t, t2);
			Eigen::VectorXd rvalSD = rbsplineSD.evalIntegral(t, t2);
			Eigen::VectorXd rvalDD = rbsplineDD.evalIntegral(t, t2);
			sm::eigen::assertNear(rval, rvalDS, 1E-9, SM_SOURCE_FILE_POS, "");
			sm::eigen::assertNear(rval, rvalSD, 1E-9, SM_SOURCE_FILE_POS, "");
			sm::eigen::assertNear(rval, rvalDD, 1E-9, SM_SOURCE_FILE_POS, "");
			sm::eigen::assertNear(rval, rvalN, 1E-3, SM_SOURCE_FILE_POS, "");
			sm::eigen::assertNear(rval, val, 1E-9, SM_SOURCE_FILE_POS, "");
		}
	}

#ifdef SPEEDMEASURE
	const int repetitions = 100, steps = 1000;

	for(int j = 0 ; j < repetitions ; j ++){
		{
			double w = 0;
			sm::timing::Timer timer("getLocalBi");
			Eigen::VectorXd ret;
			for(double t = minTime; t <= maxTime; t += duration / steps){
				ret = bspline.getLocalBiVector(t);
				w += ret[0];
			}
			timer.stop();
		}
		{
			double w2 = 0;
			sm::timing::Timer timer("getLocalBiRef");
			Eigen::VectorXd ret = Eigen::VectorXd::Zero(splineOrder);
			for(double t = minTime; t <= maxTime; t += duration / steps){
				bspline.getLocalBiInto(t, ret);
				w2 += ret[0];
			}
			timer.stop();
		}
		{
			double w3 = 0;
			sm::timing::Timer timer("getLocalBi Evaluator");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				w3 += rbspline.getEvaluatorAt<0>(t).getLocalBi()[0];
			}
			timer.stop();
		}
		{
			double w4 = 0;
			sm::timing::Timer timer("getLocalBi Evaluator dynamic static");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				w4 += rbsplineDS.getEvaluatorAt<0>(t).getLocalBi()[0];
			}
			timer.stop();
		}
		{
			double w4 = 0;
			sm::timing::Timer timer("getLocalBi Evaluator static dynamic");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				w4 += rbsplineSD.getEvaluatorAt<0>(t).getLocalBi()[0];
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("eval bspline");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				bspline.eval(t);
			timer.stop();
		}
		{
			sm::timing::Timer timer("eval rbspline");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbspline.getEvaluatorAt<0>(t).eval();
			timer.stop();
		}
		{
			sm::timing::Timer timer("eval rbspline dynamic static");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbsplineDS.getEvaluatorAt<0>(t).eval();
			timer.stop();
		}
		{
			sm::timing::Timer timer("eval rbspline static dynamic");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbsplineSD.getEvaluatorAt<0>(t).eval();
			timer.stop();
		}
		{
			sm::timing::Timer timer("eval rbspline dynamic dynamic");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbsplineDD.getEvaluatorAt<0>(t).eval();
			timer.stop();
		}
		{
			sm::timing::Timer timer("eval rbspline generic");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbspline.getEvaluatorAt<0>(t).evalGeneric();
			timer.stop();
		}
		{
			sm::timing::Timer timer("eval rbspline dynamic generic");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbsplineDS.getEvaluatorAt<0>(t).evalGeneric();
			timer.stop();
		}
		{
			sm::timing::Timer timer("eval rbspline dynamic dynamic generic");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbsplineDD.getEvaluatorAt<0>(t).evalGeneric();
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalTwice bspline");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				bspline.eval(t);
				bspline.eval(t);
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalTwice rbspline");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				TestSpline::Evaluator<0> evaluator = rbspline.getEvaluatorAt<0>(t);
				evaluator.eval();
				evaluator.eval();
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalTwice rbspline dynamic static");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				TestSplineDS::Evaluator<0> evaluator = rbsplineDS.getEvaluatorAt<0>(t);
				evaluator.eval();
				evaluator.eval();
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalTwice rbspline static dynamic");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				TestSplineSD::Evaluator<0> evaluator = rbsplineSD.getEvaluatorAt<0>(t);
				evaluator.eval();
				evaluator.eval();
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalTwice rbspline dynamic dynamic");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				TestSplineDD::Evaluator<0> evaluator = rbsplineDD.getEvaluatorAt<0>(t);
				evaluator.eval();
				evaluator.eval();
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalTwice rbspline generic");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				TestSpline::Evaluator<0> evaluator = rbspline.getEvaluatorAt<0>(t);
				evaluator.evalGeneric();
				evaluator.evalGeneric();
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalTwice rbspline dynamic static generic");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				TestSplineDS::Evaluator<0> evaluator = rbsplineDS.getEvaluatorAt<0>(t);
				evaluator.evalGeneric();
				evaluator.evalGeneric();
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalTwice rbspline static dynamic generic");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				TestSplineSD::Evaluator<0> evaluator = rbsplineSD.getEvaluatorAt<0>(t);
				evaluator.evalGeneric();
				evaluator.evalGeneric();
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalTwice rbspline dynamic dynamic generic");
			for(double t = minTime; t <= maxTime; t += duration / steps){
				TestSplineDD::Evaluator<0> evaluator = rbsplineDD.getEvaluatorAt<0>(t);
				evaluator.evalGeneric();
				evaluator.evalGeneric();
			}
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalI bspline");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				bspline.evalIntegral(0, t);
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalI rbspline");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbspline.evalIntegral(0, t);
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalI rbspline dynamic static");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbsplineDS.evalIntegral(0, t);
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalI rbspline static dynamic");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbsplineSD.evalIntegral(0, t);
			timer.stop();
		}
		{
			sm::timing::Timer timer("evalI rbspline dynamic dynamic");
			for(double t = minTime; t <= maxTime; t += duration / steps)
				rbsplineDD.evalIntegral(0, t);
			timer.stop();
		}
	}
#endif
}

TEST(EuclideanBSplineTestSuite, testEuclideanDiffManifoldBSplineInitialization)
{
	const int numberOfInterpolationPoints = numberOfSegments * 2 + 3;

	BSpline bspline = BSpline(splineOrder);

	Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(numberOfInterpolationPoints, minTime, maxTime);
	Eigen::MatrixXd interpolationPoints = Eigen::MatrixXd::Random(rows, numberOfInterpolationPoints);
	//Eigen::MatrixXd interpolationPoints = Eigen::MatrixXd::Zero(rows, numberOfInterpolationPoints);
	//interpolationPoints.block(0, 0, 1, numberOfInterpolationPoints) = Eigen::VectorXd::LinSpaced(numberOfInterpolationPoints, 0, 1).transpose();
	const double lambda = 0.1; // 0; //.1;

	// TODO fix for splineOrder = 2: 	bspline.initSpline3(times, interpolationPoints, numberOfSegments, lambda);
	// TODO fix for splineOrder = 2: 	bspline.initSplineSparse(times, interpolationPoints, numberOfSegments, lambda);
	// TODO fix for splineOrder = 3: 	bspline.initSpline2(times, interpolationPoints, numberOfSegments, lambda);
	switch(splineOrder){
		case 2:
			bspline.initSpline2(times, interpolationPoints, numberOfSegments, lambda);
			break;
		default:
			bspline.initSpline3(times, interpolationPoints, numberOfSegments, lambda);
			break;
	}

	std::vector<TestSpline::time_t> timesV;
	std::vector<TestSpline::point_t> interpolationPointsV;
	std::vector<TestSplineDD::point_t> interpolationPointsVD;
	for(int i = 0; i < numberOfInterpolationPoints; i ++){
		timesV.push_back((double)times[i]);
		interpolationPointsV.push_back(interpolationPoints.col(i));
		interpolationPointsVD.push_back(interpolationPoints.col(i));
	}

	TestSpline rbspline;
	TestSplineDS rbsplineDS(splineOrder, rows);
	TestSplineSD rbsplineSD(splineOrder, rows);
	TestSplineDD rbsplineDD(splineOrder, rows);

	BSplineFitter<TestSpline>::initUniformSpline(rbspline, timesV, interpolationPointsV, numberOfSegments, lambda, FittingBackend::DENSE);
	BSplineFitter<TestSplineDS>::initUniformSpline(rbsplineDS, timesV, interpolationPointsV, numberOfSegments, lambda, FittingBackend::DENSE);
	BSplineFitter<TestSplineSD>::initUniformSpline(rbsplineSD, timesV, interpolationPointsVD, numberOfSegments, lambda, FittingBackend::DENSE);
	BSplineFitter<TestSplineDD>::initUniformSpline(rbsplineDD, timesV, interpolationPointsVD, numberOfSegments, lambda, FittingBackend::DENSE);

	SM_ASSERT_EQ(std::runtime_error, rbspline.getAbsoluteNumberOfSegments(), numberOfSegments + splineOrder * 2 - 1, "");
	SM_ASSERT_EQ(std::runtime_error, maxTime, rbspline.getMaxTime(), "");
	SM_ASSERT_EQ(std::runtime_error, minTime, rbspline.getMinTime(), "");

	for(unsigned int i = 0; i <= numberOfTimeSteps; i ++) {
		double t = minTime + duration * ((double) i / numberOfTimeSteps);
		Eigen::VectorXd val = bspline.eval(t);
		Eigen::VectorXd rval = rbspline.getEvaluatorAt<0>(t).eval();
		Eigen::VectorXd rvalDS = rbsplineDS.getEvaluatorAt<0>(t).eval();
		Eigen::VectorXd rvalSD = rbsplineSD.getEvaluatorAt<0>(t).eval();
		Eigen::VectorXd rvalDD = rbsplineDD.getEvaluatorAt<0>(t).eval();
		sm::eigen::assertNear(rvalSD, rvalDS, 1E-9, SM_SOURCE_FILE_POS, "");
		sm::eigen::assertNear(rvalSD, rvalDD, 1E-9, SM_SOURCE_FILE_POS, "");
		sm::eigen::assertNear(rval, rvalDD, 1E-9, SM_SOURCE_FILE_POS, "");
		sm::eigen::assertNear(rval, val, 1E-9, SM_SOURCE_FILE_POS, "");
	}

#ifdef SPEEDMEASURE
	const int repetitions = 100;

	for(int j = 0 ; j < repetitions ; j ++){
		{
			sm::timing::Timer timer("initSpline bspline");

			BSpline bspline = BSpline(splineOrder);

			bspline.initSpline3(times, interpolationPoints, numberOfSegments, lambda);
			timer.stop();
		}
		{
			sm::timing::Timer timer("initSpline rbspline");

			TestSpline rbspline;

			BSplineFitter<TestSpline>::initUniformSpline(rbspline, timesV, interpolationPointsV, numberOfSegments, lambda);
			timer.stop();
		}
		{
			sm::timing::Timer timer("initSpline rbsplineDS");

			TestSplineDS rbsplineDS(splineOrder, rows);

			BSplineFitter<TestSplineDS>::initUniformSpline(rbsplineDS, timesV, interpolationPointsV, numberOfSegments, lambda);
			timer.stop();
		}
		{
			sm::timing::Timer timer("initSpline rbsplineSD");

			TestSplineSD rbsplineSD(splineOrder, rows);

			BSplineFitter<TestSplineSD>::initUniformSpline(rbsplineSD, timesV, interpolationPointsVD, numberOfSegments, lambda);
			timer.stop();
		}
		{
			sm::timing::Timer timer("initSpline rbsplineDD");

			TestSplineDD rbsplineDD(splineOrder, rows);

			BSplineFitter<TestSplineDD>::initUniformSpline(rbsplineDD, timesV, interpolationPointsVD, numberOfSegments, lambda);
			timer.stop();
		}
	}
#endif
}


TEST(EuclideanBSplineTestSuite, testDiffManifoldBSplineFitting)
{
	testDiffManifoldBSplineFitting<TestSpline>(numberOfSegments);
}

TEST(EuclideanBSplineTestSuite, testDiffManifoldBSplineFittingNsecTime)
{
	testDiffManifoldBSplineFitting<TestSplineNsecTime>(numberOfSegments);
}


TEST(EuclideanBSplineTestSuite, testDiffManifoldBSplineFittingDenseSparseAgree)
{
	const int numberOfSegments = 1;
	const int numberOfInterpolationPoints = numberOfSegments * 2 + 3;

	Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(numberOfInterpolationPoints, minTime, maxTime);
	const double lambda = 0.1; // 0; //.1;

	std::vector<TestSpline::time_t> timesV;
	std::vector<TestSpline::point_t> interpolationPointsV;
	for(int i = 0; i < numberOfInterpolationPoints; i ++){
		timesV.push_back((double)times[i]);
		interpolationPointsV.push_back(TestSpline::point_t::Random());
	}

	TestSpline rbsplineSparse, rbsplineDense;

	BSplineFitter<TestSpline>::initUniformSpline(rbsplineDense, timesV, interpolationPointsV, numberOfSegments, lambda, FittingBackend::DENSE);
	BSplineFitter<TestSpline>::initUniformSpline(rbsplineSparse, timesV, interpolationPointsV, numberOfSegments, lambda, FittingBackend::SPARSE);

	SM_ASSERT_EQ(std::runtime_error, rbsplineDense.getAbsoluteNumberOfSegments(), numberOfSegments + splineOrder * 2 - 1, "");
	SM_ASSERT_EQ(std::runtime_error, maxTime, rbsplineDense.getMaxTime(), "");
	SM_ASSERT_EQ(std::runtime_error, minTime, rbsplineDense.getMinTime(), "");

	for(unsigned int i = 0; i <= numberOfTimeSteps; i ++) {
		double t = minTime + duration * ((double) i / numberOfTimeSteps);
		Eigen::VectorXd rvalDense = rbsplineDense.getEvaluatorAt<0>(t).eval();
		Eigen::VectorXd rvalSparse = rbsplineSparse.getEvaluatorAt<0>(t).eval();
		sm::eigen::assertNear(rvalDense, rvalSparse, 1E-9, SM_SOURCE_FILE_POS, "");
	}
}


template <int IDerivativeOrder>
struct BSplineJacobianFunctor
{
	// Necessary for eigen fixed sized type member variables.
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef Eigen::VectorXd input_t;
	typedef Eigen::MatrixXd jacobian_t;
	typedef Eigen::VectorXd value_t;
	typedef double scalar_t;

	BSplineJacobianFunctor(TestSplineDD & bs, double t, int dim) :
		bs_(bs), t_(t), _dim(dim), _evaluator(bs, t)
	{
	}

	input_t update(const input_t & x, int c, double delta)
	{
		input_t xnew = x;
		xnew[c] += delta;
		return xnew;
	}

	Eigen::VectorXd operator()(const Eigen::VectorXd & c)
	{
		bs_.setLocalCoefficientVector(t_, c, _dim);
		return _evaluator.evalD(IDerivativeOrder);
	}

	TestSplineDD & bs_;
	double t_;
	int _dim;
	TestSplineDD::Evaluator<IDerivativeOrder> _evaluator;
};



template<int IDerivativeOrder>
void testBSplineJacobianD(TestSplineDD & bs, Eigen::MatrixXd & J, Eigen::VectorXd & coefficients, int dim){
	testBSplineJacobianD<IDerivativeOrder - 1>(bs, J, coefficients, dim);

	for(double t = bs.getMinTime(), end = bs.getMaxTime(); t <= end; t += 0.1)
	{
		BSplineJacobianFunctor<IDerivativeOrder> f(bs, t, dim);
		sm::eigen::NumericalDiff<BSplineJacobianFunctor<IDerivativeOrder> > nd(f);
		bs.getLocalCoefficientVector(t, coefficients, dim);
		Eigen::MatrixXd estJ = nd.estimateJacobian(coefficients);
		J.setZero(dim, dim * splineOrder);
		bs.getEvaluatorAt<IDerivativeOrder>(t).evalJacobian(IDerivativeOrder, J);
		sm::eigen::assertNear(J, estJ, 1e-5, SM_SOURCE_FILE_POS);
	}
}
template<> void testBSplineJacobianD<-1>(TestSplineDD & /* bs */, Eigen::MatrixXd & /* J */, Eigen::VectorXd & /* coefficients */, int /* dim */){};

template<int ISplineOrder>
void testBSplineJacobiansSO(){
	testBSplineJacobiansSO<ISplineOrder - 1>();

	const int segments = 2;
	const int nk = knot_arithmetics::getNumKnotsRequired(segments, ISplineOrder);
	for(int dim = 1; dim < 4; dim++)
	{
		TestSplineDD bs(ISplineOrder, dim);
		for(int i = 0; i < nk; i++) bs.addKnot(i);
		bs.init();

		int nc = bs.getNumControlVertices();
		Eigen::MatrixXd C = Eigen::MatrixXd::Random(dim,nc);
		bs.setControlVertices(C);

		Eigen::MatrixXd J(dim, dim * ISplineOrder);
		Eigen::VectorXd coefficients(dim * ISplineOrder);

		testBSplineJacobianD<ISplineOrder + 1>(bs, J, coefficients, dim);
	}
}
template<> void testBSplineJacobiansSO<1>(){}


// Check that the Jacobian calculation is correct.
TEST(EuclideanBSplineTestSuite, testBSplineJacobian)
{
	testBSplineJacobiansSO<10>();
}

TEST(EuclideanBSplineTestSuite, testDExp)
{
	DExpTester<EuclideanSpaceConf< rows >::Manifold >::testFunc(10, 10);
}

TEST(EuclideanBSplineTestSuite, evalD)
{
	SplineEvalDTester<TestSpline, 1>::testFunc(10, 10);
	SplineEvalDTester<TestSpline, 2>::testFunc(10, 10);
	SplineEvalDTester<TestSpline, 3>::testFunc(10, 10);
}

// Check that the Jacobian calculation is correct.
TEST(EuclideanBSplineTestSuite, testBSplineJacobianGenerically)
{
	BSplineJacobianTester<TestSpline, 0>::testFunc(10, 1);
	BSplineJacobianTester<TestSpline, 1>::testFunc(10, 1);
	BSplineJacobianTester<TestSpline, 2>::testFunc(10, 1);
	BSplineJacobianTester<TestSpline, 3>::testFunc(10, 1);
	BSplineJacobianTester<TestSpline, 4>::testFunc(10, 1);
}

} //namespace bsplines

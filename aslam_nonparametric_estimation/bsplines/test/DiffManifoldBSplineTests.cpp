#include "bsplines/manifolds/EuclideanSpace.hpp"
#include "bsplines/manifolds/UnitQuaternionManifold.hpp"
#include "gtest/gtest.h"
#include <sm/eigen/gtest.hpp>

#define TEST_SPLINES

#include "NodeDistributedCacheTests.cpp"
#include "NumericIntegratorTests.cpp"


namespace bsplines {

	template <int IDim, bool dynamic, typename TScalar >
	void testManifolds() {
		using namespace manifolds;
		typedef Eigen::Matrix<TScalar, IDim, 1> Vec;

		Vec rand = Vec::Random();

		typename EuclideanSpaceConf<dynamic? Eigen::Dynamic : IDim, TScalar>::Manifold m(IDim);
		sm::eigen::assertNear(m.getIdentity(), Vec::Zero(), 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(m.exp(Vec::Ones(), Vec::Ones()), 2 * Vec::Ones(), 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(m.exp(Vec::Zero(), rand), m.expAtId(rand), 1E-9, SM_SOURCE_FILE_POS);

		UnitQuaternionManifoldConf<>::Manifold u;
		Eigen::Vector4d uqId = Eigen::Vector4d::Zero();
		uqId[3] = 1;
		sm::eigen::assertNear(u.getIdentity(), uqId, 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(u.exp(uqId, Eigen::Vector3d::Zero()), uqId, 1E-9, SM_SOURCE_FILE_POS);
		Eigen::Vector3d randDouble = Eigen::Vector3d::Random();
		sm::eigen::assertNear(u.exp(uqId, randDouble), u.expAtId(randDouble), 1E-9, SM_SOURCE_FILE_POS);
	}

	TEST(DiffManifoldBSplineTestSuite, testManifolds)
	{
		testManifolds<3, true, double>();
		testManifolds<3, false, double>();
		testManifolds<3, true, float>();
		testManifolds<3, false, float>();
	}
}

#ifdef TEST_SPLINES

#include "DiffManifoldBSplineTests.hpp"

namespace bsplines {

TEST(DiffManifoldBSplineTestSuite, testInitialization)
{
	TestSpline rbspline;
	TestSplineDS rbsplineDS(splineOrder);

	Eigen::VectorXd x = Eigen::VectorXd::Random(rows);
	rbspline.initConstantUniformSpline(minTime, maxTime, numberOfSegments, x);

	SM_ASSERT_EQ(std::runtime_error, rbspline.getSplineOrder(), splineOrder, "");
	SM_ASSERT_EQ(std::runtime_error, rbsplineDS.getSplineOrder(), splineOrder, "");

	SM_ASSERT_EQ(std::runtime_error, rbspline.getAbsoluteNumberOfSegments() , numberOfSegments + splineOrder * 2 - 1, "");
	SM_ASSERT_EQ(std::runtime_error, maxTime, rbspline.getMaxTime(), "");
	SM_ASSERT_EQ(std::runtime_error, minTime, rbspline.getMinTime(), "");

	SM_ASSERT_EQ(std::runtime_error, minTime, rbspline.getTimeInterval().first, "");
	SM_ASSERT_EQ(std::runtime_error, maxTime, rbspline.getTimeInterval().second, "");

	SM_ASSERT_EQ(std::runtime_error, rbspline.firstRelevantSegment().getKnot(), rbspline.getEvaluatorAt<0>(minTime)._firstRelevantControlVertexIt.getKnot(),"");

	SM_ASSERT_EQ(std::runtime_error, minTime, rbspline.getEvaluatorAt<0>(minTime).getKnot(),"");
	SM_ASSERT_EQ(std::runtime_error, minTime, rbspline.getEvaluatorAt<0>(minTime + 0.01).getKnot(), "");
	SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(maxTime - 0.01).getKnot(), rbspline.getEvaluatorAt<0>(maxTime).getKnot(), ""); //TODO improve: remove this irregularity
	SM_ASSERT_GT(std::runtime_error, maxTime, rbspline.getEvaluatorAt<0>(maxTime - 0.01).getKnot(), "");

	TestSpline::SegmentConstIterator i = rbspline.getAbsoluteBegin(), end = rbspline.getAbsoluteEnd();

	unsigned c = 0;
	double lastKnot = minTime;

	BSpline bspline = BSpline(splineOrder);
	bspline.initConstantSpline(minTime, maxTime, numberOfSegments, x);
	std::vector<double> knots = bspline.knots();

	for(; i != end; i++){
		sm::eigen::assertEqual(i->getControlVertex(), x, SM_SOURCE_FILE_POS);

		double tKnot = i.getKnot();

		SM_ASSERT_NEAR(std::runtime_error, knots[c], tKnot, 1e-15, " c = " << c);

		if(!(c < splineOrder - 1|| c >= numberOfSegments + splineOrder - 1)){
			if(tKnot != maxTime){
				SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(tKnot).getKnot(), tKnot, "");
				SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(tKnot + 1e-13).getKnot(), rbspline.getEvaluatorAt<0>(tKnot).getKnot(), "");
			}
			else{
				SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(tKnot).getKnot(), lastKnot, "");
			}
			if(tKnot != minTime)
				SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(tKnot - 1e-13).getKnot(), lastKnot, "");

			lastKnot = rbspline.getEvaluatorAt<0>(tKnot).getKnot();
		}
		c++;
	}
}

TEST(DiffManifoldBSplineTestSuite, testCopyAndAssign)
{
	TestSpline rbspline;
	Eigen::VectorXd x = Eigen::VectorXd::Random(rows);
	rbspline.initConstantUniformSpline(minTime, maxTime, numberOfSegments, x);

	const int nCopies = 2;
	TestSpline rbsplineCopies[nCopies] = {rbspline};

	rbsplineCopies[1] = rbspline;

	double t = minTime + maxTime / 2;

	for(int i = 0; i < nCopies; i++){
		SM_ASSERT_EQ(std::runtime_error, rbsplineCopies[i].getAbsoluteNumberOfSegments() , numberOfSegments + splineOrder * 2 - 1, "");
		SM_ASSERT_EQ(std::runtime_error, maxTime, rbsplineCopies[i].getMaxTime(), "");
		SM_ASSERT_EQ(std::runtime_error, minTime, rbsplineCopies[i].getMinTime(), "");

		SM_ASSERT_EQ(std::runtime_error, minTime, rbsplineCopies[i].getTimeInterval().first, "");
		SM_ASSERT_EQ(std::runtime_error, maxTime, rbsplineCopies[i].getTimeInterval().second, "");

		sm::eigen::assertEqual(rbspline.getEvaluatorAt<0>(t).eval(), rbsplineCopies[i].getEvaluatorAt<0>(t).eval(), SM_SOURCE_FILE_POS, "");
	}
}

TEST(DiffManifoldBSplineTestSuite, testAddingSegments)
{
	TestSpline rbspline;
	typedef TestSpline::point_t point_t;

	unsigned int numberOfSegments = 1;
	double minTime = 0, maxTime = 1, delta = maxTime - minTime;

	struct PointSource {
		int i;
		bool random;

		TestSpline::point_t getNext(){
			return random ? point_t(point_t::Random(rows)) : point_t(point_t::Ones(rows) * (double)(i++));
		}

		PointSource(bool random) : i(0), random(random){}
	} pointSource(false);

	rbspline.initConstantUniformSpline(minTime, maxTime, numberOfSegments, ones * -1);
	vector<TestSpline::point_t> vertices;
	for(int i = rbspline.getNumControlVertices(); i != 0; i --){
		vertices.push_back(pointSource.getNext());
	}
	setControlVerticesFromContainer(rbspline, vertices);

	for(unsigned int i = 0; i < 10 ; i ++){
		SM_ASSERT_EQ(std::runtime_error, rbspline.getAbsoluteNumberOfSegments(), numberOfSegments + splineOrder * 2 - 1, "");
		SM_ASSERT_EQ(std::runtime_error, maxTime, rbspline.getMaxTime(), "");

		SM_ASSERT_EQ(std::runtime_error, rbspline.firstRelevantSegment().getKnot(), rbspline.getEvaluatorAt<0>(minTime)._firstRelevantControlVertexIt.getKnot(),"");

		SM_ASSERT_EQ(std::runtime_error, minTime, rbspline.getEvaluatorAt<0>(minTime).getKnot(),"");
		SM_ASSERT_EQ(std::runtime_error, minTime, rbspline.getEvaluatorAt<0>(minTime + 0.01).getKnot(), "");
		SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(maxTime - 0.01).getKnot(), rbspline.getEvaluatorAt<0>(maxTime).getKnot(), ""); //TODO improve: remove this irregularity
		SM_ASSERT_GT(std::runtime_error, maxTime, rbspline.getEvaluatorAt<0>(maxTime - 0.01).getKnot(), "");

		TestSpline rbsplineToCompare;
		rbsplineToCompare.initConstantUniformSpline(minTime, maxTime, numberOfSegments, ones * -1);
		setControlVerticesFromContainer(rbsplineToCompare, vertices);


		double lastKnot = minTime;
		unsigned int c = 0;
		for(TestSpline::SegmentConstIterator it = rbspline.getAbsoluteBegin(), end = rbspline.getAbsoluteEnd(), itToCompare = rbsplineToCompare.getAbsoluteBegin(); it != end; it++){
			if(c < vertices.size()){
				SM_ASSERT_EQ(std::runtime_error, it->getControlVertex(), itToCompare->getControlVertex(), "control vertices should match at segment data " << c);
				if(c > splineOrder - 1)
					SM_ASSERT_EQ(std::runtime_error, it->getBasisMatrix(), itToCompare->getBasisMatrix(), "basis matrices should match at segment data " << c);
			}

			const double tKnot = it.getKnot();

			SM_ASSERT_EQ(std::runtime_error, itToCompare.getKnot(), tKnot, " c = " << c);

			if(!(c < splineOrder - 1|| c >= numberOfSegments + splineOrder - 1)){
				if(tKnot != maxTime){
					SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(tKnot).getKnot(), tKnot, "");
					SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(tKnot + 1e-13).getKnot(), rbspline.getEvaluatorAt<0>(tKnot).getKnot(), "");
				}
				else{
					SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(tKnot).getKnot(), lastKnot, "");
				}
				if(tKnot != minTime)
					SM_ASSERT_EQ(std::runtime_error, rbspline.getEvaluatorAt<0>(tKnot - 1e-13).getKnot(), lastKnot, "");

				lastKnot = rbspline.getEvaluatorAt<0>(tKnot).getKnot();
			}
			itToCompare++;
			c++;
		}
		SM_ASSERT_EQ(std::runtime_error, c, numberOfSegments + 2 * splineOrder - 1, "too few segments found");

		for(unsigned int i = 0; i <= numberOfTimeSteps; i ++) {
			double t = minTime + (maxTime - minTime) * ((double) i / numberOfTimeSteps);
			TestSpline::point_t rval = rbspline.getEvaluatorAt<0>(t).eval();
			TestSpline::point_t rvalToCompare = rbsplineToCompare.getEvaluatorAt<0>(t).eval();
			sm::eigen::assertNear(rval, rvalToCompare, 1E-9, SM_SOURCE_FILE_POS, "");
		}

		unsigned int segmentsInc = i > 5 ? 1 : 2;
		maxTime += delta * segmentsInc;
		TestSpline::point_t x = pointSource.getNext();
		for(int j = segmentsInc; j != 0 ; j--) vertices.push_back(x);
		rbspline.appendSegmentsUniformly(segmentsInc, &x);
		numberOfSegments += segmentsInc;
	}
}


TEST(DiffManifoldBSplineTestSuite, testGetBi)
{
	const int numTimeSteps = 100;

	BSpline bspline = BSpline(splineOrder);
	TestSpline rbspline(splineOrder);
	TestSplineDS rbsplineDS(splineOrder);

	rbspline.initConstantUniformSpline(minTime, maxTime, numberOfSegments, zero);
	rbsplineDS.initConstantUniformSpline(minTime, maxTime, numberOfSegments, zero);

	bspline.initConstantSpline(minTime, maxTime, numberOfSegments, zero);
	copyKnots(rbspline, bspline);

	for(int i = 0; i <= numTimeSteps; i ++) {
		double t = minTime + duration * ((double) i / numTimeSteps);
		TestSpline::Evaluator<0> evaluator = rbspline.getEvaluatorAt<0>(t);
		Eigen::VectorXd localBiVector = evaluator.getLocalBi();
		SM_ASSERT_EQ(std::runtime_error, localBiVector.rows(), splineOrder, "");
		SM_ASSERT_NEAR(std::runtime_error, localBiVector.sum(), 1.0, 1e-10, "the bi at a given time should always sum up to 1")

		Eigen::VectorXd localCumulativeBiVector = evaluator.getLocalCumulativeBi();
		SM_ASSERT_EQ(std::runtime_error, localCumulativeBiVector.rows(), splineOrder, "");

		TestSpline::SegmentConstIterator firstIndex = evaluator.getFirstRelevantSegmentIterator();
		TestSpline::SegmentConstIterator lastIndex = evaluator.getLastRelevantSegmentIterator();
		SM_ASSERT_EQ(std::runtime_error, getIteratorDistance(firstIndex, lastIndex, (TestSpline::SegmentConstIterator) rbspline.end()), splineOrder - 1, "the distance between first and last local control point has to be exactly the spline order - 1. (at t = " << t << ")");

		SM_ASSERT_NEAR(std::runtime_error, (double)localCumulativeBiVector[0], 1.0, 1e-10, "");

		sm::eigen::assertNear(bspline.getLocalBiVector(t), rbspline.getEvaluatorAt<0>(t).getLocalBi(), 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(bspline.getLocalBiVector(t), rbsplineDS.getEvaluatorAt<0>(t).getLocalBi(), 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(bspline.getLocalCumulativeBiVector(t), rbspline.getEvaluatorAt<0>(t).getLocalCumulativeBi(), 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(bspline.getLocalCumulativeBiVector(t), rbsplineDS.getEvaluatorAt<0>(t).getLocalCumulativeBi(), 1E-9, SM_SOURCE_FILE_POS);
		sm::eigen::assertNear(bspline.eval(t), rbspline.getEvaluatorAt<0>(t).eval(), 1e-9, SM_SOURCE_FILE_POS);

		for(int j = 0, n = splineOrder; j < n; j++){
			if(j > 0){
				SM_ASSERT_LE(std::runtime_error, localCumulativeBiVector[j], localCumulativeBiVector[j - 1] + 1E-11, " at t = " << t << ", j = " << j);
			}
			SM_ASSERT_NEAR(std::runtime_error, (double)localCumulativeBiVector[j], (double)localBiVector.segment(j, splineOrder - j).sum(), 1e-13, "cumulativeBiVector must be the sum of the localBiVector where it overlaps, but it is not at " << j << " (localBiVector=" << localBiVector << ")");
		}
	}
}

TEST(DiffManifoldBSplineTestSuite, testLongTimeBSplineCompilation)
{
	TestSplineLongTime rbspline;
	TestSplineLongTime::point_t p = zero;
	p[0] = 1.0;
	rbspline.initConstantUniformSpline(minTimeLong, maxTimeLong, numberOfSegments, p);
	sm::eigen::assertEqual(rbspline.getEvaluatorAt<0>(minTimeLong).eval(), p, SM_SOURCE_FILE_POS);
}

} //namespace bsplines

#include "UnitQuaternionBSplineTests.cpp"
#include "EuclideanBSplineTests.cpp"

#ifdef SPEEDMEASURE
TEST(ZLASTDiffManifoldBSplineTestSuite, printTimings)
{
	sm::timing::Timing::print(std::cout);
}
#endif

#endif

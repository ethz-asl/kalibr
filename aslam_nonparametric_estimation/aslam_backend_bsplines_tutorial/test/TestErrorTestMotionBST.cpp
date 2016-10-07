#include <gtest/gtest.h>

#include <aslam/backend/ErrorTermMotionBST.hpp>
// This test harness makes it easy to test error terms.
#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <aslam/backend/TransformationBasic.hpp>

#include <aslam/backend/EuclideanPoint.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/implementation/OPTBSplineImpl.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <aslam/backend/Scalar.hpp>

#include <boost/shared_ptr.hpp>

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


TEST(AslamVChargeBackendTestSuite, testEuclidean)
{
  try {
    using namespace aslam::backend;

    double sigma_n = 0.5;

    typedef aslam::splines::OPTBSpline<bsplines::EuclideanBSpline<4, 1>::CONF> PosSpline;
    PosSpline robotPosSpline;
    const int pointSize = robotPosSpline.getPointSize();

    PosSpline::point_t initPoint(pointSize);
    initPoint(0,0) = 10.0;

    robotPosSpline.initConstantUniformSpline(0, 10, 10, initPoint);

    // First, create a design variable for the wall position.
    boost::shared_ptr<aslam::backend::Scalar> dv_w(new aslam::backend::Scalar(5.0));

    // Create observation error
    auto vecVelExpr = robotPosSpline.getExpressionFactoryAt<1>(5).getValueExpression(1);

    aslam::backend::ErrorTermMotionBST eo(vecVelExpr, 1.0, sigma_n * sigma_n);

    EXPECT_NEAR(1.0/(sigma_n * sigma_n), eo.evaluateError(), 1e-14);

    // Create the test harness
    aslam::backend::ErrorTermTestHarness<1> harness(&eo);

    // Run the unit tests.
    harness.testAll(1e-5);
  }
  catch(const std::exception & e) {
    FAIL() << e.what();
  }
}

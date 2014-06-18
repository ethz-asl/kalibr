// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>

#include <aslam/backend/test/ErrorTermTestHarness.hpp>
#include <aslam/cameras.hpp>
#include <aslam/CameraGeometryDesignVariableContainer.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
TEST( CvBackendTestSuite, testReprojectionError ) {

  using namespace aslam;
  using namespace aslam::backend;
  using namespace aslam::cameras;
  typedef DistortedOmniRsCameraGeometry camera_geometry_t;
  boost::shared_ptr<camera_geometry_t> geometry(new camera_geometry_t( camera_geometry_t::getTestGeometry() ) );

  boost::shared_ptr< CameraGeometryDesignVariableContainer > geometryDvc( new CameraGeometryDesignVariableContainer( geometry, true, true, true ));

  Eigen::Matrix2d invR;
  invR.setIdentity();

  Eigen::Vector4d p = sm::kinematics::toHomogeneous(geometry->createRandomVisiblePoint());
  boost::shared_ptr<aslam::backend::HomogeneousPoint> pt( new aslam::backend::HomogeneousPoint(p) );
  pt->setActive(true);
  pt->setBlockIndex(0);
  HomogeneousExpression hep(pt);

  Eigen::VectorXd y;
  geometry->homogeneousToKeypoint(p,y);

  boost::shared_ptr< aslam::ReprojectionError > re = geometryDvc->createReprojectionError( y, invR, hep);

  ErrorTermTestHarness<2> harness( (aslam::backend::ErrorTerm*)re.get());
  harness.testAll();

}

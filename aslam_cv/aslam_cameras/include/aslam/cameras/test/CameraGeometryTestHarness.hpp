// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <boost/lexical_cast.hpp>
#include <sm/string_routines.hpp>
#include <sm/boost/serialization.hpp>
#include <sm/serialization_macros.hpp>

namespace aslam {
namespace cameras {

template<typename CAMERA_GEOMETRY_T>
class CameraGeometryTestHarness {
 public:
  typedef CAMERA_GEOMETRY_T camera_geometry_t;
  typedef typename camera_geometry_t::keypoint_t keypoint_t;
  typedef typename camera_geometry_t::jacobian_homogeneous_t jacobian_homogeneous_t;
  typedef typename camera_geometry_t::jacobian_t jacobian_t;
  typedef typename camera_geometry_t::inverse_jacobian_t inverse_jacobian_t;
  typedef typename camera_geometry_t::inverse_jacobian_homogeneous_t inverse_jacobian_homogeneous_t;
  typedef typename camera_geometry_t::projection_t::jacobian_intrinsics_t jacobian_intrinsics_t;
  typedef Eigen::Vector3d point_t;
  typedef Eigen::Vector4d pointh_t;
  typedef typename camera_geometry_t::projection_t projection_t;

  camera_geometry_t _geometry;
  double _tolerance;

  CameraGeometryTestHarness()
      : _tolerance(1e-2) {
    _geometry = camera_geometry_t::getTestGeometry();
  }

  CameraGeometryTestHarness(const camera_geometry_t & geometry,
                            double tolerance)
      : _geometry(geometry),
        _tolerance(tolerance) {
  }

  CameraGeometryTestHarness(const camera_geometry_t & geometry)
      : _geometry(geometry),
        _tolerance(1e-2) {
  }

  CameraGeometryTestHarness(double tolerance)
      : _tolerance(tolerance) {
    _geometry = camera_geometry_t::getTestGeometry();
  }

  virtual ~CameraGeometryTestHarness() {
  }

  void testKeypointToEuclidean() {
    SCOPED_TRACE(__FUNCTION__);

    for (int i = 0; i < 20; i++) {

      keypoint_t y = _geometry.createRandomKeypoint();
      // Test that we can go from a keypoint to Euclidean and back to within tolerance.
      inverse_jacobian_t J;
      point_t p;
      _geometry.keypointToEuclidean(y, p, J);

      sm::eigen::assertFinite(p, SM_SOURCE_FILE_POS);
      sm::eigen::assertFinite(J, SM_SOURCE_FILE_POS);

      keypoint_t yhat;
      _geometry.euclideanToKeypoint(p, yhat);

      sm::eigen::assertFinite(yhat, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat, _tolerance, "");

      pointh_t ph = sm::kinematics::toHomogeneous(p);
      keypoint_t yhat2;
      _geometry.homogeneousToKeypoint(ph, yhat2);
      sm::eigen::assertFinite(yhat2, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat2, _tolerance, "");

      ph = -ph;
      _geometry.homogeneousToKeypoint(ph, yhat2);
      sm::eigen::assertFinite(yhat2, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat2, _tolerance, "");

      inverse_jacobian_t Jest;
      _geometry.keypointToEuclideanFiniteDifference(y, p, Jest);
      ASSERT_DOUBLE_MX_EQ(J, Jest, _tolerance, "");
    }

  }

  void testKeypointToHomogeneous() {
    SCOPED_TRACE(__FUNCTION__);
    for (int i = 0; i < 20; i++) {
      point_t p = _geometry.createRandomVisiblePoint();
      pointh_t ph = sm::kinematics::toHomogeneous(p);
      keypoint_t y;
      _geometry.euclideanToKeypoint(p, y);
      // Test that we can go from a keypoint to Euclidean and back to within tolerance.
      inverse_jacobian_homogeneous_t J;
      pointh_t ph2;
      _geometry.keypointToHomogeneous(y, ph2, J);

      sm::eigen::assertFinite(ph, SM_SOURCE_FILE_POS);
      sm::eigen::assertFinite(J, SM_SOURCE_FILE_POS);

      keypoint_t yhat;
      _geometry.homogeneousToKeypoint(ph, yhat);
      sm::eigen::assertFinite(yhat, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat, _tolerance, "");

      // point_t p = sm::kinematics::fromHomogeneous(ph);
      // keypoint_t yhat2;
      // _geometry.euclideanToKeypoint(p,yhat2);
      // sm::eigen::assertFinite(yhat2,SM_SOURCE_FILE_POS);
      // ASSERT_DOUBLE_MX_EQ(y,yhat2,_tolerance,"");

      inverse_jacobian_homogeneous_t Jest;
      //std::cout << ph.transpose() << "\n";
      _geometry.keypointToHomogeneousFiniteDifference(y, ph2, Jest);
      ASSERT_DOUBLE_MX_EQ(J, Jest, _tolerance, "");
      ph = -ph;
      _geometry.homogeneousToKeypoint(ph, yhat);
      sm::eigen::assertFinite(yhat, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat, _tolerance, "");

    }
  }

  void testEuclideanToKeypoint() {
    SCOPED_TRACE(__FUNCTION__);
    for (int i = 0; i < 20; i++) {
      keypoint_t y = _geometry.createRandomKeypoint();

      jacobian_t J;
      point_t p;
      _geometry.keypointToEuclidean(y, p);
      sm::eigen::assertFinite(p, SM_SOURCE_FILE_POS);

      keypoint_t yhat;
      _geometry.euclideanToKeypoint(p, yhat, J);
      sm::eigen::assertFinite(yhat, SM_SOURCE_FILE_POS);
      sm::eigen::assertFinite(J, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat, _tolerance, "");

      pointh_t ph = sm::kinematics::toHomogeneous(p);
      keypoint_t yhat2;
      _geometry.homogeneousToKeypoint(ph, yhat2);
      sm::eigen::assertFinite(yhat2, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat2, _tolerance, "");

      ph = -ph;
      _geometry.homogeneousToKeypoint(ph, yhat2);
      sm::eigen::assertFinite(yhat2, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat2, _tolerance, "");

      if (_geometry.isProjectionInvertible()) {
        // If the sensor model is invertible, we should get the same point in space.
        pointh_t ph2;
        _geometry.keypointToHomogeneous(y, ph2);
        point_t p2 = sm::kinematics::fromHomogeneous(ph2);
        ASSERT_DOUBLE_MX_EQ(p, p2, _tolerance, "");
      }

      jacobian_t Jest;
      _geometry.euclideanToKeypointFiniteDifference(p, yhat, Jest);
      ASSERT_DOUBLE_MX_EQ(J, Jest, _tolerance, "");

    }
  }

  void testHomogeneousToKeypoint() {
    SCOPED_TRACE(__FUNCTION__);
    for (int i = 0; i < 20; i++) {
      keypoint_t y = _geometry.createRandomKeypoint();

      jacobian_homogeneous_t J;
      pointh_t ph;
      _geometry.keypointToHomogeneous(y, ph);

      // having the fourth component be zero can be a singularity
      // for the Jacobian of some camera models. 
      // Setting this to 1.0 seems to work and is still testing what
      // we want.
      if (ph[3] == 0.0)
        ph[3] = 1.0;
      sm::eigen::assertFinite(ph, SM_SOURCE_FILE_POS);

      keypoint_t yhat;
      _geometry.homogeneousToKeypoint(ph, yhat, J);
      sm::eigen::assertFinite(yhat, SM_SOURCE_FILE_POS);
      sm::eigen::assertFinite(J, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat, _tolerance, "");
      jacobian_homogeneous_t Jest;
      _geometry.homogeneousToKeypointFiniteDifference(ph, y, Jest);
      ASSERT_DOUBLE_MX_EQ(J, Jest, _tolerance, "");

      if (_geometry.isProjectionInvertible()) {
        point_t pp = sm::kinematics::fromHomogeneous(ph);
        keypoint_t yhat2;
        _geometry.euclideanToKeypoint(pp, yhat2);
        sm::eigen::assertFinite(yhat2, SM_SOURCE_FILE_POS);
        ASSERT_DOUBLE_MX_EQ(y, yhat2, _tolerance, "");

        // If the sensor model is invertible, we should get the same point in space.p
        point_t p2;
        _geometry.keypointToEuclidean(y, p2);
        ASSERT_DOUBLE_MX_EQ(pp, p2, _tolerance, "");
      }
      // This should still work
      ph = -ph;

      _geometry.homogeneousToKeypoint(ph, yhat, J);
      sm::eigen::assertFinite(yhat, SM_SOURCE_FILE_POS);
      sm::eigen::assertFinite(J, SM_SOURCE_FILE_POS);
      ASSERT_DOUBLE_MX_EQ(y, yhat, _tolerance, "");
      _geometry.homogeneousToKeypointFiniteDifference(ph, y, Jest);
      ASSERT_DOUBLE_MX_EQ(J, Jest, _tolerance, "");

    }
  }

  void testIntrinsicsJacobian() {
    SCOPED_TRACE(__FUNCTION__);
    if (_geometry.projection().minimalDimensions() > 0) {

      for (int i = 0; i < 20; i++) {
        point_t p = _geometry.createRandomVisiblePoint(2);

        jacobian_intrinsics_t J;
        _geometry.euclideanToKeypointIntrinsicsJacobian(p, J);

        jacobian_intrinsics_t Jest;
        _geometry.euclideanToKeypointIntrinsicsJacobianFiniteDifference(p,
                                                                        Jest);
        ASSERT_DOUBLE_MX_EQ(J, Jest, _tolerance, "");

        Eigen::Vector4d ph = sm::kinematics::toHomogeneous(p);
        _geometry.homogeneousToKeypointIntrinsicsJacobian(ph, J);
        _geometry.homogeneousToKeypointIntrinsicsJacobianFiniteDifference(ph,
                                                                          Jest);
        ASSERT_DOUBLE_MX_EQ(J, Jest, _tolerance, "");

        ph = -ph;
        _geometry.homogeneousToKeypointIntrinsicsJacobian(ph, J);
        _geometry.homogeneousToKeypointIntrinsicsJacobianFiniteDifference(ph,
                                                                          Jest);
        ASSERT_DOUBLE_MX_EQ(J, Jest, _tolerance, "");

      }
    }
  }

  void testDistortionJacobian() {
    SCOPED_TRACE(__FUNCTION__);
    if (_geometry.projection().distortion().minimalDimensions() > 0) {

      // An initial test along the optical axis.
      point_t p(0.0, 0.0, 10.0);
      if (_geometry.isEuclideanVisible(p)) {
        SCOPED_TRACE(" P: " + sm::s_tuple(p(0), p(1), p(2)));
        //std::cout << "Optical axis\n";
        Eigen::Matrix<double, 2, projection_t::distortion_t::IntrinsicsDimension> J;
        _geometry.euclideanToKeypointDistortionJacobian(p, J);
        Eigen::Matrix<double, 2, projection_t::distortion_t::IntrinsicsDimension> Jest;

        _geometry.euclideanToKeypointDistortionJacobianFiniteDifference(p,
                                                                        Jest);
        sm::eigen::assertNear(J, Jest, 1e-5, SM_SOURCE_FILE_POS,
                              "Distortion Jacobian");
        // This relative tolerance test was too strict for small values.
        // ASSERT_DOUBLE_MX_EQ(J,Jest,_tolerance,"");
      }

      p << 0.0, 0.0, -10.0;
      if (_geometry.isEuclideanVisible(p)) {
        SCOPED_TRACE(" P: " + sm::s_tuple(p(0), p(1), p(2)));
        //std::cout << "Negative optical axis\n";
        Eigen::Matrix<double, 2, projection_t::distortion_t::IntrinsicsDimension> J;
        _geometry.euclideanToKeypointDistortionJacobian(p, J);
        Eigen::Matrix<double, 2, projection_t::distortion_t::IntrinsicsDimension> Jest;

        _geometry.euclideanToKeypointDistortionJacobianFiniteDifference(p,
                                                                        Jest);
        sm::eigen::assertNear(J, Jest, 1e-5, SM_SOURCE_FILE_POS,
                              "Distortion Jacobian");
        // This relative tolerance test was too strict for small values.
        //ASSERT_DOUBLE_MX_EQ(J,Jest,_tolerance,"");
      }

      // For some reason this case is difficult for the omni camera
      p << 1e-1, 1e-1, 10.0;
      if (_geometry.isEuclideanVisible(p)) {
        SCOPED_TRACE(" P: " + sm::s_tuple(p(0), p(1), p(2)));
        //std::cout << "Negative optical axis\n";
        Eigen::Matrix<double, 2, projection_t::distortion_t::IntrinsicsDimension> J;
        _geometry.euclideanToKeypointDistortionJacobian(p, J);
        Eigen::Matrix<double, 2, projection_t::distortion_t::IntrinsicsDimension> Jest;

        _geometry.euclideanToKeypointDistortionJacobianFiniteDifference(p,
                                                                        Jest);
        // This relative tolerance test was too strict for small values.
        //ASSERT_DOUBLE_MX_EQ(J,Jest,_tolerance,"");
        sm::eigen::assertNear(J, Jest, 1e-5, SM_SOURCE_FILE_POS,
                              "Distortion Jacobian");
      }

      for (int i = 0; i < 200; i++) {
        point_t p = _geometry.createRandomVisiblePoint(10);
        SCOPED_TRACE(
            "Trial " + boost::lexical_cast < std::string
                > (i) + " P: " + sm::s_tuple(p(0), p(1), p(2)));
        // SM_OUT(p);
        // keypoint_t k;
        // bool valid = _geometry.euclideanToKeypoint(p,k);
        // SM_OUT(k);
        // SM_OUT(valid);

        Eigen::Matrix<double, 2, projection_t::distortion_t::IntrinsicsDimension> J;
        _geometry.euclideanToKeypointDistortionJacobian(p, J);
        Eigen::Matrix<double, 2, projection_t::distortion_t::IntrinsicsDimension> Jest;

        _geometry.euclideanToKeypointDistortionJacobianFiniteDifference(p,
                                                                        Jest);

        // This relative tolerance test was too strict for small values.
        //ASSERT_DOUBLE_MX_EQ(J,Jest,_tolerance,"");
        sm::eigen::assertNear(J, Jest, 1e-5, SM_SOURCE_FILE_POS,
                              "Distortion Jacobian");
      }
    }
  }

  void testSerialization() {
    SCOPED_TRACE(__FUNCTION__);
    ASSERT_TRUE(_geometry.isBinaryEqual(_geometry));

    sm::boost_serialization::save(_geometry, "test.ba");

    camera_geometry_t G2;

    ASSERT_FALSE(_geometry.isBinaryEqual(G2));

    sm::boost_serialization::load(G2, "test.ba");

    ASSERT_TRUE(_geometry.isBinaryEqual(G2));
  }

  void testAll() {
    {
      SCOPED_TRACE("EuclideanToKeypoint");
      testEuclideanToKeypoint();
    }
    {
      SCOPED_TRACE("HomogeneousToKeypoint");
      testHomogeneousToKeypoint();
    }

    {
      SCOPED_TRACE("KeypointToEuclidean");
      testKeypointToEuclidean();
    }
    {
      SCOPED_TRACE("KeypointToHomogeneous");
      testKeypointToHomogeneous();
    }
    {
      SCOPED_TRACE("IntrinsicsJacobian");
      testIntrinsicsJacobian();
    }
    {
      SCOPED_TRACE("DistortionJacobian");
      testDistortionJacobian();
    }
    {
      SCOPED_TRACE("Serialization");
      testSerialization();
    }
  }

};

}  // namespace cameras
}  // namespace asl


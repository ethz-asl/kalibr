#include <aslam/cameras/OmniCameraGeometry.hpp>
#include <aslam/Exceptions.hpp>
#include <Eigen/Dense>

namespace aslam {
namespace cameras {

OmniCameraGeometry::OmniCameraGeometry() {
  *this = createTestGeometry();
}

OmniCameraGeometry::OmniCameraGeometry(double xi, double k1, double k2,
                                       double p1, double p2, double gamma1,
                                       double gamma2, double u0, double v0,
                                       int width, int height)
    : _xi(xi),
      _k1(k1),
      _k2(k2),
      _p1(p1),
      _p2(p2),
      _gamma1(gamma1),
      _gamma2(gamma2),
      _u0(u0),
      _v0(v0),
      _width(width),
      _height(height) {
  updateTemporaries();
}

OmniCameraGeometry::~OmniCameraGeometry() {

}

// This updates the intrinsic parameters with a small step: i <-- i + di
// The Jacobians above are with respect to this update function.
void OmniCameraGeometry::updateIntrinsicsOplus(double * di) {
  _xi += di[0];
  _k1 += di[1];
  _k2 += di[2];
  _p1 += di[3];
  _p2 += di[4];
  _gamma1 += di[5];
  _gamma2 += di[6];
  _u0 += di[7];
  _v0 += di[8];
  updateTemporaries();
}

// The amount of time elapsed between the start of the image and the
// keypoint. For a global shutter camera, this can return Duration(0).
Duration OmniCameraGeometry::temporalOffset(const keypoint_t & keypoint) const {
  return Duration(0);
}

OmniCameraGeometry::keypoint_t OmniCameraGeometry::maxKeypoint() const {
  return keypoint_t(_width, _height);
}

OmniCameraGeometry::keypoint_t OmniCameraGeometry::minKeypoint() const {
  return keypoint_t(0, 0);
}

std::string OmniCameraGeometry::typeName() const {
  return "OmniCamera";
}

OmniCameraGeometry OmniCameraGeometry::createTestGeometry() {
  return OmniCameraGeometry(1.2080, -2.103030975849235e-01,
                            1.511327079893021e-02, 6.035288960688402e-04,
                            -5.277839371727245e-04, 5.671146836756682e+02,
                            5.665554204210736e+02, 6.196542975873804e+02,
                            5.106649931478771e+02, 1224, 1024);
}

/** 
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param u u image coordinate
 * \param v v image coordinate
 * \param X X coordinate of the point on the sphere
 * \param Y Y coordinate of the point on the sphere
 * \param Z Z coordinate of the point on the sphere
 */
void OmniCameraGeometry::lift_sphere(double u, double v, double *X, double *Y,
                                     double *Z) const {
  double mx_d, my_d, mx_u, my_u;
  double lambda;

  // Lift points to normalised plane
  // Matlab points start at 1 (calibration)
  mx_d = _inv_K11 * (u) + _inv_K13;
  my_d = _inv_K22 * (v) + _inv_K23;

  // Recursive distortion model
  // int n = 6;
  // double dx_u, dy_u;
  // distortion(mx_d,my_d,&dx_u,&dy_u);
  // // Approximate value
  // mx_u = mx_d-dx_u;
  // my_u = my_d-dy_u;

  // for(int i=1;i<n;i++) {
  // 	distortion(mx_u,my_u,&dx_u,&dy_u);
  // 	mx_u = mx_d-dx_u;
  // 	my_u = my_d-dy_u;
  // }
  // PTF: March 31, 2012. The above was not that accurate.
  //      Substitute Gauss-Newton.
  undistortGN(mx_d, my_d, &mx_u, &my_u);

  // Lift normalised points to the sphere (inv_hslash)
  lambda = (_xi + sqrt(1 + (1 - _xi * _xi) * (mx_u * mx_u + my_u * my_u)))
      / (1 + mx_u * mx_u + my_u * my_u);
  *X = lambda * mx_u;
  *Y = lambda * my_u;
  *Z = lambda - _xi;
}

/** 
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param u u image coordinate
 * \param v v image coordinate
 * \param X X coordinate of the projective ray
 * \param Y Y coordinate of the projective ray
 * \param Z Z coordinate of the projective ray
 */
void OmniCameraGeometry::lift_projective(double u, double v, double *X,
                                         double *Y, double *Z) const {
  double mx_d, my_d, mx_u, my_u;
  double rho2_d;

  // Lift points to normalised plane
  // Matlab points start at 1 (calibration)
  mx_d = _inv_K11 * (u) + _inv_K13;
  my_d = _inv_K22 * (v) + _inv_K23;

  // Recursive distortion model
  // int n = 8;
  // double dx_u, dy_u;
  // distortion(mx_d,my_d,&dx_u,&dy_u);
  // // Approximate value
  // mx_u = mx_d-dx_u;
  // my_u = my_d-dy_u;

  // for(int i=1;i<n;i++) {
  // 	distortion(mx_u,my_u,&dx_u,&dy_u);
  // 	mx_u = mx_d-dx_u;
  // 	my_u = my_d-dy_u;
  // }
  // PTF: March 31, 2012. The above was not that accurate.
  //      Substitute Gauss-Newton.
  undistortGN(mx_d, my_d, &mx_u, &my_u);

  //std::cout << "lift projective: u: " << mx_u << ", v: " << my_u << std::endl;

  // Obtain a projective ray
  // Reuse variable
  rho2_d = mx_u * mx_u + my_u * my_u;
  *X = mx_u;
  *Y = my_u;
  *Z = 1 - _xi * (rho2_d + 1) / (_xi + sqrt(1 + (1 - _xi * _xi) * rho2_d));

  //std::cout << "lift projective: p: " << *X << ", " << *Y << ", " << *Z << std::endl;
}

/** 
 * \brief Project a 3D points (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param x 3D point x coordinate
 * \param y 3D point y coordinate
 * \param z 3D point z coordinate
 * \param u return value, contains the image point u coordinate
 * \param v return value, contains the image point v coordinate
 */
void OmniCameraGeometry::space2plane(double x, double y, double z, double *u,
                                     double *v) const {
  double mx_u, my_u, mx_d, my_d;

  // Project points to the normalised plane
  z = z + _xi * sqrt(x * x + y * y + z * z);
  mx_u = x / z;
  my_u = y / z;

  // Apply distortion
  double dx_u, dy_u;
  distortion(mx_u, my_u, &dx_u, &dy_u);
  mx_d = mx_u + dx_u;
  my_d = my_u + dy_u;

  // Apply generalised projection matrix
  // Matlab points start at 1
  *u = _gamma1 * mx_d + _u0;
  *v = _gamma2 * my_d + _v0;
}

/** 
 * \brief Project a 3D points (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *        and calculate jacobian
 *
 * \param x 3D point x coordinate
 * \param y 3D point y coordinate
 * \param z 3D point z coordinate
 * \param u return value, contains the image point u coordinate
 * \param v return value, contains the image point v coordinate
 */
void OmniCameraGeometry::space2plane(double x, double y, double z, double *u,
                                     double *v, double *dudx, double *dvdx,
                                     double *dudy, double *dvdy, double *dudz,
                                     double *dvdz) const {
  double mx_u, my_u, mx_d, my_d;
  double norm, inv_denom;
  double dxdmx, dydmx, dxdmy, dydmy;

  norm = sqrt(x * x + y * y + z * z);
  // Project points to the normalised plane
  inv_denom = 1 / (z + _xi * norm);
  mx_u = inv_denom * x;
  my_u = inv_denom * y;

  // Calculate jacobian
  inv_denom = inv_denom * inv_denom / norm;
  *dudx = inv_denom * (norm * z + _xi * (y * y + z * z));
  *dvdx = -inv_denom * _xi * x * y;
  *dudy = *dvdx;
  *dvdy = inv_denom * (norm * z + _xi * (x * x + z * z));
  inv_denom = inv_denom * (-_xi * z - norm);  // reuse variable
  *dudz = x * inv_denom;
  *dvdz = y * inv_denom;

  // Apply distortion
  double dx_u, dy_u;
  distortion(mx_u, my_u, &dx_u, &dy_u, &dxdmx, &dydmx, &dxdmy, &dydmy);
  mx_d = mx_u + dx_u;
  my_d = my_u + dy_u;

  // Make the product of the jacobians
  // and add projection matrix jacobian
  inv_denom = _gamma1 * (*dudx * dxdmx + *dvdx * dxdmy);  // reuse
  *dvdx = _gamma2 * (*dudx * dydmx + *dvdx * dydmy);
  *dudx = inv_denom;

  inv_denom = _gamma1 * (*dudy * dxdmx + *dvdy * dxdmy);  // reuse
  *dvdy = _gamma2 * (*dudy * dydmx + *dvdy * dydmy);
  *dudy = inv_denom;

  inv_denom = _gamma1 * (*dudz * dxdmx + *dvdz * dxdmy);  // reuse
  *dvdz = _gamma2 * (*dudz * dydmx + *dvdz * dydmy);
  *dudz = inv_denom;

  // Apply generalised projection matrix
  // Matlab points start at 1
  *u = _gamma1 * mx_d + _u0;
  *v = _gamma2 * my_d + _v0;
}

/** 
 * \brief Projects an undistorted 2D point (\a mx_u,\a my_u) to the image plane in (\a u,\a v)
 *
 * \param mx_u 2D point x coordinate
 * \param my_u 3D point y coordinate
 * \param u return value, contains the image point u coordinate
 * \param v return value, contains the image point v coordinate
 */
void OmniCameraGeometry::undist2plane(double mx_u, double my_u, double *u,
                                      double *v) const {
  double mx_d, my_d;

  // Apply distortion
  double dx_u, dy_u;
  distortion(mx_u, my_u, &dx_u, &dy_u);
  mx_d = mx_u + dx_u;
  my_d = my_u + dy_u;

  // Apply generalised projection matrix
  // Matlab points start at 1
  *u = _gamma1 * mx_d + _u0;
  *v = _gamma2 * my_d + _v0;
}

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *  
 * \param mx_u undistorted x coordinate of point on the normalised plane
 * \param my_u undistorted y coordinate of point on the normalised plane
 * \param dx return value, to obtain the distorted point : mx_d = mx_u+dx_u
 * \param dy return value, to obtain the distorted point : my_d = my_u+dy_u
 */
void OmniCameraGeometry::distortion(double mx_u, double my_u, double *dx_u,
                                    double *dy_u) const {
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = mx_u * mx_u;
  my2_u = my_u * my_u;
  mxy_u = mx_u * my_u;
  rho2_u = mx2_u + my2_u;
  rad_dist_u = _k1 * rho2_u + _k2 * rho2_u * rho2_u;
  *dx_u = mx_u * rad_dist_u + 2 * _p1 * mxy_u + _p2 * (rho2_u + 2 * mx2_u);
  *dy_u = my_u * rad_dist_u + 2 * _p2 * mxy_u + _p1 * (rho2_u + 2 * my2_u);
}

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate jacobian
 *
 * \param mx_u undistorted x coordinate of point on the normalised plane
 * \param my_u undistorted y coordinate of point on the normalised plane
 * \param dx return value, to obtain the distorted point : mx_d = mx_u+dx_u
 * \param dy return value, to obtain the distorted point : my_d = my_u+dy_u
 */
void OmniCameraGeometry::distortion(double mx_u, double my_u, double *dx_u,
                                    double *dy_u, double *dxdmx, double *dydmx,
                                    double *dxdmy, double *dydmy) const {
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = mx_u * mx_u;
  my2_u = my_u * my_u;
  mxy_u = mx_u * my_u;
  rho2_u = mx2_u + my2_u;
  rad_dist_u = _k1 * rho2_u + _k2 * rho2_u * rho2_u;
  *dx_u = mx_u * rad_dist_u + 2 * _p1 * mxy_u + _p2 * (rho2_u + 2 * mx2_u);
  *dy_u = my_u * rad_dist_u + 2 * _p2 * mxy_u + _p1 * (rho2_u + 2 * my2_u);

  *dxdmx = 1 + rad_dist_u + _k1 * 2 * mx2_u + _k2 * rho2_u * 4 * mx2_u
      + 2 * _p1 * my_u + 6 * _p2 * mx_u;
  *dydmx = _k1 * 2 * mx_u * my_u + _k2 * 4 * rho2_u * mx_u * my_u
      + _p1 * 2 * mx_u + 2 * _p2 * my_u;
  *dxdmy = *dydmx;
  *dydmy = 1 + rad_dist_u + _k1 * 2 * my2_u + _k2 * rho2_u * 4 * my2_u
      + 6 * _p1 * my_u + 2 * _p2 * mx_u;
}

void OmniCameraGeometry::updateTemporaries() {
  // Inverse camera projection matrix parameters
  _inv_K11 = 1.0 / _gamma1;
  _inv_K13 = -_u0 / _gamma1;
  _inv_K22 = 1.0 / _gamma2;
  _inv_K23 = -_v0 / _gamma2;
  _one_over_xixi_m_1 = 1.0 / (_xi * _xi - 1.0);

}

void OmniCameraGeometry::setIntrinsicsVectorImplementation(
    const Eigen::VectorXd & V) {
  _xi = V[0];
  _k1 = V[1];
  _k2 = V[2];
  _p1 = V[3];
  _p2 = V[4];
  _gamma1 = V[5];
  _gamma2 = V[6];
  _u0 = V[7];
  _v0 = V[8];
  updateTemporaries();

}

Eigen::VectorXd OmniCameraGeometry::getIntrinsicsVectorImplementation() const {
  Eigen::VectorXd V(9);
  V[0] = _xi;
  V[1] = _k1;
  V[2] = _k2;
  V[3] = _p1;
  V[4] = _p2;
  V[5] = _gamma1;
  V[6] = _gamma2;
  V[7] = _u0;
  V[8] = _v0;
  return V;
}

Eigen::Matrix3d OmniCameraGeometry::getCameraMatrix() const {
  Eigen::Matrix3d K;
  K << _gamma1, 0, _u0, 0, _gamma2, _v0, 0, 0, 1;
  return K;
}

Eigen::VectorXd OmniCameraGeometry::createRandomKeypoint() const {
  // This is tricky...The camera model defines a circle on the normalized image
  // plane and the projection equations don't work outside of it.
  // With some manipulation, we can see that, on the normalized image plane,
  // the edge of this circle is at u^2 + v^2 = 1/(xi^2 - 1)
  // So: this function creates keypoints inside this boundary.

  // Create a point on the normalized image plane inside the boundary.
  // This is not efficient, but it should be correct.

  Eigen::Vector2d u(width() + 1, height() + 1);

  while (u[0] < 0 || u[0] > width() - 1 || u[1] < 0 || u[1] > height() - 1) {
    u.setRandom();
    u = u - Eigen::Vector2d(0.5, 0.5);
    u /= u.norm();
    u *= ((double) rand() / (double) RAND_MAX) * _one_over_xixi_m_1;

    // Now we run the point through distortion and projection.
    // Apply distortion
    double dx_u, dy_u;
    distortion(u[0], u[1], &dx_u, &dy_u);
    double mx_d = u[0] + dx_u;
    double my_d = u[1] + dy_u;

    // Apply generalised projection matrix
    // Matlab points start at 1
    u[0] = _gamma1 * mx_d + _u0;
    u[1] = _gamma2 * my_d + _v0;
  }

  // I would like to do this below but it is way, way worse than above.
  // // The output
  // Eigen::Vector2d u;
  // // The output projected to the normalized image plane.
  // Eigen::Vector2d u_norm;

  // // The singularity radius minus an epsilon
  // double nRadius = 1.0/_one_over_xixi_m_1;

  // do
  // 	{
  // 	  // Create a random keypoint in the image.
  // 	  //u.setRandom();
  // 	  u[0] = ( (double)rand() / (double)RAND_MAX)*_width;
  // 	  u[1] = ( (double)rand() / (double)RAND_MAX)*_height;

  // 	  // Lift points to normalised plane
  // 	  // Matlab points start at 1 (calibration)
  // 	  double u0_d = _inv_K11*(u[0])+_inv_K13;
  // 	  double u1_d = _inv_K22*(u[1])+_inv_K23;
  // 	  undistortGN(u0_d, u1_d, &u_norm[0], &u_norm[1]);

  // 	}
  // while( u_norm.dot(u_norm) >= nRadius );

  return u;
}

// Use Gauss-Newton to undistort.
void OmniCameraGeometry::undistortGN(double u_d, double v_d, double * u,
                                     double * v) const {
  *u = u_d;
  *v = v_d;

  double ubar = u_d;
  double vbar = v_d;
  const int n = 5;
  Eigen::Matrix2d F;

  double hat_u_d;
  double hat_v_d;

  // void OmniCameraGeometry::distortion(double mx_u, double my_u, 
  // 					  double *dx_u, double *dy_u,
  // 					  double *dxdmx, double *dydmx,
  // 					  double *dxdmy, double *dydmy) const
  for (int i = 0; i < n; i++) {
    distortion(ubar, vbar, &hat_u_d, &hat_v_d, &F(0, 0), &F(1, 0), &F(0, 1),
               &F(1, 1));

    Eigen::Vector2d e(u_d - ubar - hat_u_d, v_d - vbar - hat_v_d);
    Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;

    ubar += du[0];
    vbar += du[1];

    if (e.dot(e) < 1e-15)
      break;

  }
  *u = ubar;
  *v = vbar;
}

/// \brief initialize the intrinsics based on one view of a gridded calibration target
/// \return true on success
///
/// These functions were developed with the help of Lionel Heng and the excellent camodocal
/// https://github.com/hengli/camodocal
bool OmniProjection::initializeIntrinsics(
    const GridCalibrationTargetObservation & obs) {
  if (!obs.target()) {
    return false;
  }

  double square(double x) {return x*x;}
  float square(float x) {return x*x;}
  double hypot(double a, double b) {return sqrt( square(a) + square(b) );}

  // First, initialize the image center at the center of the image.
  _xi = 1.0;
  _cu = obs.imCols() / 2.0;
  _cv = obs.imRows() / 2.0;
  _ru = obs.imCols();
  _rv = obs.imRows();

  _distortion.clear();

  // Grab a reference to the target for easy access.
  const GridCalibrationTarget & target = *obs.target();

  /// Initialize some temporaries needed.
  double gamma0 = 0.0;
  double minReprojErr = std::numeric_limits<double>::max();

  // Now we try to find a non-radial line to initialize the focal length
  bool success = false;
  for (size_t r = 0; r < target.rows(); ++r) {
    // Grab all the valid corner points for this checkerboard observation
    cv::Mat
    P(target.cols(); 4, CV_64F
        );
        size_t count = 0;
        for (size_t c = 0; c < target.cols(); ++c) {
          Eigen::Vector2d imagePoint;
          Eigen::Vector3d gridPoint;
          if (obs.imageGridPoint(r, c, imagePoint)) {
            double u = imagePoint[0] - _cu;
            double v = imagePoint[1] - _cv;
            P.at<double>(count, 0) = u;
            P.at<double>(count, 1) = v;
            P.at<double>(count, 2) = 0.5;
            P.at<double>(count, 3) = -0.5 * (square(u) + square(v));
            ++count;
          }
        }

        const int MIN_CORNERS = 8;
        // MIN_CORNERS is an arbitrary threshold for the number of corners
        if (count > MIN_CORNERS) {
          // Resize P to fit with the count of valid points.
          cv::Mat C;
          cv::SVD::solveZ(P.colRange(0, count), C);

          double t = square(C.at<double>(0)) + square(C.at<double>(1))
              + C.at<double>(2) * C.at<double>(3);
          if (t < 0) {
            continue;
          }

          // check that line image is not radial
          double d = sqrt(1.0 / t);
          double nx = C.at<double>(0) * d;
          double ny = C.at<double>(1) * d;
          if (hypot(nx, ny) > 0.95) {
            continue;
          }

          double nz = sqrt(1.0 - square(nx) - square(ny));
          double gamma = fabs(C.at<double>(2) * d / nz);

          _fu = gamma;
          _fv = gamma;
          sm::kinematics::Transformation T_target_camera;
          if (!estimateTransformation(obs, T_target_camera)) {
            continue;
          }

          double reprojErr = 0.0;
          size_t numReprojected = computeReprojectionError(obs, T_target_camera,
                                                           reprojErr);

          if (numReprojected > MIN_CORNERS) {
            double avgReprojErr = reprojErr / numReprojected;

            if (avgReprojErr < minReprojErr) {
              minReprojErr = avgReprojErr;
              gamma0 = gamma;
              success = true;
            }
          }

        }  // If this observation has enough valid corners
      }  // For each row in the image.

      _fu = gamma0;
      _fv = gamma0;

      return success;

    }  // initializeIntrinsics()

    bool OmniProjection::computeReprojectionError(
        const GridCalibrationTargetObservation & obs,
        const sm::kinematics::Transformation & T_target_camera,
        double & outErr) const {
      outErr = 0.0;
      size_t count = 0;
      sm::kinematics::Transformation T_camera_target =
          T_target_camera.inverse();

      for (size_t i = 0; i < obs.size(); ++i) {
        Eigen::Vector2d y, yhat;
        if (obs.imagePoint(i, y)
            && euclideanToKeypoint(T_camera_target * obs.target()->point(i),
                                   yhat)) {
          outErr += (y - yhat).norm();
          ++count;
        }
      }

      return count;

    }

    /// \brief estimate the transformation of the camera with respect to the calibration target
    ///        On success out_T_t_c is filled in with the transformation that takes points from
    ///        the camera frame to the target frame
    /// \return true on success
    ///
    /// These functions were developed with the help of Lionel Heng and the excellent camodocal
    /// https://github.com/hengli/camodocal
    bool OmniProjection::estimateTransformation(
        const GridCalibrationTargetObservation & obs,
        sm::kinematics::Transformation & out_T_t_c) const {
      // Convert all chessboard corners to a fakey pinhole view.
      // Call the OpenCV pnp function.
    }

    }  // namespace cameras  

    }  // namespace aslam

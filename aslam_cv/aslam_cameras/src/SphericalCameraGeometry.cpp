#include <aslam/cameras/SphericalCameraGeometry.hpp>
#include <aslam/Exceptions.hpp>
#include <Eigen/Dense>

namespace aslam {
namespace cameras {

SphericalCameraGeometry::SphericalCameraGeometry() {
  *this = createTestGeometry();
}

SphericalCameraGeometry::SphericalCameraGeometry(double xi, double gamma1,
                                                 double gamma2, double u0,
                                                 double v0, int width,
                                                 int height)
    : _xi(xi),
      _gamma1(gamma1),
      _gamma2(gamma2),
      _u0(u0),
      _v0(v0),
      _width(width),
      _height(height) {
  updateTemporaries();
}

SphericalCameraGeometry::~SphericalCameraGeometry() {

}

// This updates the intrinsic parameters with a small step: i <-- i + di
// The Jacobians above are with respect to this update function.
void SphericalCameraGeometry::updateIntrinsicsOplus(double * di) {
  _xi += di[0];
  // MN: This offset (4 empty entries) is not nice, but provides a
  // common interface with OmniCameras
  _gamma1 += di[5];
  _gamma2 += di[6];
  _u0 += di[7];
  _v0 += di[8];
  updateTemporaries();
}

// The amount of time elapsed between the start of the image and the
// keypoint. For a global shutter camera, this can return Duration(0).
Duration SphericalCameraGeometry::temporalOffset(
    const keypoint_t & keypoint) const {
  return Duration(0);
}

SphericalCameraGeometry::keypoint_t SphericalCameraGeometry::maxKeypoint() const {
  return keypoint_t(_width, _height);
}

SphericalCameraGeometry::keypoint_t SphericalCameraGeometry::minKeypoint() const {
  return keypoint_t(0, 0);
}

std::string SphericalCameraGeometry::typeName() const {
  return "SphericalCamera";
}

SphericalCameraGeometry SphericalCameraGeometry::createTestGeometry() {
  return SphericalCameraGeometry(1.2080, 5.671146836756682e+02,
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
void SphericalCameraGeometry::lift_sphere(double u, double v, double *X,
                                          double *Y, double *Z) const {
  double mx_u, my_u;
  double lambda;

  // Lift points to normalised plane
  // Matlab points start at 1 (calibration)
  mx_u = _inv_K11 * (u) + _inv_K13;
  my_u = _inv_K22 * (v) + _inv_K23;

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
  //undistortGN(mx_d, my_d, &mx_u, &my_u);

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
void SphericalCameraGeometry::lift_projective(double u, double v, double *X,
                                              double *Y, double *Z) const {
  double mx_u, my_u;
  double rho2_d;

  // Lift points to normalised plane
  // Matlab points start at 1 (calibration)
  mx_u = _inv_K11 * (u) + _inv_K13;
  my_u = _inv_K22 * (v) + _inv_K23;

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
  //undistortGN(mx_d, my_d, &mx_u, &my_u);

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
void SphericalCameraGeometry::space2plane(double x, double y, double z,
                                          double *u, double *v) const {
  double mx_u, my_u;

  // Project points to the normalised plane
  z = z + _xi * sqrt(x * x + y * y + z * z);
  mx_u = x / z;
  my_u = y / z;

  /*
   // Apply distortion
   double dx_u, dy_u;
   distortion(mx_u,my_u,&dx_u,&dy_u);
   mx_d = mx_u+dx_u;
   my_d = my_u+dy_u;
   */

  // Apply generalised projection matrix
  // Matlab points start at 1
  *u = _gamma1 * mx_u + _u0;
  *v = _gamma2 * my_u + _v0;
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
void SphericalCameraGeometry::space2plane(double x, double y, double z,
                                          double *u, double *v, double *dudx,
                                          double *dvdx, double *dudy,
                                          double *dvdy, double *dudz,
                                          double *dvdz) const {

  SM_THROW(std::runtime_error, "Not implemented");
  double mx_u, my_u;
  double norm, inv_denom;
  //double dxdmx, dydmx, dxdmy, dydmy;

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
  /*
   double dx_u, dy_u;
   distortion(mx_u,my_u,&dx_u,&dy_u,&dxdmx,&dydmx,&dxdmy,&dydmy);
   mx_d = mx_u+dx_u;
   my_d = my_u+dy_u;
   */

  // Make the product of the jacobians
  // and add projection matrix jacobian
  inv_denom = _gamma1 * (*dudx + *dvdx);  // reuse
  *dvdx = _gamma2 * (*dudx + *dvdx);
  *dudx = inv_denom;

  inv_denom = _gamma1 * (*dudy + *dvdy);  // reuse
  *dvdy = _gamma2 * (*dudy + *dvdy);
  *dudy = inv_denom;

  inv_denom = _gamma1 * (*dudz + *dvdz);  // reuse
  *dvdz = _gamma2 * (*dudz + *dvdz);
  *dudz = inv_denom;

  // Apply generalised projection matrix
  // Matlab points start at 1
  *u = _gamma1 * mx_u + _u0;
  *v = _gamma2 * my_u + _v0;

}

/** 
 * \brief Projects an undistorted 2D point (\a mx_u,\a my_u) to the image plane in (\a u,\a v)
 *
 * \param mx_u 2D point x coordinate
 * \param my_u 3D point y coordinate
 * \param u return value, contains the image point u coordinate
 * \param v return value, contains the image point v coordinate
 */
void SphericalCameraGeometry::undist2plane(double mx_u, double my_u, double *u,
                                           double *v) const {
  //double mx_d, my_d;

  /*
   // Apply distortion
   double dx_u, dy_u;
   distortion(mx_u,my_u,&dx_u,&dy_u);
   mx_d = mx_u+dx_u;
   my_d = my_u+dy_u;
   */

  // Apply generalised projection matrix
  // Matlab points start at 1
  *u = _gamma1 * mx_u + _u0;
  *v = _gamma2 * my_u + _v0;
}

void SphericalCameraGeometry::updateTemporaries() {
  // Inverse camera projection matrix parameters
  _inv_K11 = 1.0 / _gamma1;
  _inv_K13 = -_u0 / _gamma1;
  _inv_K22 = 1.0 / _gamma2;
  _inv_K23 = -_v0 / _gamma2;
  _one_over_xixi_m_1 = 1.0 / (_xi * _xi - 1.0);

}

void SphericalCameraGeometry::setIntrinsicsVectorImplementation(
    const Eigen::VectorXd & V) {
  _xi = V[0];
  // MN: This offset (4 empty entries) is not nice, but provides a
  // common interface with OmniCameras
  _gamma1 = V[5];
  _gamma2 = V[6];
  _u0 = V[7];
  _v0 = V[8];
  updateTemporaries();

}

Eigen::VectorXd SphericalCameraGeometry::getIntrinsicsVectorImplementation() const {
  Eigen::VectorXd V = Eigen::VectorXd::Zero(9);
  V[0] = _xi;
  // MN: This offset (4 empty entries) is not nice, but provides a
  // common interface with OmniCameras
  V[1] = 0;
  V[2] = 0;
  V[3] = 0;
  V[4] = 0;
  V[5] = _gamma1;
  V[6] = _gamma2;
  V[7] = _u0;
  V[8] = _v0;
  return V;
}

Eigen::Matrix3d SphericalCameraGeometry::getCameraMatrix() const {
  Eigen::Matrix3d K;
  K << _gamma1, 0, _u0, 0, _gamma2, _v0, 0, 0, 1;
  return K;
}

Eigen::VectorXd SphericalCameraGeometry::createRandomKeypoint() const {
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

    /*
     // Now we run the point through distortion and projection.
     // Apply distortion
     double dx_u, dy_u;
     distortion(u[0],u[1],&dx_u,&dy_u);
     double mx_d = u[0]+dx_u;
     double my_d = u[1]+dy_u;
     */

    // Apply generalised projection matrix
    // Matlab points start at 1
    u[0] = _gamma1 * u[0] + _u0;
    u[1] = _gamma2 * u[1] + _v0;
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

}  // namespace cameras  

}  // namespace aslam

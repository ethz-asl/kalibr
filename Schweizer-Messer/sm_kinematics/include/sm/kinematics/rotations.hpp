/**
 * @file   rotations.hpp
 * @author Paul Furgale <paul.furgale@utoronto.ca>
 * @date   Mon Nov 22 21:45:54 2010
 * 
 * @brief  
 * 
 * 
 */

#ifndef SM_ROTATIONS_HPP
#define SM_ROTATIONS_HPP

#define SM_2_PI 		0.6366197723675814 // 2/pi
#define SM_PI 		3.141592653589793  // pi
#define SM_PI_2 		1.5707963267948966 // pi/2
#define SM_PI_4 		0.7853981633974483 // pi/4
#define SM_2PI 		6.283185307179586  // 2*pi
#define SM_DEG2RAD	0.017453292519943  // pi/180
#define SM_RAD2DEG	57.295779513082323 // 180/pi

#define SM_1_PI_F 	0.3183098861837907f
#define SM_2_PI_F 	0.6366197723675814f
#define SM_PI_F 		3.141592653589793f
#define SM_PI_2_F 	1.5707963267948966f
#define SM_PI_4_F		0.7853981633974483f
#define SM_2PI_F 		6.283185307179586f
#define SM_DEG2RAD_F	0.017453292519943f
#define SM_RAD2DEG_F	57.295779513082323f

#include <Eigen/Core>

namespace sm { namespace kinematics {

  // Principle axis rotations.
  Eigen::Matrix3d Rx(double radians);
  Eigen::Matrix3d Ry(double radians);
  Eigen::Matrix3d Rz(double radians);

  Eigen::Matrix3d rph2R(double x, double y, double z);
  Eigen::Matrix3d rph2R(Eigen::Vector3d const & x);
  Eigen::Vector3d R2rph(Eigen::Matrix3d const & C);

  // The C rotations are more standard. They go the other way
  Eigen::Matrix3d Cx(double radians);
  Eigen::Matrix3d Cy(double radians);
  Eigen::Matrix3d Cz(double radians);

  Eigen::Matrix3d rph2C(double x, double y, double z);
  Eigen::Matrix3d rph2C(Eigen::Vector3d const & x);
  Eigen::Vector3d C2rph(Eigen::Matrix3d const & C);
  
  // Small angle approximation.
  template <typename Scalar_ = double>
  Eigen::Matrix<Scalar_, 3, 3> crossMx(Scalar_ x, Scalar_ y, Scalar_ z);

  template <typename Derived_>
  Eigen::Matrix<typename Eigen::internal::traits< Derived_ >::Scalar, 3, 3> crossMx(Eigen::MatrixBase<Derived_> const & x){ return crossMx(x(0, 0),x(1, 0),x(2, 0)); }



  // Axis Angle rotation.
  Eigen::Matrix3d axisAngle2R(double a, double ax, double ay, double az);
  Eigen::Matrix3d axisAngle2R(double x, double y, double z);
  Eigen::Matrix3d axisAngle2R(Eigen::Vector3d const & x);
  Eigen::Vector3d R2AxisAngle(Eigen::Matrix3d const & C);

  // Utility functions
  // Moves a value in radians to within -pi, pi
  double angleMod(double radians);
  double deg2rad(double degrees);
  double rad2deg(double radians);

  extern template Eigen::Matrix<double, 3, 3> crossMx<double>(double x, double y, double z);
  extern template Eigen::Matrix<float, 3, 3> crossMx<float>(float x, float y, float z);
  extern template Eigen::Matrix<double, 3, 3> crossMx(const Eigen::MatrixBase<Eigen::Matrix<double, 3, 1> > &);
  extern template Eigen::Matrix<float, 3, 3> crossMx(const Eigen::MatrixBase<Eigen::Matrix<float, 3, 1> > &);
  extern template Eigen::Matrix<double, 3, 3> crossMx(const Eigen::MatrixBase<Eigen::Matrix<double, Eigen::Dynamic, 1> > &);
  extern template Eigen::Matrix<float, 3, 3> crossMx(const Eigen::MatrixBase<Eigen::Matrix<float, Eigen::Dynamic, 1> > &);
}} // namespace sm::kinematics

#endif // ndef SM_ROTATIONS_HPP

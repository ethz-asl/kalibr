#include <sm/kinematics/rotations.hpp>
#include <sm/assert_macros.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/round.hpp>

namespace sm { namespace kinematics {

  // Euler angle rotations.
  Eigen::Matrix3d Rx(double radians){
    Eigen::Matrix3d C;
    double c = cos(radians);
    double s = sin(radians);
    C(0,0) = 1;   C(0,1) = 0.0; C(0,2) =  0;
    C(1,0) = 0.0; C(1,1) = c;   C(1,2) = -s;
    C(2,0) = 0.0; C(2,1) = s;   C(2,2) =  c;
		
    return C;
  }

  Eigen::Matrix3d Ry(double radians){
    Eigen::Matrix3d C;
    double c = cos(radians);
    double s = sin(radians);
    C(0,0) =  c;   C(0,1) = 0.0; C(0,2) =  s;
    C(1,0) =  0.0; C(1,1) = 1;   C(1,2) = 0.0;
    C(2,0) = -s;   C(2,1) = 0.0; C(2,2) =  c;
		
    return C;
  }
  Eigen::Matrix3d Rz(double radians){
    Eigen::Matrix3d C;
    double c = cos(radians);
    double s = sin(radians);
    C(0,0) = c;   C(0,1) = -s;  C(0,2) = 0.0;
    C(1,0) = s;   C(1,1) =  c;  C(1,2) = 0.0;
    C(2,0) = 0.0; C(2,1) = 0.0; C(2,2) =  1;
		
    return C;
  }

  Eigen::Matrix3d rph2R(double x, double y, double z){
    Eigen::Matrix3d C;
    double cx = cos(x);
    double sx = sin(x);
    double cy = cos(y);
    double sy = sin(y);
    double cz = cos(z);
    double sz = sin(z);
    //[cos(z)*cos(y), -sin(z)*cos(x)+cos(z)*sin(y)*sin(x),  sin(z)*sin(x)+cos(z)*sin(y)*cos(x)]
    //[sin(z)*cos(y),  cos(z)*cos(x)+sin(z)*sin(y)*sin(x), -cos(z)*sin(x)+sin(z)*sin(y)*cos(x)]
    //[      -sin(y),                       cos(y)*sin(x),                       cos(y)*cos(x)]
    C(0,0) = cz*cy; C(0,1) = -sz*cx+cz*sy*sx; C(0,2) =   sz*sx+cz*sy*cx;
    C(1,0) = sz*cy; C(1,1) =  cz*cx+sz*sy*sx; C(1,2) =  -cz*sx+sz*sy*cx;
    C(2,0) = -sy;   C(2,1) =           cy*sx; C(2,2) =            cy*cx;

    return C;
  }
  Eigen::Matrix3d rph2R(Eigen::Vector3d const & x){
    return rph2R(x[0],x[1],x[2]);
  }

  Eigen::Vector3d R2rph(Eigen::Matrix3d const & C){
    double phi = asin(C(2,0));
    double theta = atan2(C(2,1),C(2,2));
    double psi = atan2(C(1,0), C(0,0));
    
    Eigen::Vector3d ret;
    ret[0] = theta;
    ret[1] = -phi;
    ret[2] = psi;

    return ret;
  }
	

  //// Small angle approximation.
  template <typename Scalar_>
  Eigen::Matrix<Scalar_, 3, 3> crossMx(Scalar_ x, Scalar_ y, Scalar_ z){
    Eigen::Matrix<Scalar_, 3, 3> C;
    C(0,0) =  0.0; C(0,1) = -z;   C(0,2) =   y;
    C(1,0) =  z;   C(1,1) =  0.0; C(1,2) =  -x;
    C(2,0) = -y;   C(2,1) =  x;   C(2,2) =   0.0;
    return C;
  }
  template Eigen::Matrix<double, 3, 3> crossMx(double x, double y, double z);
  template Eigen::Matrix<float, 3, 3> crossMx(float x, float y, float z);

  template Eigen::Matrix<double, 3, 3> crossMx(Eigen::MatrixBase<Eigen::Matrix<double, 3, 1> > const &);
  template Eigen::Matrix<float, 3, 3> crossMx(Eigen::MatrixBase<Eigen::Matrix<float, 3, 1> > const &);

  template Eigen::Matrix<double, 3, 3> crossMx(Eigen::MatrixBase<Eigen::Matrix<double, Eigen::Dynamic, 1> > const &);
  template Eigen::Matrix<float, 3, 3> crossMx(Eigen::MatrixBase<Eigen::Matrix<float, Eigen::Dynamic, 1> > const &);

  // Axis Angle rotation.
  Eigen::Matrix3d axisAngle2R(double a, double ax, double ay, double az){
	
    SM_ASSERT_LT_DBG(std::runtime_error,fabs(sqrt(ax*ax + ay*ay + az*az) - 1.0), 1e-4, "The axis is not a unit vector. ||a|| = " << (sqrt(ax*ax + ay*ay + az*az)));
		
    if(a < 1e-12)
      return Eigen::Matrix3d::Identity();
    // e = [ax ay az]
    // e*(e') + (eye(3) - e*(e'))*cos(a) - crossMx(e) * sin(a) = 
    //[         ax^2+ca*(1-ax^2), ax*ay-ca*ax*ay+sa*az, ax*az-ca*ax*az-sa*ay]
    //[ ax*ay-ca*ax*ay-sa*az,         ay^2+ca*(1-ay^2), ay*az-ca*ay*az+sa*ax]
    //[ ax*az-ca*ax*az+sa*ay, ay*az-ca*ay*az-sa*ax,         az^2+ca*(1-az^2)]
    double sa = sin(a);
    double ca = cos(a);
    double ax2 = ax*ax;
    double ay2 = ay*ay;
    double az2 = az*az;
    double const one = double(1);

    Eigen::Matrix3d C;
    C(0,0) =  ax2+ca*(one-ax2);       C(0,1) = ax*ay-ca*ax*ay+sa*az; C(0,2) = ax*az-ca*ax*az-sa*ay;
    C(1,0) =  ax*ay-ca*ax*ay-sa*az; C(1,1) = ay2+ca*(one-ay2);       C(1,2) = ay*az-ca*ay*az+sa*ax;
    C(2,0) =  ax*az-ca*ax*az+sa*ay; C(2,1) = ay*az-ca*ay*az-sa*ax; C(2,2) = az2+ca*(one-az2);

    return C;
  }
  Eigen::Matrix3d axisAngle2R(double x, double y, double z) {
    double a = sqrt(x*x + y*y + z*z);
    if(a < 1e-12)
      return Eigen::Matrix3d::Identity();
		
    double d = 1/a;
    return axisAngle2R(a,x*d, y*d, z*d);
  }

  Eigen::Matrix3d axisAngle2R(Eigen::Vector3d const & x){
    return axisAngle2R(x[0], x[1], x[2]);
  }
	
  Eigen::Vector3d R2AxisAngle(Eigen::Matrix3d const & C){
    // Sometimes, because of roundoff error, the value of tr ends up outside
    // the valid range of arccos. Truncate to the valid range.
    double tr = std::max(-1.0, std::min( (C(0,0) + C(1,1) + C(2,2) - 1.0) * 0.5, 1.0));
    double a = acos( tr ) ;

    Eigen::Vector3d axis;

    if(fabs(a) < 1e-10) {
      return Eigen::Vector3d::Zero();
    }
		
    axis[0] = (C(2,1) - C(1,2));
    axis[1] = (C(0,2) - C(2,0));
    axis[2] = (C(1,0) - C(0,1));
    double n2 = axis.norm();
    if(fabs(n2) < 1e-10)
      return Eigen::Vector3d::Zero();
		
    double scale = -a/n2;
    axis = scale * axis;

    return axis;

  }

  // Utility functions
  double angleMod(double radians){
    return (double)(radians - (SM_2PI * boost::math::round(radians / SM_2PI)));
  }
  double deg2rad(double degrees){
    return (double)(degrees * SM_DEG2RAD);
  }
  double rad2deg(double radians){
    return (double)(radians * SM_RAD2DEG);
  }


  Eigen::Matrix3d Cx(double radians)
  {
    return Rx(-radians);
  }
  Eigen::Matrix3d Cy(double radians)
  {
    return Ry(-radians);
  }
  Eigen::Matrix3d Cz(double radians)
  {
    return Rz(-radians);
  }

   Eigen::Matrix3d rph2C(double x, double y, double z)
   {     
     return rph2R(-x,-y,-z);
   }

   Eigen::Matrix3d rph2C(Eigen::Vector3d const & x)
   {
     return rph2C(x[0],x[1],x[2]);
   }
   Eigen::Matrix3d rph2C(Eigen::VectorXd const & x)
   {
     SM_ASSERT_EQ_DBG(std::runtime_error,x.size(),3,"The input vector must have 3 components");
     return rph2C(x[0],x[1],x[2]);
   }

   Eigen::Matrix3d rph2C(Eigen::MatrixXd const & A, unsigned column)
   {
     SM_ASSERT_EQ_DBG(std::runtime_error,A.rows(),3,"The input matrix must have 3 rows");
     SM_ASSERT_LT_DBG(std::runtime_error,column,A.cols(),"The requested column is out of bounds");
     return rph2C(A(0,column),A(1,column),A(2,column));
   }

   Eigen::Vector3d C2rph(Eigen::MatrixXd const & C)
   {
     SM_ASSERT_EQ_DBG(std::runtime_error,C.rows(),3,"The input matrix must be 3x3");
     SM_ASSERT_EQ_DBG(std::runtime_error,C.cols(),3,"The input matrix must be 3x3");

     Eigen::Vector3d rph;

     rph[1] = asin(C(2,0));
     rph[2] = atan2(-C(1,0),C(0,0));
     rph[0] = atan2(-C(2,1),C(2,2));

     return rph;
   }

   Eigen::Vector3d C2rph(Eigen::Matrix3d const & C)
   {
     Eigen::Vector3d rph;

     rph[1] = asin(C(2,0));
     rph[2] = atan2(-C(1,0),C(0,0));
     rph[0] = atan2(-C(2,1),C(2,2));

     return rph;
   }



  }} // sm::kinematics



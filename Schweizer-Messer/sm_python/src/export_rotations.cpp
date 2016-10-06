#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/kinematics/rotations.hpp>


using namespace boost::python;
using namespace sm::kinematics;

    Eigen::Matrix3d crossMxWrap( const Eigen::Vector3d & v ) {
      return crossMx(v[0],v[1],v[2]);
    }


  void export_rotations()
  {
    using namespace boost::python;

    // Euler angle rotations.
    //   Eigen::Matrix3d Rx(double radians);
    def("Rx", Rx);
    //   Eigen::Matrix3d Ry(double radians);
    def("Ry", Ry);
    //   Eigen::Matrix3d Rz(double radians);
    def("Rz",Rz);
    
    def("Cx", Cx);
    def("Cy", Cy);
    def("Cz", Cz);
  
    Eigen::Matrix3d (*rph2r1)(double, double, double) = &rph2R;
    //   Eigen::Matrix3d rph2r(double x, double y, double z);
    def("rph2R",rph2r1);

    Eigen::Matrix3d (*rph2r2)(Eigen::Vector3d const &) = &rph2R;
    //   Eigen::Matrix3d rph2r(Eigen::Vector3d const & x);
    def("rph2R",rph2r2);

    Eigen::Matrix3d (*rph2C1)(double , double , double ) = &rph2C;
    def("rph2C",rph2C1);

    Eigen::Matrix3d (*rph2C2)(Eigen::Vector3d const &) = &rph2C;
    def("rph2C",rph2C2);

    def("C2rph",C2rph);

    Eigen::Vector3d (*r2rph1)(Eigen::Matrix3d const &) = &R2rph;
    //   Eigen::Vector3d r2rph(Eigen::Matrix3d const & C);
    def("R2rph",r2rph1);
    //   Eigen::Matrix3d rph_S(Eigen::Vector3d const & rph);


    //   // Small angle approximation.
    //   Eigen::Matrix3d crossMx(double x, double y, double z);
    Eigen::Matrix3d (*crossMx1)(double , double , double ) = &crossMx;
    def("crossMx",crossMx1);

    def("crossMx",&crossMxWrap);
    
        

    
    //   Eigen::Matrix3d crossMx(Eigen::Vector3d const & x);
    //Eigen::Matrix3d (*crossMx2)(Eigen::Vector3d const & ) = &crossMx<Eigen::Vector3d>;
    //def("crossMx",crossMx2);
    //   // Axis Angle rotation.

    //   Eigen::Matrix3d axisAngle2r(double x, double y, double z);
    Eigen::Matrix3d (*axisAngle2r1)(double , double , double ) = &axisAngle2R;
    def("axisAngle2R",axisAngle2r1);
    //   Eigen::Matrix3d axisAngle2r(Eigen::Vector3d const & x);
    Eigen::Matrix3d (*axisAngle2r2)(Eigen::Vector3d const & ) = &axisAngle2R;
    def("axisAngle2R",axisAngle2r2);

    //   Eigen::Vector3d r2AxisAngle(Eigen::Matrix3d const & C);
    Eigen::Vector3d (*r2AxisAngle1)(Eigen::Matrix3d const & ) = &R2AxisAngle;
    def("R2AxisAngle",r2AxisAngle1);
    //   // quaternion rotation
    //   vector4 r2quat(Eigen::Matrix3d R);
    //def("r2quat",r2quat);
    //   Eigen::Matrix3d quat2r(vector4 q);
    //def("quat2r",quat2r);
    //   // Utility functions
    //   // Moves a value in radians to within -pi, pi
    //   double angleMod(double radians);
    def("angleMod",angleMod);
    //   double deg2rad(double degrees);
    def("deg2rad",deg2rad);
    //   double rad2deg(double radians);
    def("rad2deg",rad2deg);
}


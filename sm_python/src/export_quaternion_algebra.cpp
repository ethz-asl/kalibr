#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>


using namespace boost::python;
using namespace sm::kinematics;

  void export_quaternion_algebra()
  {
    using namespace boost::python;
    using namespace sm;
    // Eigen::Matrix3d quat2r(Eigen::Vector4d const & q);
    def("quat2r",quat2r,"Build a unit-length quaternion from a rotation matrix");
    // Eigen::Vector4d r2quat(Eigen::Matrix3d const & C);
    def("r2quat",r2quat,"Build a rotation matrix from unit-length quaternion");
    // Eigen::Vector4d axisAngle2quat(Eigen::Vector3d const & a);
    def("axisAngle2quat",axisAngle2quat, "Build a quaternion from a axis/angle representation (the input is the unit-length axis times the angle of rotation about that axis)");
    // Eigen::Vector3d quat2AxisAngle<double>(Eigen::Vector4d const & q);
    def("quat2AxisAngle",quat2AxisAngle<double>, "Build an axis angle from a quaternion. The output is the unit-length axis times the angle of rotation about that axis");
    // Eigen::Matrix4d quatPlus(Eigen::Vector4d const & q);
    def("quatPlus",quatPlus, "Build a q-plus matrix");
    // Eigen::Matrix4d quatOPlus(Eigen::Vector4d const & q);
    def("quatOPlus",quatOPlus, "Build a q-oplus matrix");
    // Eigen::Vector4d quatInv(Eigen::Vector4d const & q);
    def("quatInv",quatInv, "Return the inverse of the quaternion input");
    // void invertQuat(Eigen::Vector4d & q);
    // Eigen::Vector3d qeps(Eigen::Vector4d const & q);
    def("qeps",qeps, "Returns the vector part of the quaternion");
    // double qeta(Eigen::Vector4d const & q);
    def("qeta",qeta, "Returns the scalar part of the quaternion");
    // // For estimation functions to handle a constraint-sensitive minimal parameterization for a quaternion update
    // Eigen::Matrix<double,4,3> quatJacobian(Eigen::Vector4d const & q);
    def("quatJacobian",quatJacobian);
    // Eigen::Vector4d updateQuat(Eigen::Vector4d const & q, Eigen::Vector3d const & dq);
    def("updateQuat",updateQuat);
    def("quatIdentity",&quatIdentity, "Return the identity quaternion");
    def("quatRandom",&quatRandom, "Return a random quaternion");
    def("quatS",&quatS);
    def("quatInvS",&quatInvS);
    // Eigen::Vector3d quatRotate(Eigen::Vector4d const & q_a_b, Eigen::Vector3d const & v_b);
    def("quatRotate", &quatRotate);
    // Eigen::Vector4d qoplus(Eigen::Vector4d const & q, Eigen::Vector4d const & p);
    def("qoplus", &qoplus);
    // Eigen::Vector4d qplus(Eigen::Vector4d const & q, Eigen::Vector4d const & p);
    def("qplus", &qplus);
    def("qlog", &qlog);
    def("qexp", &qexp);
    def("qslerp", &qslerp);
    def("lerp", &lerp);
      ;
  }

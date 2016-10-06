#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>


using namespace boost::python;
using namespace sm::kinematics;

Eigen::Vector4d toHomogeneousWrap(const Eigen::Vector3d & v)
{
  return sm::kinematics::toHomogeneous(v);
}


Eigen::Vector3d fromHomogeneousWrap(const Eigen::Vector4d & v)
{
  return sm::kinematics::fromHomogeneous(v);
}

  void export_homogeneous_coordinates()
  {
    
    def("toHomogeneousJacobian",&toHomogeneousJacobian);
    //Eigen::Matrix<double,4,3> toHomogeneousJacobian(const Eigen::Vector3d & v);
    //Eigen::Vector4d toHomogeneous(const Eigen::Vector3d & v, Eigen::Matrix<double,4,3> * jacobian = NULL);
    def("toHomogeneous",&toHomogeneousWrap);
    //Eigen::Matrix<double,3,4> fromHomogeneousJacobian(const Eigen::Vector4d & v);
    def("fromHomogeneousJacobian", &fromHomogeneousJacobian);
    //Eigen::Vector3d fromHomogeneous(const Eigen::Vector4d & v, Eigen::Matrix<double,3,4> * jacobian = NULL);
    def("fromHomogeneous", &fromHomogeneousWrap);
    //Eigen::MatrixXd toHomogeneousColumns(const Eigen::MatrixXd & M);
    def("toHomogeneousColumns", &toHomogeneousColumns);
    //Eigen::MatrixXd fromHomogeneousColumns(const Eigen::MatrixXd & M);
    def("fromHomogeneousColumns", &fromHomogeneousColumns);
  }

#ifndef SM_MATH_HOMOGENEOUS_COORDINATES_HPP
#define SM_MATH_HOMOGENEOUS_COORDINATES_HPP

#include <Eigen/Core>

namespace sm { namespace kinematics {

    Eigen::Matrix<double,4,3> toHomogeneousJacobian(const Eigen::Vector3d & v);
    Eigen::Vector4d toHomogeneous(const Eigen::Vector3d & v, Eigen::Matrix<double,4,3> * jacobian = NULL);


    Eigen::Matrix<double,3,4> fromHomogeneousJacobian(const Eigen::Vector4d & v);
    Eigen::Vector3d fromHomogeneous(const Eigen::Vector4d & v, Eigen::Matrix<double,3,4> * jacobian = NULL);


    Eigen::MatrixXd toHomogeneousColumns(const Eigen::MatrixXd & M);
    Eigen::MatrixXd fromHomogeneousColumns(const Eigen::MatrixXd & M);

    template<int N>
    Eigen::Matrix<double,N,1> normalize(const Eigen::Matrix<double,N,1> & v)
    {
      return v/v.norm();
    }

    template<int N>
    Eigen::Matrix<double,N,1> normalizeAndJacobian(const Eigen::Matrix<double,N,1> & v, Eigen::Matrix<double,N,N> & outJacobian)
    {
      double recip_s_vtv = 1.0/v.norm();
      
      outJacobian = (Eigen::Matrix<double,N,N>::Identity() * recip_s_vtv) - (recip_s_vtv*recip_s_vtv*recip_s_vtv)*v*v.transpose();
      
      return v*recip_s_vtv;
    }

    /// \brief to the matrix that implements "plus" for homogeneous coordinates.
    ///        this operation has the same result as adding the corresponding Euclidean points.
    Eigen::Matrix4d toHomogeneousPlus(const Eigen::Vector4d & ph);

    /// \brief to the matrix that implements "minus" for homogeneous coordinates.
    ///        this operation has the same result as adding the corresponding Euclidean points.
    Eigen::Matrix4d toHomogeneousMinus(const Eigen::Vector4d & ph);

  }} // namespace sm::kinematics

#endif /* SM_MATH_HOMOGENEOUS_COORDINATES_HPP */

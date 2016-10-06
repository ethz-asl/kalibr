#ifndef SM_ROTATIONAL_KINEMATICS_EXYZ_HPP
#define SM_ROTATIONAL_KINEMATICS_EXYZ_HPP

#include "RotationalKinematics.hpp"

namespace sm { namespace kinematics {
    
    class EulerAnglesZYX : public RotationalKinematics
    {
    public:
      virtual ~EulerAnglesZYX();
      // Three functions that must be implemented.
      virtual Eigen::Matrix3d parametersToRotationMatrix(const Eigen::Vector3d & parameters, Eigen::Matrix3d * S = NULL) const;
      virtual Eigen::Vector3d rotationMatrixToParameters(const Eigen::Matrix3d & rotationMatrix) const;
      virtual Eigen::Matrix3d parametersToSMatrix(const Eigen::Vector3d & parameters) const;
      virtual Eigen::Vector3d angularVelocityAndJacobian(const Eigen::Vector3d & p, const Eigen::Vector3d & pdot, Eigen::Matrix<double,3,6> * Jacobian) const;
    
    private:
      // A helper function
      Eigen::Matrix3d buildSMatrix(double sz, double cz, double sy, double cy) const;
    };

  }} // namespace sm::kinematics


#endif 

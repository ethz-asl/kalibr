#ifndef _ROTATIONVECTOR_H_
#define _ROTATIONVECTOR_H_

#include "RotationalKinematics.hpp"

namespace sm { namespace kinematics {

    class RotationVector : public RotationalKinematics
    {
    public:
      virtual ~RotationVector();
      virtual Eigen::Matrix3d parametersToRotationMatrix(const Eigen::Vector3d & parameters, Eigen::Matrix3d * S = NULL) const;
      virtual Eigen::Vector3d rotationMatrixToParameters(const Eigen::Matrix3d & rotationMatrix) const;
      virtual Eigen::Matrix3d parametersToSMatrix(const Eigen::Vector3d & parameters) const;
      virtual Eigen::Vector3d angularVelocityAndJacobian(const Eigen::Vector3d & p, const Eigen::Vector3d & pdot, Eigen::Matrix<double,3,6> * Jacobian) const;    
      // Fabio:
	  Eigen::Matrix3d parametersToInverseSMatrix(const Eigen::Vector3d & parameters) const;
	};

  }} // sm::kinematics

#endif /* _ROTATIONVECTOR_H_ */

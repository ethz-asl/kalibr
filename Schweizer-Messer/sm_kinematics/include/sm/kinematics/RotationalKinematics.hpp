#ifndef SM_ROTATIONAL_KINEMATICS_HPP
#define SM_ROTATIONAL_KINEMATICS_HPP

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <sm/assert_macros.hpp>
#include "rotations.hpp"

namespace sm { namespace kinematics {
    class RotationalKinematics
    {
    public:
      SM_DEFINE_EXCEPTION(Exception,std::runtime_error);
      virtual ~RotationalKinematics();
      typedef ::boost::shared_ptr<RotationalKinematics> Ptr;
      typedef ::boost::shared_ptr<const RotationalKinematics> ConstPtr;

      // Three functions that must be implemented.
      virtual Eigen::Matrix3d parametersToRotationMatrix(const Eigen::Vector3d & parameters, Eigen::Matrix3d * S = NULL) const = 0;
      virtual Eigen::Vector3d rotationMatrixToParameters(const Eigen::Matrix3d & rotationMatrix) const = 0;
      virtual Eigen::Matrix3d parametersToSMatrix(const Eigen::Vector3d & parameters) const = 0;
      virtual Eigen::Vector3d angularVelocityAndJacobian(const Eigen::Vector3d & /* p */, const Eigen::Vector3d & /* pdot */, Eigen::Matrix<double,3,6> * /* Jacobian */) const {SM_THROW(Exception,"Not implemented"); return Eigen::Vector3d::Zero();}
    };



  }} // namespace sm::kinematics


#endif

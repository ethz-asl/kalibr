#ifndef _EULERANGLESYAWPITCHROLL_H_
#define _EULERANGLESYAWPITCHROLL_H_

#include "RotationalKinematics.hpp"

namespace sm { namespace kinematics {

  /**
   * @class EulerAnglesYawPitchRoll
   * This is the standard Rotation sequence for aerospace vehicles.
   * 
   * Passing this the parameters [yaw, pitch, roll] of a vehicle frame, F_v,
   * with respect to an inertial frame, F_i, returns the rotation matrix,
   * C_iv, the rotation that takes vectors from F_v, to F_i.
   *
   * Similarly passing the rotation matrix C_iv to this class will cause
   * return the vector [heading, pitch, roll] where these represent the usual
   * meaning with respect to aerospace vehicles.
   *
   * This assumes that the vehicle frame has:
   * x-axis pointing to the front.
   * y-axis pointing to the left.
   * z-axis pointing up.
   *
   * heading: positive is vehicle nose left (counter clockwise).
   * pitch:   positive is vehicle nose down.
   * roll:    positive is left wing up.
   * 
   */
    class EulerAnglesYawPitchRoll : public RotationalKinematics
    {
    public:
      virtual ~EulerAnglesYawPitchRoll();
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


#endif /* _EULERANGLESYAWPITCHROLL_H_ */

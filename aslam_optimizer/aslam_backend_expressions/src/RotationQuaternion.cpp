
#include <aslam/backend/RotationQuaternion.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>


namespace aslam {
  namespace backend {

    RotationQuaternion::RotationQuaternion(const Eigen::Vector4d & q) : _q(q), _p_q(q), _C(sm::kinematics::quat2r(q)) {}

    RotationQuaternion::RotationQuaternion(const Eigen::Matrix3d& C) :
        _q(sm::kinematics::r2quat(C)),
        _p_q(sm::kinematics::r2quat(C)),
        _C(C) {
    }

    RotationQuaternion::~RotationQuaternion(){}

      
    /// \brief Revert the last state update.
    void RotationQuaternion::revertUpdateImplementation()
    {
      _q = _p_q;
      _C = sm::kinematics::quat2r(_q);
    }
    
    /// \brief Update the design variable.
    void RotationQuaternion::updateImplementation(const double * dp, int size) 
    {
      SM_ASSERT_EQ_DBG(Exception, size, 3, "Incorrect update size");
      _p_q = _q;
      Eigen::Map<const Eigen::Vector3d> dpv(dp);
      _q = sm::kinematics::updateQuat(_q, dpv);
      _C = sm::kinematics::quat2r(_q);
    }
    
    int RotationQuaternion::minimalDimensionsImplementation() const
    { 
      return 3; 
    }
    
    Eigen::Matrix3d RotationQuaternion::toRotationMatrixImplementation() const
    {
      return _C;
    }

    void RotationQuaternion::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      outJacobians.add( const_cast<RotationQuaternion *>(this), Eigen::Matrix3d::Identity() );
    }

    void RotationQuaternion::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      outJacobians.add( const_cast<RotationQuaternion*>(this), applyChainRule );
    }

    RotationExpression RotationQuaternion::toExpression()
    {
      return RotationExpression(this);
    }

    void RotationQuaternion::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      designVariables.insert(const_cast<RotationQuaternion*>(this));
    }

    void RotationQuaternion::getParametersImplementation(
        Eigen::MatrixXd& value) const {
      value = _q;
    }

    void RotationQuaternion::setParametersImplementation(
        const Eigen::MatrixXd& value) {
      _p_q = _q;
      _q = value;
      _C = sm::kinematics::quat2r(_q);
    }

    void RotationQuaternion::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
    {
     SM_ASSERT_TRUE(aslam::InvalidArgumentException, (xHat.rows() == 4)&&(xHat.cols() == 1), "The dimension of xHat does not conform to a 4x1 quaternion vector");
     outDifference = sm::kinematics::qlog(sm::kinematics::qplus(_q, sm::kinematics::quatInv(xHat)));
    }

    void RotationQuaternion::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const
    {
     minimalDifferenceImplementation(xHat, outDifference);
     // compute the evaluation point:
     Eigen::Vector4d quatHat(xHat(0,0),xHat(1,0),xHat(2,0),xHat(3,0));
     Eigen::Vector4d evalPoint= sm::kinematics::qplus(_q, sm::kinematics::quatInv(quatHat));
     outJacobian = sm::kinematics::quatLogJacobian2(evalPoint)*sm::kinematics::quatJacobian(evalPoint); //???
    }

  } // namespace backend
} // namespace aslam


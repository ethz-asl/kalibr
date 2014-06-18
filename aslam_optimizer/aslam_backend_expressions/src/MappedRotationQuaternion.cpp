
#include <aslam/backend/MappedRotationQuaternion.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>


namespace aslam {
  namespace backend {

    MappedRotationQuaternion::MappedRotationQuaternion(double * q) : _q(q), _p_q(_q), _C(sm::kinematics::quat2r(_q)) {}

    MappedRotationQuaternion::~MappedRotationQuaternion(){}

      
    /// \brief Revert the last state update.
    void MappedRotationQuaternion::revertUpdateImplementation()
    {
      _q = _p_q;
      _C = sm::kinematics::quat2r(_q);
    }
    
    /// \brief Update the design variable.
    void MappedRotationQuaternion::updateImplementation(const double * dp, int size) 
    {
      SM_ASSERT_EQ_DBG(Exception, size, 3, "Incorrect update size");
      _p_q = _q;
      Eigen::Map<const Eigen::Vector3d> dpv(dp);
      _q = sm::kinematics::updateQuat(_q, dpv);
      _C = sm::kinematics::quat2r(_q);
    }
    
    int MappedRotationQuaternion::minimalDimensionsImplementation() const
    { 
      return 3; 
    }
    
    Eigen::Matrix3d MappedRotationQuaternion::toRotationMatrixImplementation() const
    {
      return _C;
    }

    void MappedRotationQuaternion::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      outJacobians.add( const_cast<MappedRotationQuaternion *>(this), Eigen::Matrix3d::Identity() );
    }

    void MappedRotationQuaternion::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      outJacobians.add( const_cast<MappedRotationQuaternion*>(this), applyChainRule );
    }

    RotationExpression MappedRotationQuaternion::toExpression()
    {
      return RotationExpression(this);
    }

    void MappedRotationQuaternion::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      designVariables.insert(const_cast<MappedRotationQuaternion*>(this));
    }

    void MappedRotationQuaternion::getParametersImplementation(
        Eigen::MatrixXd& value) const {
      value = _q;
    }

    void MappedRotationQuaternion::setParametersImplementation(
        const Eigen::MatrixXd& value) {
      _p_q = _q;
      _q = value;
      _C = sm::kinematics::quat2r(_q);
    }

    void MappedRotationQuaternion::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
    {
     SM_ASSERT_TRUE(aslam::InvalidArgumentException, (xHat.rows() == 4)&&(xHat.cols() == 1), "The dimension of xHat does not conform to a 4x1 quaternion vector");
     outDifference = sm::kinematics::qlog(sm::kinematics::qplus(_p_q, sm::kinematics::quatInv(xHat)));
    }

    void MappedRotationQuaternion::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const
    {
     minimalDifferenceImplementation(xHat, outDifference);
     // compute the evaluation point:
     Eigen::Vector4d quatHat(xHat(0,0),xHat(1,0),xHat(2,0),xHat(3,0));
     Eigen::Vector4d evalPoint= sm::kinematics::qplus(_p_q, sm::kinematics::quatInv(quatHat));
     outJacobian = sm::kinematics::quatLogJacobian(evalPoint)*sm::kinematics::quatJacobian(evalPoint); //???
    }

  } // namespace backend
} // namespace aslam


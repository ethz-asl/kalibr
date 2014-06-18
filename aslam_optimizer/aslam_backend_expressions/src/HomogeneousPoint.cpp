#include <aslam/backend/HomogeneousPoint.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>


namespace aslam {
  namespace backend {
    HomogeneousPoint::HomogeneousPoint(const Eigen::Vector4d & p) :
      _p(p), _p_p(p)
    {
      double recipPnorm = 1.0/p.norm();
      _p *= recipPnorm;
      _p_p *= recipPnorm;
    }
    HomogeneousPoint::~HomogeneousPoint()
    {

    }

    /// \brief Revert the last state update.
    void HomogeneousPoint::revertUpdateImplementation()
    {
      _p = _p_p;
    }
    
    /// \brief Update the design variable.
  void HomogeneousPoint::updateImplementation(const double * dp, int /* size */)
    {
      _p_p = _p;
      Eigen::Map<const Eigen::Vector3d> dpv(dp);
      _p = sm::kinematics::updateQuat(_p, dpv);      

    }
    
    /// \brief the size of an update step
    int HomogeneousPoint::minimalDimensionsImplementation() const
    {
      return 3;
    }
    
    Eigen::Vector4d HomogeneousPoint::toHomogeneousImplementation() const
    {
      return _p;
    }
    
    void HomogeneousPoint::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      outJacobians.add(const_cast<HomogeneousPoint *>(this), sm::kinematics::quatInvS(_p));
    }
    
    void HomogeneousPoint::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      outJacobians.add(const_cast<HomogeneousPoint *>(this), applyChainRule * sm::kinematics::quatInvS(_p));
    }
      
    void HomogeneousPoint::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      designVariables.insert(const_cast<HomogeneousPoint *>(this));
    }

    HomogeneousExpression HomogeneousPoint::toExpression()
    {
      return HomogeneousExpression(this);
    }

    void HomogeneousPoint::getParametersImplementation(Eigen::MatrixXd& value)
        const {
      value = _p;
    }

    void HomogeneousPoint::setParametersImplementation(const Eigen::MatrixXd&
        value) {
      _p_p = _p;
      _p = value;
    }

    void HomogeneousPoint::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
    {
     SM_ASSERT_TRUE(aslam::InvalidArgumentException, (xHat.rows() == 4)&&(xHat.cols() == 1), "The dimension of xHat does not conform to a 4x1 quaternion vector");
     outDifference = sm::kinematics::qlog(sm::kinematics::qplus(_p, sm::kinematics::quatInv(xHat)));
    }

    void HomogeneousPoint::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const
    {
     minimalDifferenceImplementation(xHat, outDifference);
     // compute the evaluation point:
     Eigen::Vector4d quatHat; quatHat(0) = xHat(0,0); quatHat(1) = xHat(1,0); quatHat(2) = xHat(2,0); quatHat(3) = xHat(3,0);
     Eigen::Vector4d evalPoint= sm::kinematics::qplus(_p, sm::kinematics::quatInv(quatHat));
     outJacobian = sm::kinematics::quatLogJacobian(evalPoint)*sm::kinematics::quatJacobian(evalPoint);
    }

  } // namespace backend
} // namespace aslam

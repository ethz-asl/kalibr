#include <aslam/backend/MappedHomogeneousPoint.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>


namespace aslam {
  namespace backend {
    MappedHomogeneousPoint::MappedHomogeneousPoint(double * p) :
      _p(p), _p_p(_p)
    {
      double recipPnorm = 1.0/_p.norm();
      _p *= recipPnorm;
      _p_p *= recipPnorm;
    }
    MappedHomogeneousPoint::~MappedHomogeneousPoint()
    {

    }

    /// \brief Revert the last state update.
    void MappedHomogeneousPoint::revertUpdateImplementation()
    {
      _p = _p_p;
    }
    
    /// \brief Update the design variable.
  void MappedHomogeneousPoint::updateImplementation(const double * dp, int /* size */)
    {
      _p_p = _p;
      Eigen::Map<const Eigen::Vector3d> dpv(dp);
      _p = sm::kinematics::updateQuat(_p, dpv);      

    }
    
    /// \brief the size of an update step
    int MappedHomogeneousPoint::minimalDimensionsImplementation() const
    {
      return 3;
    }
    
    Eigen::Vector4d MappedHomogeneousPoint::toHomogeneousImplementation() const
    {
      return _p;
    }
    
    void MappedHomogeneousPoint::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      outJacobians.add(const_cast<MappedHomogeneousPoint *>(this), sm::kinematics::quatInvS(_p));
    }
    
    void MappedHomogeneousPoint::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      outJacobians.add(const_cast<MappedHomogeneousPoint *>(this), applyChainRule * sm::kinematics::quatInvS(_p));
    }
      
    void MappedHomogeneousPoint::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      designVariables.insert(const_cast<MappedHomogeneousPoint *>(this));
    }

    HomogeneousExpression MappedHomogeneousPoint::toExpression()
    {
      return HomogeneousExpression(this);
    }

    void MappedHomogeneousPoint::getParametersImplementation(Eigen::MatrixXd& value)
        const {
      value = _p;
    }

    void MappedHomogeneousPoint::setParametersImplementation(const Eigen::MatrixXd&
        value) {
      _p_p = _p;
      _p = value;
    }

    void MappedHomogeneousPoint::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
    {
     SM_ASSERT_TRUE(aslam::InvalidArgumentException, (xHat.rows() == 4)&&(xHat.cols() == 1), "The dimension of xHat does not conform to a 4x1 quaternion vector");
     outDifference = sm::kinematics::qlog(sm::kinematics::qplus(_p_p, sm::kinematics::quatInv(xHat)));
    }

    void MappedHomogeneousPoint::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const
    {
     minimalDifferenceImplementation(xHat, outDifference);
     // compute the evaluation point:
     Eigen::Vector4d quatHat; quatHat(0) = xHat(0,0); quatHat(1) = xHat(1,0); quatHat(2) = xHat(2,0); quatHat(3) = xHat(3,0);
     Eigen::Vector4d evalPoint= sm::kinematics::qplus(_p_p, sm::kinematics::quatInv(quatHat));
     outJacobian = sm::kinematics::quatLogJacobian(evalPoint)*sm::kinematics::quatJacobian(evalPoint);
    }

  } // namespace backend
} // namespace aslam

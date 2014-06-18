#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>

namespace aslam {
  namespace backend {
    EuclideanPoint::EuclideanPoint(const Eigen::Vector3d & p) :
      _p(p), _p_p(p)
    {

    }
    EuclideanPoint::~EuclideanPoint()
    {

    }

    /// \brief Revert the last state update.
    void EuclideanPoint::revertUpdateImplementation()
    {
      _p = _p_p;
    }
    
    /// \brief Update the design variable.
    void EuclideanPoint::updateImplementation(const double * dp, int size)
    {
      static_cast<void>(size); // used depending on NDEBUG
      SM_ASSERT_EQ_DBG(std::runtime_error, size, 3, "Incorrect size");
      _p_p = _p;
      
      Eigen::Map< const Eigen::Vector3d > dpv(dp);
      _p += dpv;

    }
    
    /// \brief the size of an update step
    int EuclideanPoint::minimalDimensionsImplementation() const
    {
      return 3;
    }
    
    Eigen::Vector3d EuclideanPoint::toEuclideanImplementation() const
    {
      return _p;
    }
    
    void EuclideanPoint::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      outJacobians.add(const_cast<EuclideanPoint *>(this), Eigen::Matrix3d::Identity());
    }
    
    void EuclideanPoint::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      outJacobians.add(const_cast<EuclideanPoint *>(this), applyChainRule);
    }

    void EuclideanPoint::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      designVariables.insert(const_cast<EuclideanPoint *>(this));
    }

    EuclideanExpression EuclideanPoint::toExpression()
    {
      return EuclideanExpression(this);
    }

  HomogeneousExpression EuclideanPoint::toHomogeneousExpression() {
    return EuclideanExpression(this).toHomogeneousExpression();
  }

    void EuclideanPoint::getParametersImplementation(
        Eigen::MatrixXd& value) const {
      value = _p;
    }

    void EuclideanPoint::setParametersImplementation(
        const Eigen::MatrixXd& value) {
      _p_p = _p;
      _p = value;
    }

    void EuclideanPoint::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
    {
     SM_ASSERT_TRUE(aslam::InvalidArgumentException, (xHat.rows() == 3) && (xHat.cols() == 1), "xHat has incompatible dimensions");
     outDifference = _p - xHat;
    }

    void EuclideanPoint::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const
    {
     minimalDifferenceImplementation(xHat, outDifference);
     outJacobian = Eigen::Matrix3d::Identity();
    }

  } // namespace backend
} // namespace aslam

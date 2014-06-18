#include <aslam/backend/EuclideanDirection.hpp>

namespace aslam {
    namespace backend {
        EuclideanDirection::EuclideanDirection(const Eigen::Vector3d & direction)             
        {
            _magnitude = direction.norm();
            SM_ASSERT_GT(Exception, _magnitude, 0.0, "The pointing vector must have a magnitude greater than zero");

            // Decompose the pointing vector into a rotation matrix.
            Eigen::Vector3d unit = direction / _magnitude;
            double y = asin(-unit[0]);
            double x = atan2(unit[1], unit[2]);
            Eigen::Vector3d parameters(0.0, y, x);
            _C = _ypr.parametersToRotationMatrix(parameters).transpose();
            _p_C = _C;
        }

        EuclideanDirection::~EuclideanDirection()
        {

        }


        /// \brief Revert the last state update.
        void EuclideanDirection::revertUpdateImplementation()
        {
            _C = _p_C;
        }


        /// \brief Update the design variable.
    void EuclideanDirection::updateImplementation(const double * dp, int /* size */)
        {
            Eigen::Vector3d parameters(0.0, dp[0], dp[1]);
            _p_C = _C;
            _C = (_C * _ypr.parametersToRotationMatrix(parameters).transpose()).eval();
        }


        /// \brief the size of an update step
        int EuclideanDirection::minimalDimensionsImplementation() const
        {
            return 2;
        }


        EuclideanExpression EuclideanDirection::toExpression()
        {
            return EuclideanExpression(this);
        }

    
        Eigen::Vector3d EuclideanDirection::toEuclideanImplementation() const
        {
            return _magnitude * _C.col(2);
        }


        void EuclideanDirection::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd J(3,2);
            J.setZero();
            J.col(0) = -_C.col(0) * _magnitude;
            J.col(1) = _C.col(1) * _magnitude;
            outJacobians.add(const_cast<EuclideanDirection*>(this), J);
        }


        void EuclideanDirection::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            Eigen::MatrixXd J(3,2);
            J.setZero();
            J.col(0) = -_C.col(0) * _magnitude;
            J.col(1) = _C.col(1) * _magnitude;
            outJacobians.add(const_cast<EuclideanDirection*>(this), applyChainRule * J);
        }


        void EuclideanDirection::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            designVariables.insert(const_cast<EuclideanDirection*>(this));
        }

      void EuclideanDirection::getParametersImplementation(
          Eigen::MatrixXd& value) const {
        value = _C;
      }

      void EuclideanDirection::setParametersImplementation(
          const Eigen::MatrixXd& value) {
        _p_C = _C;
        _C = value;
      }

      void EuclideanDirection::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
      {
        SM_THROW(std::runtime_error, "Not implemented");
        SM_ASSERT_TRUE(aslam::InvalidArgumentException, (xHat.rows()==3)&&(xHat.cols()==3), "xHat does have incompatible dimensions");
        outDifference = sm::kinematics::R2AxisAngle(_C * xHat.transpose());
      }

    void EuclideanDirection::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& /* xHat */, Eigen::VectorXd& /* outDifference */, Eigen::MatrixXd& /* outJacobian */) const
     {
       SM_THROW(std::runtime_error, "Not implemented");
       //minimalDifferenceImplementation(xHat, outDifference);
		 // outJacobian = ??
       // outJacobian.setIdentity(3,3);
     }

    } // namespace backend
} // namespace aslam

#include <aslam/backend/Scalar.hpp>


namespace aslam {
    namespace backend {
        Scalar::Scalar(const double & p) :
            _p(p), _p_p(p)
        {

        }
        Scalar::~Scalar()
        {

        }

        /// \brief Revert the last state update.
        void Scalar::revertUpdateImplementation()
        {
            _p = _p_p;
        }
    
        /// \brief Update the design variable.
    void Scalar::updateImplementation(const double * dp, int /* size */)
        {
            _p_p = _p;
      
            _p += dp[0];

        }
    
        /// \brief the size of an update step
        int Scalar::minimalDimensionsImplementation() const
        {
            return 1;
        }
    
        double Scalar::toScalarImplementation() const
        {
            return _p;
        }
    
        void Scalar::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
        {
            Eigen::Matrix<double,1,1> J;
            J(0,0) = 1;
            outJacobians.add(const_cast<Scalar *>(this), J);
        }
    
        void Scalar::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            outJacobians.add(const_cast<Scalar *>(this), applyChainRule);
        }

        void Scalar::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
        {
            designVariables.insert(const_cast<Scalar *>(this));
        }

        ScalarExpression Scalar::toExpression()
        {
            return ScalarExpression(this);
        }

        Eigen::MatrixXd Scalar::getParameters()
        {
        	Eigen::Matrix<double, 1,1> M;
        	M << toScalar();
        	return M;
        }

      void Scalar::getParametersImplementation(Eigen::MatrixXd& value) const {
        Eigen::Matrix<double, 1, 1> valueMat;
        valueMat << _p;
        value = valueMat;
      }

      void Scalar::setParametersImplementation(const Eigen::MatrixXd& value) {
        _p_p = _p;
        _p = value(0, 0);
      }

      void Scalar::minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
      {
		 SM_ASSERT_TRUE(aslam::InvalidArgumentException, (xHat.rows() == 1)&(xHat.cols() == 1), "xHat has incompatible dimensions");
		 outDifference = Eigen::VectorXd(1);
		 outDifference(0) = _p - xHat(0,0);
      }

      void Scalar::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const
      {
		 minimalDifferenceImplementation(xHat, outDifference);
		 outJacobian = Eigen::MatrixXd::Identity(1,1);
      }

    } // namespace backend
} // namespace aslam

#ifndef ASLAM_BACKEND_EUCLIDEAN_DIRECTION_HPP
#define ASLAM_BACKEND_EUCLIDEAN_DIRECTION_HPP

#include "EuclideanExpressionNode.hpp"
#include "EuclideanExpression.hpp"
#include <aslam/backend/DesignVariable.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

namespace aslam {
    namespace backend {
    
        class EuclideanDirection : public EuclideanExpressionNode, public DesignVariable
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

        SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

            EuclideanDirection(const Eigen::Vector3d & direction);
            virtual ~EuclideanDirection();

            /// \brief Revert the last state update.
            virtual void revertUpdateImplementation();

            /// \brief Update the design variable.
            virtual void updateImplementation(const double * dp, int size);

            /// \brief the size of an update step
            virtual int minimalDimensionsImplementation() const;

            EuclideanExpression toExpression();
        private:
            virtual Eigen::Vector3d toEuclideanImplementation() const;

            virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const;

            virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;

            virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const;

			/// Returns the content of the design variable
			virtual void getParametersImplementation(Eigen::MatrixXd& value) const;

			/// Sets the content of the design variable
			virtual void setParametersImplementation(const Eigen::MatrixXd& value);

		    /// Computes the minimal distance in tangent space between the current value of the DV and xHat
		    virtual void minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const;

		    /// Computes the minimal distance in tangent space between the current value of the DV and xHat and the jacobian
		    virtual void minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const;

            Eigen::Matrix3d _C;
            Eigen::Matrix3d _p_C;

            double _magnitude;

            sm::kinematics::EulerAnglesYawPitchRoll _ypr;

        };
        
    } // namespace backend    
} // namespace aslam


#endif /* ASLAM_BACKEND_UNIT_EUCLIDEAN_HPP */

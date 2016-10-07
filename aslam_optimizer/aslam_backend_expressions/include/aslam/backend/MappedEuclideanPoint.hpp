#ifndef ASLAM_BACKEND_MAPPED_EUCLIDEAN_POINT_HPP
#define ASLAM_BACKEND_MAPPED_EUCLIDEAN_POINT_HPP

#include "EuclideanExpressionNode.hpp"
#include "EuclideanExpression.hpp"
#include <aslam/backend/DesignVariable.hpp>


namespace aslam {
  namespace backend {
    
    class MappedEuclideanPoint : public EuclideanExpressionNode, public DesignVariable
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      MappedEuclideanPoint(double * p);
      virtual ~MappedEuclideanPoint();

      /// \brief Revert the last state update.
      virtual void revertUpdateImplementation() override;

      /// \brief Update the design variable.
      virtual void updateImplementation(const double * dp, int size) override;

      /// \brief the size of an update step
      virtual int minimalDimensionsImplementation() const override;

      EuclideanExpression toExpression();

        void set(const Eigen::Vector3d & p){ _p = p; _p_p = _p; }
    private:
      virtual Eigen::Vector3d evaluateImplementation() const override;

      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const override;

      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const override;

      /// Returns the content of the design variable
      virtual void getParametersImplementation(Eigen::MatrixXd& value) const override;

      /// Sets the content of the design variable
      virtual void setParametersImplementation(const Eigen::MatrixXd& value) override;

      /// Computes the minimal distance in tangent space between the current value of the DV and xHat
      virtual void minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const override;

      /// Computes the minimal distance in tangent space between the current value of the DV and xHat and the jacobian
      virtual void minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const override;

      /// \brief The current value of the design variable.
      Eigen::Map<Eigen::Vector3d> _p;

      /// \brief The previous version of the design variable.
      Eigen::Vector3d _p_p;


    };
    
  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_EUCLIDEAN_POINT_HPP */
